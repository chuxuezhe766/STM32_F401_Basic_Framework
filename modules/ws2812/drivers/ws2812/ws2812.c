// Ties to the 'compatible = "custom, ws2812"' node in the Devicetree
#define DT_DRV_COMPAT custom_ws2812

#include <errno.h>
#include <zephyr/logging/log.h>

#include "ws2812.h"

// Private macro
/* 定时器时钟 84MHz: TIM5 挂在 APB1, 定时器时钟 = 84MHz
 * 1 tick = 1/84MHz = 11.9ns
 * WS2812 时序: T0H=0.42us, T1H=0.83us, 位周期=1.25us
 */
#define WS2812_PWM_PERIOD  105  /* 位周期（计数周期ARR）: 105 ticks = 1.25us */
#define WS2812_ONE_PWM      70  /* 1码高电平: 70 ticks = 0.83us */
#define WS2812_ZERO_PWM     35  /* 0码高电平: 35 ticks = 0.42us */
#define WS2812_RESET_DELAY 80
//复位字节数量
#define WS2812_RESET_SLOTS  1

/*
 * STM32 TIMx CCR1~CCR4 偏移：
 * CCR1 = 0x34
 * CCR2 = 0x38
 * CCR3 = 0x3C
 * CCR4 = 0x40
 */
#define TIM_CCR_OFFSET(channel) (0x34U + ((channel) - 1U) * 4U)
#define TIM_DIER_OFFSET         (0x0CU) /* TIM_DIER 寄存器偏移 */
/* CC3 比较事件 DMA 请求(bit11): CNT 计数到 CCR3 时触发 DMA 写下一个值,
 * 下个周期开始立即生效, 无延迟无竞态(标准 WS2812 PWM+DMA 例程做法)。
 * 对应 DMA 请求线: TIM5_CH3 = DMA1 Stream 0, Channel 6。
 */
#define TIM_DIER_CC3DE          BIT(11) /* CC3 DMA request enable */

// Enable Logging at LOG_LEVEL_INF
LOG_MODULE_REGISTER(ws2812, LOG_LEVEL_INF);

// Configure
struct ws2812_pwm_dma_cfg {
    /* PWM输出 */
    struct pwm_dt_spec pwm;
    /* DMA */
    const struct device *dma_dev;
    uint32_t dma_channel;
    uint32_t dma_slot;
    uintptr_t pwm_base;
    uintptr_t gpioa_base; /* PA2 OSPEEDR 高速配置用 */
    /* WS2812灯带参数*/
    size_t length;
    uint8_t num_colors;
    const uint8_t *color_mapping;
    /* PWM值*/
    uint16_t pwm_period;        /* PWM周期对应的ARR值*/
    uint16_t one_pwm;           /* 1码对应的CCR比较值*/
    uint16_t zero_pwm;          /* 0码对应的CCR比较值*/
    /* 复位延时*/
    uint32_t reset_delay_us;
    /* DMA缓冲*/
    uint16_t *dma_buf;
    size_t dma_buf_len;

};

struct ws2812_pwm_dma_data {
    struct k_mutex lock;        /* 防止并发刷新灯带*/
    struct k_sem dma_done;      /* DMA发送完成信号量*/
    struct dma_config dma_cfg;  /* DMA 静态配置，init 中配置一次 */
    struct dma_block_config dma_blk; /* DMA 块描述，block_size 每次 update 更新 */
};

/* 当前正在传输的 cfg, 供 DMA 完成回调里 dma_stop() 使用。
 * 单实例: 当前 DTS 只有一个 ws2812 节点, 全局指针足够。
 */
static const struct ws2812_pwm_dma_cfg *g_active_cfg;

static void ws2812_dma_callback(const struct device *dma_dev,
                                void *user_data,
                                uint32_t channel,
                                int status)
{
    struct ws2812_pwm_dma_data *data = user_data;

    /* 循环模式下 Zephyr STM32 DMA 驱动会额外使能半传输(HT)中断,
     * DMA 传到缓冲区一半时触发, 回调以 status=DMA_STATUS_BLOCK 被调用。
     * 只有传输完成(TC, status=DMA_STATUS_COMPLETE)才该停 DMA;
     * HT 必须忽略, 否则数据传一半就被停, WS2812 收到残帧 → 不亮。
     */
    if (status == DMA_STATUS_COMPLETE) {
        /* 传完整帧后停 DMA 实现单帧停发; CCR3 保留帧尾 0,
         * PWM 输出保持低电平形成 WS2812 复位。 */
        if (g_active_cfg) {
            dma_stop(g_active_cfg->dma_dev, g_active_cfg->dma_channel);
        }
        k_sem_give(&data->dma_done);
    } else if (status < 0) {
        /* 传输错误: 停 DMA + 给信号量, 避免主线程死等。 */
        if (g_active_cfg) {
            dma_stop(g_active_cfg->dma_dev, g_active_cfg->dma_channel);
        }
        k_sem_give(&data->dma_done);
    }
    /* DMA_STATUS_BLOCK (半传输): 忽略, 继续传下半段。 */
}

//----------------------------------------------------------------------
// Forward declaration
static size_t ws2812_length(const struct device *dev);
static int ws2812_init(const struct device *dev);
static int ws2812_update_rgb(const struct device *dev,
                             struct led_rgb *pixels,
                             size_t num_pixels);

// Private functions

// ws2812's Length
static size_t ws2812_length(const struct device *dev)
{
    const struct ws2812_pwm_dma_cfg *cfg = dev->config;
    return cfg->length;
}

// Initialize the ws2812
static int ws2812_init(const struct device *dev)
{
    int ret;

    // Cast device.config (declared const void*) to our ws2812's config
    const struct ws2812_pwm_dma_cfg *cfg = (const struct ws2812_pwm_dma_cfg *)dev->config;
    struct ws2812_pwm_dma_data *data = dev->data;

    /* Check that the pwm device is ready*/
    if (!device_is_ready(cfg->pwm.dev)) {
        LOG_ERR("PWM is not ready\r\n");
        return -ENODEV;
    }

    /* Check that the dma device is ready*/
    if (!device_is_ready(cfg->dma_dev)) {
        LOG_ERR("DMA is not ready\r\n");
        return -ENODEV;
    }

    /* 先做引脚/触发准备，再使能 PWM 输出，顺序更合理 */

    /* PA2 输出速度设为 100MHz 高速(OSPEEDR[5:4] = 0b11)。
     * Zephyr 的 STM32 pinctrl 不支持 slew-rate, 默认 OSPEEDR=00(2MHz 低速),
     * 800kHz 的 WS2812 波形边沿会被拉长导致采样失败。
     * GPIOA_OSPEEDR 偏移 0x08。
     */
    *(volatile uint32_t *)(cfg->gpioa_base + 0x08) |= (0x3U << 4);

    /* 使能定时器 CC3 比较事件 DMA 请求(CC3DE)。
     * Zephyr 的 STM32 PWM 驱动不会设置 DIER.CC3DE,
     * 不打开则 DMA 永远不会被触发, 输出恒为低电平。
     */
    *(volatile uint32_t *)(cfg->pwm_base + TIM_DIER_OFFSET) |= TIM_DIER_CC3DE;

    /* Initialize PWM: ARR=104, CCR3=0, PWM1 模式, CC3E 输出使能*/
    ret = pwm_set_cycles(cfg->pwm.dev,
                         cfg->pwm.channel,
                         cfg->pwm_period,
                         0,
                         cfg->pwm.flags);
    if (ret < 0) {
        LOG_ERR("Could not configure PWM");
        return -ENODEV;
    }

    /* Initialize DMA*/
    k_mutex_init(&data->lock);
    k_sem_init(&data->dma_done, 0, 1);

    /* DMA 静态配置：这里只配置一次。
     * 每次传输的 block_size/源地址等会由 update 里的 dma_reload() 重新加载，
     * 所以这里的块大小只放一个占位值即可，避免在 update 里反复 dma_config()。
     */
    data->dma_blk.source_address = (uintptr_t)cfg->dma_buf;
    data->dma_blk.dest_address = cfg->pwm_base + TIM_CCR_OFFSET(cfg->pwm.channel);
    data->dma_blk.block_size = sizeof(uint16_t); /* 占位，update 会用 dma_reload 覆盖 */
    data->dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    data->dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    /* 循环模式 + 显式写 0 兜底，见 update 中说明 */
    data->dma_blk.source_reload_en = 1;
    data->dma_blk.dest_reload_en = 1;

    data->dma_cfg.dma_slot = cfg->dma_slot;
    data->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
    data->dma_cfg.source_data_size = 2;
    data->dma_cfg.dest_data_size = 2;
    data->dma_cfg.source_burst_length = 2;
    data->dma_cfg.dest_burst_length = 2;
    data->dma_cfg.complete_callback_en = 1;
    data->dma_cfg.block_count = 1;
    data->dma_cfg.head_block = &data->dma_blk;
    data->dma_cfg.dma_callback = ws2812_dma_callback;
    data->dma_cfg.user_data = data;

    ret = dma_config(cfg->dma_dev, cfg->dma_channel, &data->dma_cfg);
    if (ret < 0) {
        LOG_ERR("dma_config failed: %d", ret);
        return ret;
    }

    return 0;
}

static int ws2812_update_rgb(const struct device *dev,
                             struct led_rgb *pixels,
                             size_t num_pixels)
{
    const struct ws2812_pwm_dma_cfg *cfg = dev->config;
    struct ws2812_pwm_dma_data *data = dev->data;
    uint16_t *buf = cfg->dma_buf;
    size_t idx = 0;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);

    /* 扫描每个RGB转为PWM比较值序列*/
    for (size_t i = 0; i < num_pixels; i++) {
        for (int j = 0; j < cfg->num_colors; j++) {
            uint8_t pixel;

            switch (cfg->color_mapping[j]) {
                case LED_COLOR_ID_RED:
                    pixel = pixels[i].r;
                    break;
                case LED_COLOR_ID_GREEN:
                    pixel = pixels[i].g;
                    break;
                case LED_COLOR_ID_BLUE:
                    pixel = pixels[i].b;
                    break;
                default:
                    k_mutex_unlock(&data->lock);
                    return -EINVAL;
            }

            /* 每个 bit 对应一个pwm ccr值, MSB 先发*/
            for (int bit = 7; bit >= 0; bit--) {
                buf[idx++] = (pixel & BIT(bit)) ? cfg->one_pwm : cfg->zero_pwm;
            }
        }
    }

    /* 帧尾追加 WS2812_RESET_SLOTS 个 0x0000: DMA 把 CCR3 写为 0,
     * PWM 输出变为持续低电平(无脉冲), 这是 WS2812 有效复位的起点。
     * 注意只能追加"0 值"(CCR3=0 恒低), 不能追加 0 码(0x23, 带脉冲,
     * 会被 WS2812 当成数据位)。
     * 循环模式下多个 0 兜底: TC 中断到回调 dma_stop() 生效之间 DMA
     * 可能循环重开又搬值, 多个 0 保证重开写的还是 0, CCR3 保持 0。
     * 复位持续时长由下方 k_usleep(reset_delay) 保证。
     */
    for (int i = 0; i < WS2812_RESET_SLOTS; i++) {
        buf[idx++] = 0;
    }

    /* DMA 静态参数已在 init() 中通过 dma_config() 配置好。
     * 这里只需要更新本次传输长度，再用 dma_reload() 重新加载
     * 源/目的地址和 block_size。dma_reload() 会直接启动本次 DMA。
     */
    data->dma_blk.block_size = idx * sizeof(uint16_t);

    LOG_ERR("DMA dest = 0x%x (should be 0x40000C3C = TIM5_CCR3)\r\n",
            (unsigned int)(cfg->pwm_base + TIM_CCR_OFFSET(cfg->pwm.channel)));
    /* 诊断: pwm_base 字段决定 CEN 写哪、也决定 DMA 目标折算。
     * 若 pwm_base=0, 上面 CEN |= BIT(0) 会写到 0x00000000 地址(内存起始),
     * 定时器根本没使能 → CCR3 恒 0 → 灯不亮, 与实测 0V 完全吻合。
     * 必须确认 cfg->pwm_base 已正确初始化为 0x40000C00。 */
    LOG_ERR("cfg->pwm_base = 0x%x, dma_channel=%u, dma_slot=%u\r\n",
            (unsigned int)cfg->pwm_base, cfg->dma_channel, cfg->dma_slot);

    /* 记录当前 cfg, 供 DMA 完成回调里 dma_stop() 使用 */
    g_active_cfg = cfg;

    /* 关键: 显式使能 TIM5 计数器(CR1.CEN=1)。
     * Zephyr 的 pwm_set_cycles 只设 CCR/ARR, 不启动计数; 单次(DMA NORMAL)
     * 模式 DMA 严格等 CC3 比较事件触发, 若 CEN=0 则 CC3 永不发生比较,
     * DMA 永不被触发, CCR3 停在初始化 0, 没有脉冲 → 灯不亮。
     * 之前循环模式能偶亮是因为循环 DMA 硬件持续搬运脉冲掩盖了这点。
     * TIM5 基址 0x40000C00, CR1 偏移 0x00, CEN = bit0。
     */
    *(volatile uint32_t *)(cfg->pwm_base + 0x00) |= BIT(0);

    ret = dma_reload(cfg->dma_dev, cfg->dma_channel,
                     (uintptr_t)cfg->dma_buf,
                     cfg->pwm_base + TIM_CCR_OFFSET(cfg->pwm.channel),
                     idx * sizeof(uint16_t));
    if (ret < 0) {
        LOG_ERR("dma_reload failed: %d", ret);
        k_mutex_unlock(&data->lock);
        return ret;
    }

    /* 等 DMA 传输完成(回调里已 dma_stop 停掉循环), 加超时防止
     * DMA 异常时永久阻塞。
     */
    if (k_sem_take(&data->dma_done, K_MSEC(100)) < 0) {
        LOG_ERR("DMA transfer timeout");
        k_mutex_unlock(&data->lock);
        return -ETIMEDOUT;
    }
      /* 关键判据: DMA 传完(信号量到手、回调已 dma_stop)这一瞬读 CCR3。
       *  - 测到 68/30(0x44/0x1E) → DMA 真搬了数据, 问题在复位/电气;
       *  - 测到 0x0000            → DMA 没搬进去, 根因在 DMA 触发链。
       */
      volatile uint32_t cc3 =
          *(volatile uint32_t *)(cfg->pwm_base + TIM_CCR_OFFSET(3));
      LOG_ERR("CCR3 after DMA = 0x%x (期望数据值; 0=没搬)\r\n",
              (unsigned int)cc3);

    /* 关键兜底: 循环模式下 DMA 发完整帧触发 TC 后会立刻循环重开、
     * 搬回缓冲区开头(帧头数据), 回调里 dma_stop() 执行时 CCR3 多半
     * 已被循环重开的数据覆盖(实测 CCR3=0x46=70, 1码高电平) —— 复位
     * 低电平没有建立, WS2812 收不到复位 → 灯不亮。
     * 复位不能依赖 "DMA 停在帧尾 0", 必须显式把 CCR3 写为 0 强制输出
     * 拉低, 由下方 k_usleep 维持 >=50us 复位低电平, 与原始单次模式一致。
     */
    ret = pwm_set_cycles(cfg->pwm.dev, cfg->pwm.channel,
                         cfg->pwm_period, 0, cfg->pwm.flags);
    if (ret < 0) {
        LOG_ERR("Could not set PWM low for reset");
        k_mutex_unlock(&data->lock);
        return ret;
    }

    /* WS2812 帧间复位低电平时间*/
    k_usleep(cfg->reset_delay_us);

    k_mutex_unlock(&data->lock);

    return 0;
}

#define WS2812_NUM_COLORS(inst) \
    DT_INST_PROP_LEN(inst, color_mapping)

#define WS2812_COLOR_MAPPING(inst) \
    static const uint8_t ws2812_color_map_##inst[] = \
        DT_INST_PROP(inst, color_mapping)

/* 每颗灯 3 色 × 8 bit, 末尾追加 WS2812_RESET_SLOTS 个 0 作为复位值,
 * DMA 把这些 0 写进 CCR3 后输出拉低, 产生 WS2812 复位低电平。
 * 循环模式下多个 0 可兜底回调延迟(见 WS2812_RESET_SLOTS 注释)。
 */
#define WS2812_BUF_LEN(inst) \
    (DT_INST_PROP(inst, chain_length) * \
     WS2812_NUM_COLORS(inst) * 8 + WS2812_RESET_SLOTS)

// Devicetree handling
// Define DEVICE API
static DEVICE_API(led_strip, ws2812_pwm_dma_api) = {
    .update_rgb = ws2812_update_rgb,
    .length = ws2812_length,
};

// Expansion macro to define driver instances
#define WS2812_DEFINE(inst)                                                 \
    static struct ws2812_pwm_dma_data ws2812_data_##inst;                   \
    WS2812_COLOR_MAPPING(inst);                                             \
    static uint16_t ws2812_buf_##inst[WS2812_BUF_LEN(inst)];                \
    static const struct ws2812_pwm_dma_cfg ws2812_cfg_##inst = {            \
        .pwm = PWM_DT_SPEC_INST_GET(inst),                                  \
        .dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)),                  \
        .gpioa_base = DT_REG_ADDR(DT_NODELABEL(gpioa)),                     \
        .dma_channel = DT_INST_DMAS_CELL_BY_IDX(inst, 0, channel),          \
        .dma_slot = DT_INST_DMAS_CELL_BY_IDX(inst, 0, slot),                \
        .pwm_base = DT_REG_ADDR(DT_PARENT(DT_PWMS_CTLR(DT_DRV_INST(inst)))), \
        .length = DT_INST_PROP(inst, chain_length),                         \
        .num_colors = WS2812_NUM_COLORS(inst),                              \
        .color_mapping = ws2812_color_map_##inst,                           \
        .pwm_period = WS2812_PWM_PERIOD,                                    \
        .one_pwm = WS2812_ONE_PWM,                                          \
        .zero_pwm = WS2812_ZERO_PWM,                                        \
        .reset_delay_us = WS2812_RESET_DELAY,                               \
        .dma_buf = ws2812_buf_##inst,                                       \
        .dma_buf_len = WS2812_BUF_LEN(inst),                                \
    };                                                                      \
    DEVICE_DT_INST_DEFINE(inst,                                             \
                          ws2812_init,                                      \
                          NULL,                                             \
                          &ws2812_data_##inst,                              \
                          &ws2812_cfg_##inst,                               \
                          POST_KERNEL,                                      \
                          80,                                               \
                          &ws2812_pwm_dma_api);                             \

DT_INST_FOREACH_STATUS_OKAY(WS2812_DEFINE)
