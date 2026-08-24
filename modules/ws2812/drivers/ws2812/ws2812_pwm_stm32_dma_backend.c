/*
 * Copyright (c) 2024 CurrentMakers
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * STM32 DMA backend for the ws2812_pwm driver.
 *
 * Strategy:
 *   - The PWM is set up (by drivers/pwm/pwm_stm32.c) on channel N of
 *     some TIMx, running at the WS2812 bit rate (~800 kHz).
 *   - We enable the timer's "update" DMA request (TIMx_DIER.UDE).
 *   - A single memory-to-peripheral DMA transfer streams our buffer
 *     of 16-bit compare values into TIMx->CCRn, one slot per update
 *     event. The CPU is idle for the whole frame.
 *   - We block on a semaphore signalled from the DMA completion
 *     callback, then stop the DMA request.
 */

#define DT_DRV_COMPAT worldsemi_ws2812_pwm

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>

#include <stm32_ll_tim.h>

LOG_MODULE_DECLARE(ws2812_pwm, CONFIG_LED_STRIP_LOG_LEVEL);

/* Must match the typedef in ws2812_pwm.c */
typedef uint32_t ws2812_pwm_slot_t;

int ws2812_pwm_transmit(const struct device *dev,
			const ws2812_pwm_slot_t *buf, size_t len);

/* ------------------------------------------------------------------ */
/* Per-instance backend state                                          */
/* ------------------------------------------------------------------ */

struct ws2812_stm32_backend {
	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_slot;       /* request line / mux slot */
	uint32_t dma_priority;

	TIM_TypeDef *timer;      /* parent of the PWM channel */
	uint32_t ccr_addr;       /* &TIMx->CCRn as a uint32_t for DMA */

	struct k_sem done;
	int xfer_status;
};

/* ------------------------------------------------------------------ */
/* DT helpers                                                          */
/* ------------------------------------------------------------------ */

/*
 * The `pwms` phandle points at a node whose parent is the timer, e.g.
 *    &timers3 { pwm3: pwm { ... }; };
 * so DT_PARENT(DT_PHANDLE(inst, pwms)) gives us the timer node, from
 * which we can read `reg` to get TIMx's base address.
 */
#define WS2812_TIMER_NODE(idx) \
	DT_PARENT(DT_PHANDLE_BY_IDX(DT_DRV_INST(idx), pwms, 0))

#define WS2812_TIMER_ADDR(idx) \
	((TIM_TypeDef *)DT_REG_ADDR(WS2812_TIMER_NODE(idx)))

/*
 * The PWM channel number is the 2nd cell of the `pwms` specifier.
 * For st,stm32-pwm the cells are <timer-channel period flags>.
 */
#define WS2812_PWM_CHANNEL(idx) \
	DT_PWMS_CHANNEL_BY_IDX(DT_DRV_INST(idx), 0)

/* &TIMx->CCRn for channel n in 1..4 */
static uint32_t tim_ccr_addr(TIM_TypeDef *tim, uint32_t channel)
{
	switch (channel) {
	case 1: return (uint32_t)&tim->CCR1;
	case 2: return (uint32_t)&tim->CCR2;
	case 3: return (uint32_t)&tim->CCR3;
	case 4: return (uint32_t)&tim->CCR4;
	default: return 0;
	}
}

/* ------------------------------------------------------------------ */
/* DMA callback                                                        */
/* ------------------------------------------------------------------ */

static void ws2812_dma_cb(const struct device *dma, void *user,
			  uint32_t channel, int status)
{
	ARG_UNUSED(dma);
	ARG_UNUSED(channel);

	struct ws2812_stm32_backend *b = user;

	b->xfer_status = status;
	k_sem_give(&b->done);
}

/* ------------------------------------------------------------------ */
/* The backend entry point — overrides the weak symbol in ws2812_pwm.c */
/* ------------------------------------------------------------------ */

/* Forward declarations of the per-instance backends we define at the
 * bottom of the file via DT_INST_FOREACH_STATUS_OKAY.
 */
struct ws2812_stm32_backend *ws2812_stm32_backend_for(const struct device *dev);

int ws2812_pwm_transmit(const struct device *dev,
			const ws2812_pwm_slot_t *buf, size_t len)
{
	struct ws2812_stm32_backend *b = ws2812_stm32_backend_for(dev);

	if (b == NULL) {
		LOG_ERR("No STM32 backend bound for %s", dev->name);
		return -ENODEV;
	}
	if (!device_is_ready(b->dma_dev)) {
		LOG_ERR("DMA %s not ready", b->dma_dev->name);
		return -ENODEV;
	}

	struct dma_block_config block = {
		.source_address   = (uint32_t)buf,
		.dest_address     = b->ccr_addr,
		.source_addr_adj  = DMA_ADDR_ADJ_INCREMENT,
		.dest_addr_adj    = DMA_ADDR_ADJ_NO_CHANGE,
		.block_size       = len * sizeof(ws2812_pwm_slot_t),
	};

	struct dma_config cfg = {
		.dma_slot         = b->dma_slot,
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_data_size = sizeof(ws2812_pwm_slot_t),
		.dest_data_size   = sizeof(ws2812_pwm_slot_t),
		.source_burst_length = sizeof(ws2812_pwm_slot_t),
		.dest_burst_length   = sizeof(ws2812_pwm_slot_t),
		.channel_priority = b->dma_priority,
		.block_count      = 1,
		.head_block       = &block,
		.user_data        = b,
		.dma_callback     = ws2812_dma_cb,
		.complete_callback_en = 0, /* fire only on final completion */
		.error_callback_dis   = 0,
	};

	k_sem_reset(&b->done);
	b->xfer_status = 0;

	int ret = dma_config(b->dma_dev, b->dma_channel, &cfg);
	if (ret < 0) {
		LOG_ERR("dma_config: %d", ret);
		return ret;
	}

	ret = dma_start(b->dma_dev, b->dma_channel);
	if (ret < 0) {
		LOG_ERR("dma_start: %d", ret);
		return ret;
	}

	/* Arm the timer's update-event DMA request. */
	LL_TIM_EnableDMAReq_UPDATE(b->timer);

	/* Worst-case wait: one period per slot + some slack. At 800 kHz
	 * that's 1.25 us * len; for a 300-LED strip (~7200 slots) that
	 * is ~9 ms. Give it 10x headroom.
	 */
	k_timeout_t timeout = K_MSEC(10 + (len / 100));
	if (k_sem_take(&b->done, timeout) != 0) {
		LOG_ERR("DMA timed out");
		(void)dma_stop(b->dma_dev, b->dma_channel);
		LL_TIM_DisableDMAReq_UPDATE(b->timer);
		return -ETIMEDOUT;
	}

	LL_TIM_DisableDMAReq_UPDATE(b->timer);
	(void)dma_stop(b->dma_dev, b->dma_channel);

	return b->xfer_status;
}

/* ------------------------------------------------------------------ */
/* Per-instance wiring                                                 */
/* ------------------------------------------------------------------ */

#define WS2812_HAS_DMA(idx) DT_INST_DMAS_HAS_NAME(idx, update)

#define WS2812_STM32_DEFINE(idx)					      \
	COND_CODE_1(WS2812_HAS_DMA(idx), (					\
		static struct ws2812_stm32_backend				\
			ws2812_stm32_backend_##idx = {				\
			.dma_dev      = DEVICE_DT_GET(				\
				DT_INST_DMAS_CTLR_BY_NAME(idx, update)),	\
			.dma_channel  = DT_INST_DMAS_CELL_BY_NAME(		\
				idx, update, channel),				\
			.dma_slot     = DT_INST_DMAS_CELL_BY_NAME(		\
				idx, update, slot),				\
			.dma_priority = 0,					\
			.timer        = WS2812_TIMER_ADDR(idx),			\
		};								\
	), (									\
		static struct ws2812_stm32_backend				\
			ws2812_stm32_backend_##idx;				\
	))

DT_INST_FOREACH_STATUS_OKAY(WS2812_STM32_DEFINE)

/* Build a constant lookup table from `struct device *` to backend
 * state. Keeps the backend completely decoupled from the generic
 * driver's private data layout.
 */
struct ws2812_stm32_entry {
	const struct device *dev;
	struct ws2812_stm32_backend *backend;
};

#define WS2812_STM32_ENTRY(idx) {					\
	.dev = DEVICE_DT_INST_GET(idx),					\
	.backend = COND_CODE_1(WS2812_HAS_DMA(idx),			\
		(&ws2812_stm32_backend_##idx), (NULL)),			\
},

static const struct ws2812_stm32_entry ws2812_stm32_table[] = {
	DT_INST_FOREACH_STATUS_OKAY(WS2812_STM32_ENTRY)
};

struct ws2812_stm32_backend *ws2812_stm32_backend_for(const struct device *dev)
{
	for (size_t i = 0; i < ARRAY_SIZE(ws2812_stm32_table); i++) {
		if (ws2812_stm32_table[i].dev == dev) {
			struct ws2812_stm32_backend *b =
				ws2812_stm32_table[i].backend;

			if (b != NULL && b->ccr_addr == 0) {
				/* Lazy init: compute CCR address and init sem. */
				k_sem_init(&b->done, 0, 1);
				uint32_t ch = 0;

				/* Find the channel for this instance. */
				#define WS2812_MATCH(idx)			  \
					if (dev == DEVICE_DT_INST_GET(idx)) { \
						ch = WS2812_PWM_CHANNEL(idx); \
					}
				DT_INST_FOREACH_STATUS_OKAY(WS2812_MATCH)
				#undef WS2812_MATCH

				b->ccr_addr = tim_ccr_addr(b->timer, ch);

				/*
				 * UDE 在每个更新事件触发 DMA 写 CCR。
				 * 必须关闭 CCR 影子寄存器(preload)，否则新比较值要
				 * 等到下一个更新事件才生效，会整体错一位且最后一 bit 丢失。
				 */
				switch (ch) {
				case 1:
					LL_TIM_OC_DisablePreload(b->timer, LL_TIM_CHANNEL_CH1);
					break;
				case 2:
					LL_TIM_OC_DisablePreload(b->timer, LL_TIM_CHANNEL_CH2);
					break;
				case 3:
					LL_TIM_OC_DisablePreload(b->timer, LL_TIM_CHANNEL_CH3);
					break;
				case 4:
					LL_TIM_OC_DisablePreload(b->timer, LL_TIM_CHANNEL_CH4);
					break;
				default:
					break;
				}
			}
			return b;
		}
	}
	return NULL;
}
