/*
 * Copyright (c) 2024 CurrentMakers
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_pwm

#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(ws2812_pwm, CONFIG_LED_STRIP_LOG_LEVEL);

/*
 * One DMA slot per bit.
 *
 * NOTE: on this STM32F401 target, a 16-bit write to the 32-bit TIMx_CCRn
 * register is replicated to both the low and high 16-bit halves
 * (observed: writing 0x1234 produces 0x12341234).  That would make the
 * compare value huge and break WS2812 output.  We therefore use 32-bit
 * DMA slots so each DMA write places a normal 32-bit compare value into
 * CCRn (e.g. 0x00000023).
 */
typedef uint32_t ws2812_pwm_slot_t;

#define WS2812_BITS_PER_LED   8
#define WS2812_RESET_SLOTS(ratio_unused) 0 /* reset is done by PWM stop + sleep */

struct ws2812_pwm_config {
	struct pwm_dt_spec pwm;
	uint32_t chain_length;
	const uint8_t *color_mapping;
	size_t num_colors;       /* bytes per pixel: typically 3 (RGB) or 4 (RGBW) */
	uint32_t reset_delay_us;
	uint32_t zero_pulse_ns;
	uint32_t one_pulse_ns;

	/* Pre-computed at init */
	ws2812_pwm_slot_t period_cycles;
	ws2812_pwm_slot_t zero_cycles;
	ws2812_pwm_slot_t one_cycles;

	/* DMA buffer: one slot per bit of the full strip */
	ws2812_pwm_slot_t *buf;
	size_t buf_len;          /* in slots */
};

struct ws2812_pwm_data {
	struct k_mutex lock;
	void *backend;   /* opaque handle for the DMA backend, if any */
};

/* ------------------------------------------------------------------ */
/* DMA backend hook                                                    */
/* ------------------------------------------------------------------ */
int ws2812_pwm_transmit(const struct device *dev,
			const ws2812_pwm_slot_t *buf, size_t len);

__weak int ws2812_pwm_transmit(const struct device *dev,
			 const ws2812_pwm_slot_t *buf, size_t len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);

	if (IS_ENABLED(CONFIG_WS2812_PWM_CPU_FALLBACK)) {
		return -ENOTSUP;
	}

	return -ENOSYS;
}


/* ------------------------------------------------------------------ */
/* Bitstream builder                                                   */
/* ------------------------------------------------------------------ */

static inline uint8_t pick_channel(const struct led_rgb *px, uint8_t color_id)
{
	switch (color_id) {
	case LED_COLOR_ID_RED:   return px->r;
	case LED_COLOR_ID_GREEN: return px->g;
	case LED_COLOR_ID_BLUE:  return px->b;
	default:
		/* White / unknown: WS2812B has no white channel. */
		return 0;
	}
}

static void ws2812_encode_byte(ws2812_pwm_slot_t *dst, uint8_t byte,
			       ws2812_pwm_slot_t zero, ws2812_pwm_slot_t one)
{
	/* WS2812 is MSB-first. */
	for (int b = 7; b >= 0; b--) {
		*dst++ = (byte & BIT(b)) ? one : zero;
	}
}

/* ------------------------------------------------------------------ */
/* led_strip API                                                       */
/* ------------------------------------------------------------------ */

static int ws2812_pwm_update_rgb(const struct device *dev,
				 struct led_rgb *pixels, size_t num_pixels)
{
	const struct ws2812_pwm_config *cfg = dev->config;
	struct ws2812_pwm_data *data = dev->data;

	if (num_pixels > cfg->chain_length) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ws2812_pwm_slot_t *p = cfg->buf;
	for (size_t i = 0; i < num_pixels; i++) {
		for (size_t c = 0; c < cfg->num_colors; c++) {
			uint8_t byte = pick_channel(&pixels[i],
						    cfg->color_mapping[c]);
			ws2812_encode_byte(p, byte,
					   cfg->zero_cycles, cfg->one_cycles);
			p += WS2812_BITS_PER_LED;
		}
	}

	/* Any unused tail (num_pixels < chain_length) -> blank LEDs. */
	size_t used = num_pixels * cfg->num_colors * WS2812_BITS_PER_LED;
	for (size_t i = used; i < cfg->buf_len; i++) {
		cfg->buf[i] = 0;
	}

	int ret = ws2812_pwm_transmit(dev, cfg->buf, cfg->buf_len);

	/* Reset latch: line low for >= 50 us. Easiest portable way is
	 * to drop the duty cycle to 0 and sleep.
	 */
	(void)pwm_set_cycles(cfg->pwm.dev, cfg->pwm.channel,
			     cfg->period_cycles, 0, cfg->pwm.flags);
	k_busy_wait(cfg->reset_delay_us);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ws2812_pwm_update_channels(const struct device *dev,
				      uint8_t *channels, size_t num_channels)
{
	const struct ws2812_pwm_config *cfg = dev->config;
	struct ws2812_pwm_data *data = dev->data;

	if (num_channels > cfg->buf_len / WS2812_BITS_PER_LED) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	for (size_t i = 0; i < num_channels; i++) {
		ws2812_encode_byte(&cfg->buf[i * WS2812_BITS_PER_LED],
				   channels[i],
				   cfg->zero_cycles, cfg->one_cycles);
	}
	size_t used = num_channels * WS2812_BITS_PER_LED;
	for (size_t i = used; i < cfg->buf_len; i++) {
		cfg->buf[i] = 0;
	}

	int ret = ws2812_pwm_transmit(dev, cfg->buf, cfg->buf_len);
	(void)pwm_set_cycles(cfg->pwm.dev, cfg->pwm.channel,
			     cfg->period_cycles, 0, cfg->pwm.flags);
	k_busy_wait(cfg->reset_delay_us);

	k_mutex_unlock(&data->lock);
	return ret;
}

static size_t ws2812_pwm_length(const struct device *dev)
{
	const struct ws2812_pwm_config *cfg = dev->config;

	return cfg->chain_length;
}

static DEVICE_API(led_strip, ws2812_pwm_api) = {
	.update_rgb      = ws2812_pwm_update_rgb,
	.update_channels = ws2812_pwm_update_channels,
	.length          = ws2812_pwm_length,
};

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

static int ws2812_pwm_init(const struct device *dev)
{
	struct ws2812_pwm_config *cfg = (struct ws2812_pwm_config *)dev->config;
	struct ws2812_pwm_data *data = dev->data;

	if (!pwm_is_ready_dt(&cfg->pwm)) {
		LOG_ERR("PWM device %s not ready", cfg->pwm.dev->name);
		return -ENODEV;
	}

	/* Resolve nanoseconds -> cycles using the PWM controller itself. */
	uint64_t cycles_per_sec;
	int ret = pwm_get_cycles_per_sec(cfg->pwm.dev, cfg->pwm.channel,
					 &cycles_per_sec);
	if (ret < 0) {
		LOG_ERR("pwm_get_cycles_per_sec: %d", ret);
		return ret;
	}

	uint64_t period_cycles =
		(uint64_t)cfg->pwm.period * cycles_per_sec / NSEC_PER_SEC;
	uint64_t zero_cycles =
		(uint64_t)cfg->zero_pulse_ns * cycles_per_sec / NSEC_PER_SEC;
	uint64_t one_cycles =
		(uint64_t)cfg->one_pulse_ns * cycles_per_sec / NSEC_PER_SEC;

	if (period_cycles == 0 || period_cycles > UINT16_MAX ||
	    zero_cycles >= period_cycles || one_cycles >= period_cycles ||
	    zero_cycles == 0 || one_cycles == 0) {
		LOG_ERR("Bad WS2812 timing: period=%llu zero=%llu one=%llu",
			period_cycles, zero_cycles, one_cycles);
		return -EINVAL;
	}

	cfg->period_cycles = (ws2812_pwm_slot_t)period_cycles;
	cfg->zero_cycles   = (ws2812_pwm_slot_t)zero_cycles;
	cfg->one_cycles    = (ws2812_pwm_slot_t)one_cycles;

	LOG_DBG("WS2812 PWM: %u cyc/bit, T0H=%u T1H=%u, %u LEDs, %zu-byte buf",
		cfg->period_cycles, cfg->zero_cycles, cfg->one_cycles,
		cfg->chain_length, cfg->buf_len * sizeof(ws2812_pwm_slot_t));

	k_mutex_init(&data->lock);

	/* Program the period once; DMA will only rewrite the compare value. */
	ret = pwm_set_cycles(cfg->pwm.dev, cfg->pwm.channel,
			     cfg->period_cycles, 0, cfg->pwm.flags);
	if (ret < 0) {
		LOG_ERR("Initial pwm_set_cycles failed: %d", ret);
		return ret;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Instance boilerplate                                                */
/* ------------------------------------------------------------------ */

#define WS2812_NUM_COLORS(idx) DT_INST_PROP_LEN(idx, color_mapping)

#define WS2812_PWM_BUF_LEN(idx)						\
	(DT_INST_PROP(idx, chain_length) *				\
	 WS2812_NUM_COLORS(idx) * WS2812_BITS_PER_LED)

#define WS2812_PWM_INST(idx)						      \
	BUILD_ASSERT(WS2812_NUM_COLORS(idx) >= 3 &&			      \
		     WS2812_NUM_COLORS(idx) <= 4,			      \
		     "WS2812 color-mapping must have 3 or 4 entries");	      \
									      \
	static const uint8_t ws2812_pwm_color_mapping_##idx[] =		      \
		DT_INST_PROP(idx, color_mapping);			      \
									      \
	static ws2812_pwm_slot_t					      \
		ws2812_pwm_buf_##idx[WS2812_PWM_BUF_LEN(idx)];		      \
									      \
	static struct ws2812_pwm_config ws2812_pwm_config_##idx = {	      \
		.pwm = PWM_DT_SPEC_INST_GET(idx),			      \
		.chain_length = DT_INST_PROP(idx, chain_length),	      \
		.color_mapping = ws2812_pwm_color_mapping_##idx,	      \
		.num_colors = WS2812_NUM_COLORS(idx),			      \
		.reset_delay_us = DT_INST_PROP(idx, reset_delay),	      \
		.zero_pulse_ns = DT_INST_PROP(idx, zero_pulse_ns),	      \
		.one_pulse_ns  = DT_INST_PROP(idx, one_pulse_ns),	      \
		.buf = ws2812_pwm_buf_##idx,				      \
		.buf_len = ARRAY_SIZE(ws2812_pwm_buf_##idx),		      \
	};								      \
									      \
	static struct ws2812_pwm_data ws2812_pwm_data_##idx;		      \
									      \
	DEVICE_DT_INST_DEFINE(idx,					      \
			    ws2812_pwm_init,				      \
			    NULL,					      \
			    &ws2812_pwm_data_##idx,			      \
			    &ws2812_pwm_config_##idx,			      \
			    POST_KERNEL,				      \
			    90,						      \
			    &ws2812_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_INST)
