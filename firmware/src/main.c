/* SP-1FX: a four-effect processor for the Teenage Engineering SP-1.
 *
 * Audio in:  USB-C, class compliant (UAC2), 48 kHz / 16- or 24-bit stereo (the host picks), from a computer or phone.
 * Audio out: headphones (CS42L42) or, with nothing plugged in, the speaker (TAS2505), 24-bit.
 * In between, fx/fx.c at 48 kHz: one effect per fader (DUB, FILTER, CRUSH, LA-2A), then a
 * safety ceiling.
 *
 * Controls
 *   faders 1-4        the four effects (see fx/fx.h)
 *   FWD (>>) / RWD (<<)  page 2 / page 1. Page 2 puts each effect's second parameter on its
 *                     fader. After a page change a parameter keeps its value until its fader
 *                     reaches it (pick-up), so nothing jumps.
 *   TRACK 1-4         that effect off/on (it glides to rest and back to its fader)
 *   PLAY              bypass on/off (the effects glide to rest; the safety ceiling stays)
 *   VOL + / VOL -     output volume, 3 dB steps
 *   FUNCTION, hold    power on (1.5 s) / power off (2.5 s)
 *   TRACK 1 + 4, hold 3 s: reset into the bootloader for flashing
 *
 * Lights
 *   track lights      page 1: how far each effect is in (filter: distance from the centre); a
 *                     faint glow when that effect is switched off. Page 2: each second
 *                     parameter; blinking until its fader picks it up
 *   status lights     LA-2A gain reduction (0.5, 2, 4, 8 dB), the fourth blinking on page 2;
 *                     volume for a second after a change; the page for 0.6 s after a change
 *                     (1+2 = page 1, 3+4 = page 2); all four slowly breathing = bypass
 *
 * The platform code (power, codecs, I2S clocking, the USB audio class and its feedback, the
 * bootloader rules) follows chattock/sp1-tape-looper (MIT, see LICENSE-sp1-tape-looper), which
 * runs on SP-1 hardware, and Tim Knapen's SP-1-dev wiki. */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/fatal.h>
#include <sample_usbd.h>
#include <soc.h>
#include <string.h>

extern bool uac2_fs_fb_windows_fmt;

#include "fx.h"

/* ------------------------------------------------------------------ faults ---- */
static __noinit uint32_t g_fault_key;   /* 0xFA17FA17: the last boot crashed */
static __noinit uint32_t g_fault_reason;
static __noinit uint32_t g_fault_pc;
static uint32_t g_resetreas;
static uint32_t g_last_fault = 0xFFFFFFFFu, g_last_fault_pc;

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	g_fault_reason = reason;
	g_fault_pc = esf ? esf->basic.pc : 0u;
	g_fault_key = 0xFA17FA17u;
	sys_reboot(SYS_REBOOT_COLD);
	CODE_UNREACHABLE;
}

/* ------------------------------------------------------------------- pins ---- */
struct led { NRF_GPIO_Type *port; uint32_t pin; };
static const struct led sta_leds[4] = { { NRF_P1, 13 }, { NRF_P0, 0 }, { NRF_P1, 12 }, { NRF_P0, 1 } };
static const struct led trk_leds[4] = { { NRF_P0, 29 }, { NRF_P0, 26 }, { NRF_P1, 15 }, { NRF_P1, 14 } };

#define PWR_PORT      NRF_P0   /* FUNCTION button, active low */
#define PWR_PIN       27u
#define BQ_PORT       NRF_P0   /* BQ24232 charger */
#define BQ_NCE_PIN    21u
#define BQ_NCHG_PIN   22u
#define BQ_NPGOOD_PIN 24u
#define BTN_COM_PORT  NRF_P1   /* powers the button ladders and faders */
#define BTN_COM_PIN   10u
#define CS42_RST_PORT NRF_P0
#define CS42_RST_PIN  15u
#define TAS_RST_PORT  NRF_P0
#define TAS_RST_PIN   9u
#define OSC_EN_PORT   NRF_P0   /* the 3.072 MHz oscillator: the I2S bit clock */
#define OSC_EN_PIN    13u
#define EMMC_RST_PORT NRF_P1
#define EMMC_RST_PIN  8u
#define EMMC_VCCQ_PIN 14u      /* P0 */

#define CS42L42_ADDR  0x48u
#define TAS2505_ADDR  0x18u

#define HOLD_ON_MS    1500
#define HOLD_OFF_MS   2500
#define DFU_HOLD_MS   3000

static void pin_out(NRF_GPIO_Type *port, uint32_t pin, int high)
{
	if (high) port->OUTSET = 1u << pin; else port->OUTCLR = 1u << pin;
	port->PIN_CNF[pin] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
			     (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
			     (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
	if (high) port->OUTSET = 1u << pin; else port->OUTCLR = 1u << pin;
}

static void pin_in_pullup(NRF_GPIO_Type *port, uint32_t pin, int sense_low)
{
	port->PIN_CNF[pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
			     (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
			     (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
			     ((sense_low ? GPIO_PIN_CNF_SENSE_Low : GPIO_PIN_CNF_SENSE_Disabled)
			      << GPIO_PIN_CNF_SENSE_Pos);
}

static bool pwr_pressed(void) { return (PWR_PORT->IN & (1u << PWR_PIN)) == 0u; }
static bool usb_present(void) { return (BQ_PORT->IN & (1u << BQ_NPGOOD_PIN)) == 0u; }
static bool charging(void)    { return (BQ_PORT->IN & (1u << BQ_NCHG_PIN)) == 0u; }

static void charger_init(void)
{
	pin_in_pullup(BQ_PORT, BQ_NCHG_PIN, 0);
	pin_in_pullup(BQ_PORT, BQ_NPGOOD_PIN, 0);
	pin_out(BQ_PORT, BQ_NCE_PIN, 0);   /* /CE low: charging on */
}

static void feed_wdt(void)
{
	for (int ch = 0; ch < 8; ch++) NRF_WDT->RR[ch] = WDT_RR_RR_Reload;
}

/* ------------------------------------------------------------------- LEDs ----
 * Hardware PWM, 1 kHz: PWM2 the status row, PWM3 the track row (as the looper's LEDPWM-710).
 * Levels are 0..1000 (microseconds on per millisecond). */
#define LED_PERIOD 1000u
#define LED_POL    0x8000u
static uint16_t g_sta_duty[4] __aligned(4);
static uint16_t g_trk_duty[4] __aligned(4);

static void led_pwm_one(NRF_PWM_Type *pwm, const struct led *l, uint16_t *duty)
{
	pwm->ENABLE = 0;
	for (int i = 0; i < 4; i++) {
		pin_out(l[i].port, l[i].pin, 0);
		pwm->PSEL.OUT[i] = (l[i].pin & 31u) | ((l[i].port == NRF_P1) ? (1u << 5) : 0u);
	}
	pwm->MODE = PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos;
	pwm->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos; /* 1 us */
	pwm->COUNTERTOP = LED_PERIOD;
	pwm->LOOP = 1u;
	pwm->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
		       (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
	for (int s = 0; s < 2; s++) {
		pwm->SEQ[s].PTR = (uint32_t)duty;
		pwm->SEQ[s].CNT = 4;
		pwm->SEQ[s].REFRESH = 0;
		pwm->SEQ[s].ENDDELAY = 0;
	}
	pwm->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;
	pwm->ENABLE = 1;
	pwm->TASKS_SEQSTART[0] = 1;
}

static void led_set(uint16_t *duty, int i, int level)
{
	if (level < 0) level = 0;
	if (level > (int)LED_PERIOD) level = LED_PERIOD;
	duty[i] = (uint16_t)(LED_POL | (uint32_t)level);
}

static void leds_off(void)
{
	for (int i = 0; i < 4; i++) { led_set(g_sta_duty, i, 0); led_set(g_trk_duty, i, 0); }
}

/* before SYSTEM_OFF: PWM off and every LED pin driven low, so nothing stays lit */
static void leds_shutdown(void)
{
	NRF_PWM2->ENABLE = 0;
	NRF_PWM3->ENABLE = 0;
	for (int i = 0; i < 4; i++) {
		NRF_PWM2->PSEL.OUT[i] = 0xFFFFFFFFu;
		NRF_PWM3->PSEL.OUT[i] = 0xFFFFFFFFu;
		pin_out(sta_leds[i].port, sta_leds[i].pin, 0);
		pin_out(trk_leds[i].port, trk_leds[i].pin, 0);
	}
}

#define LED_DIM 220   /* full brightness for "on"; the panel LEDs are bright */

/* ------------------------------------------------------------------ ADC ---- */
static const struct adc_dt_spec adc_ch[] = {
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0), /* AIN0: PLAY + tracks */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1), /* AIN1: VOL + FWD/RWD */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2), /* fader 1 */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3), /* fader 2 */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4), /* fader 3 */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 5), /* fader 4 */
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 6), /* battery */
};
#define NCH        ARRAY_SIZE(adc_ch)
#define CH_TRACKS  0
#define CH_VOL     1
#define CH_FADER0  2
#define CH_BATT    6
#define FADER_TOP  3700.0f

static int16_t g_adc_buf[NCH];
static int g_adc[NCH];
static bool g_adc_ok;

static void controls_init(void)
{
	pin_out(BTN_COM_PORT, BTN_COM_PIN, 1);
	g_adc_ok = true;
	for (int i = 0; i < (int)NCH; i++) {
		if (!device_is_ready(adc_ch[i].dev) || adc_channel_setup_dt(&adc_ch[i]) < 0 ||
		    adc_ch[i].channel_id != (uint8_t)i || adc_ch[i].dev != adc_ch[0].dev)
			g_adc_ok = false;
	}
}

/* one scan of every channel, averaged over two conversions */
static bool adc_scan(void)
{
	if (!g_adc_ok) return false;
	int16_t first[NCH];
	struct adc_sequence seq = { .buffer = g_adc_buf, .buffer_size = sizeof(g_adc_buf) };
	if (adc_sequence_init_dt(&adc_ch[0], &seq) < 0) return false;
	seq.channels = 0u;
	for (int i = 0; i < (int)NCH; i++) seq.channels |= BIT(adc_ch[i].channel_id);
	if (adc_read_dt(&adc_ch[0], &seq) < 0) return false;
	memcpy(first, g_adc_buf, sizeof(first));
	if (adc_read_dt(&adc_ch[0], &seq) < 0) return false;
	for (int i = 0; i < (int)NCH; i++) g_adc[i] = ((int)first[i] + (int)g_adc_buf[i]) / 2;
	return true;
}

/* Button ladders. Every set of track buttons pressed together makes its own code; measured on
 * hardware by the looper (SP1-BUTTON-LADDER-MAP.md, main.c M27-r3):
 *   idle 2 | T1 213 | T2 404 | 1+2 572 | T3 728 | 1+3 862 | 2+3 989 | 1+2+3 1100 | T4 1209
 *   1+4 1303 | 2+4 1391 | 1+2+4 1470 | 3+4 1548 | 1+3+4 1618 | 2+3+4 1683 | ALL4 1743
 *   PLAY 1804 | PLAY+T1 1861 ...
 * Only the two codes this firmware uses are decoded, each in its own band. */
static bool is_play(int v)  { return v >= 1773 && v < 1835; }
/* one track button alone (T1 213, T2 404, T3 728, T4 1209), each band clear of every chord */
static int single_track(int v)
{
	if (v >= 150 && v < 300) return 0;
	if (v >= 330 && v < 470) return 1;
	if (v >= 680 && v < 790) return 2;
	if (v >= 1160 && v < 1256) return 3;
	return -1;
}
static bool combo_1_4(int v) { return v >= 1256 && v < 1347; }   /* ONLY exactly 1+4 */

enum { VOL_NONE = -1, VOL_RWD, VOL_DOWN, VOL_FWD, VOL_UP, VOL_BOTH };
static int decode_vol(int v)
{
	if (v < 200) return VOL_NONE;
	if (v < 560) return VOL_RWD;   /* ~404 */
	if (v < 950) return VOL_DOWN;  /* ~729 */
	if (v < 1500) return VOL_FWD;  /* ~1220 */
	if (v < 1910) return VOL_UP;   /* ~1820 */
	return VOL_BOTH;               /* ~1998 */
}

/* --------------------------------------------------------------- codecs ---- */
static const struct device *const i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static bool g_tas_ok;
static int g_hp_on;               /* CS42L42 up */
static volatile int g_hp_in = -1; /* headphones plugged */

static int tas_wr(uint8_t reg, uint8_t val) { return i2c_reg_write_byte(i2c_bus, TAS2505_ADDR, reg, val); }
static void tas_page(uint8_t p) { (void)tas_wr(0x00, p); }

static void tas_set_speaker(int on)
{
	tas_page(0x01);
	(void)tas_wr(0x2D, on ? 0x02 : 0x00);
	tas_page(0x00);
}

/* TAS2505 speaker amp (TI SLAU472C 5.1): PLL locked to the 3.072 MHz bit clock, J=32 ->
 * 98.304 MHz, NDAC 2 x MDAC 8 x DOSR 128 -> 48 kHz. 24-bit I2S slave. */
static bool tas2505_configure(void)
{
	int rc = 0;
	tas_page(0x00);
	rc |= tas_wr(0x01, 0x01);   /* software reset */
	k_msleep(5);
	tas_page(0x01);
	rc |= tas_wr(0x02, 0x00);   /* LDO 1.8 V, level shifters up */
	tas_page(0x00);
	rc |= tas_wr(0x04, 0x07);   /* PLL_CLKIN = BCLK, CODEC_CLKIN = PLL */
	rc |= tas_wr(0x05, 0x91);   /* PLL on, P=1, R=1 */
	rc |= tas_wr(0x06, 0x20);   /* J = 32 */
	rc |= tas_wr(0x07, 0x00);
	rc |= tas_wr(0x08, 0x00);
	k_msleep(15);
	rc |= tas_wr(0x0B, 0x82);   /* NDAC = 2 */
	rc |= tas_wr(0x0C, 0x88);   /* MDAC = 8 */
	rc |= tas_wr(0x0D, 0x00);
	rc |= tas_wr(0x0E, 0x80);   /* DOSR = 128 */
	rc |= tas_wr(0x1B, 0x20);   /* I2S, 24-bit, BCLK and WCLK in (SP-1-dev wiki) */
	rc |= tas_wr(0x1C, 0x00);
	rc |= tas_wr(0x3C, 0x02);   /* PRB_P2 */
	rc |= tas_wr(0x3F, 0x90);   /* DAC on, left data */
	rc |= tas_wr(0x41, 0x00);   /* 0 dB */
	rc |= tas_wr(0x40, 0x04);   /* unmuted */
	tas_page(0x01);
	rc |= tas_wr(0x01, 0x10);
	rc |= tas_wr(0x0A, 0x00);
	rc |= tas_wr(0x0C, 0x04);
	rc |= tas_wr(0x16, 0x00);
	rc |= tas_wr(0x18, 0x00);
	rc |= tas_wr(0x09, 0x20);
	rc |= tas_wr(0x10, 0x00);
	rc |= tas_wr(0x2E, 0x00);
	rc |= tas_wr(0x30, 0x30);   /* class-D gain +18 dB, as the looper */
	rc |= tas_wr(0x2D, 0x02);   /* speaker driver on */
	tas_page(0x00);
	k_msleep(10);
	g_tas_ok = rc == 0;
	return g_tas_ok;
}

static bool tpw(uint16_t reg, uint8_t val)   /* CS42L42 paged write */
{
	uint8_t p[2] = { 0x00, (uint8_t)(reg >> 8) };
	uint8_t b[2] = { (uint8_t)reg, val };
	if (i2c_write(i2c_bus, p, 2, CS42L42_ADDR) != 0) return false;
	return i2c_write(i2c_bus, b, 2, CS42L42_ADDR) == 0;
}

static bool tpr(uint16_t reg, uint8_t *val)
{
	uint8_t p[2] = { 0x00, (uint8_t)(reg >> 8) };
	uint8_t o = (uint8_t)reg;
	if (i2c_write(i2c_bus, p, 2, CS42L42_ADDR) != 0) return false;
	return i2c_write_read(i2c_bus, CS42L42_ADDR, &o, 1, val, 1) == 0;
}

static int hp_detect(void)
{
	uint8_t st;
	if (!tpr(0x1B77, &st)) return -1;
	return (st >> 7) & 1;
}

/* CS42L42 headphone codec: Tim Knapen's wiki sequence (the I2S frame master at 48 kHz from the
 * 3.072 MHz SCLK, 24-bit channels), as the looper runs it. */
static void hp_codec_init(void)
{
	uint8_t id = 0, st = 0;
	g_hp_on = 0;
	pin_out(CS42_RST_PORT, CS42_RST_PIN, 0);
	k_msleep(5);
	pin_out(CS42_RST_PORT, CS42_RST_PIN, 1);
	k_msleep(10);
	(void)tpr(0x1001, &id);
	if (id != 0x42) return;
	(void)tpw(0x1508, 0x10);
	(void)tpw(0x1504, 0x80);
	(void)tpw(0x1505, 0x3E);
	(void)tpw(0x150A, 0x7D);
	(void)tpw(0x1009, 0x00);
	(void)tpw(0x1201, 0x01);
	(void)tpw(0x120A, 0x01);
	(void)tpw(0x120B, 0x01);
	(void)tpw(0x1501, 0x01);
	(void)tpw(0x1107, 0x01);
	for (int t = 0; t < 10; t++) {
		k_msleep(1);
		if (tpr(0x1109, &st) && st == 0x02) break;
	}
	(void)tpw(0x1007, 0x13);
	(void)tpw(0x1203, 0x1F);
	(void)tpw(0x1205, 0x3F);
	(void)tpw(0x1207, 0x34);   /* ASP master */
	(void)tpw(0x1208, 0x1A);
	(void)tpw(0x2A02, 0x02);   /* channel 1: 24-bit */
	(void)tpw(0x2A05, 0x42);   /* channel 2: 24-bit */
	(void)tpw(0x2601, 0x4C);
	(void)tpw(0x2609, 0x4C);
	(void)tpw(0x2A01, 0x0C);
	(void)tpw(0x240E, 0x01);
	(void)tpw(0x2301, 0x00);
	(void)tpw(0x2303, 0x00);
	(void)tpw(0x1101, 0x96);   /* power up */
	k_msleep(10);
	(void)tpw(0x1121, 0x41);
	(void)tpw(0x1B74, 0x03);
	(void)tpw(0x1129, 0x01);
	(void)tpw(0x2001, 0x0D);   /* muted while the rest settles */
	(void)tpw(0x1F06, 0x84);
	(void)tpw(0x2301, 0x00);
	(void)tpw(0x2303, 0x00);
	(void)tpw(0x1B73, 0xC2);   /* tip sense: headphone detect */
	(void)tpw(0x1B75, 0x9F);
	(void)tpw(0x2001, 0x01);   /* unmute */
	g_hp_on = 1;
}

/* -------------------------------------------------------------- USB audio ----
 * The host streams 48 kHz stereo, 16-bit (alternate setting 1) or 24-bit (alternate setting 2),
 * whichever it supports: Linux handhelds such as the TrimUI Brick open the card as 16-bit. Either
 * way it lands in the ring as 24-bit frames; the audio thread drains it. The ring's
 * fill steers the explicit feedback, so the host sends at the rate the I2S bus really runs. */
#define USB_FRAME_BYTES 6u
#define USB_RING_FRAMES 1024u
#define FB_SETPOINT     256   /* ~5.3 ms of input held back against USB jitter */
RING_BUF_DECLARE(usb_ring, USB_RING_FRAMES * USB_FRAME_BYTES);

static volatile bool g_usb_streaming;
static volatile uint32_t g_ring_under, g_ring_over, g_rx_nobuf;

#define FB_FRAC        14
#define I2S_HZ         48000u
#define FB_TRUE        ((uint32_t)(((uint64_t)I2S_HZ << FB_FRAC) / 1000u))
#define FB_MIN         (FB_TRUE - (1u << FB_FRAC))
#define FB_MAX         (FB_TRUE + (1u << FB_FRAC))
#define FILL_Q         8
#define FILL_EMA_SHIFT 6
#define FB_KP          3

static atomic_t g_fb = ATOMIC_INIT(FB_TRUE);
static int32_t g_fill_avg;
static volatile bool g_fb_running;

static void feedback_reset(void)
{
	g_fill_avg = (int32_t)FB_SETPOINT << FILL_Q;
	atomic_set(&g_fb, (atomic_val_t)FB_TRUE);
}

static void feedback_update(void)
{
	int frames = (int)(ring_buf_size_get(&usb_ring) / USB_FRAME_BYTES);
	g_fill_avg += (((int32_t)frames << FILL_Q) - g_fill_avg) >> FILL_EMA_SHIFT;
	int err = (g_fill_avg >> FILL_Q) - FB_SETPOINT;
	int32_t fb = (int32_t)FB_TRUE - err * FB_KP;
	if (fb > (int32_t)FB_MAX) fb = (int32_t)FB_MAX;
	if (fb < (int32_t)FB_MIN) fb = (int32_t)FB_MIN;
	atomic_set(&g_fb, (atomic_val_t)fb);
}

#define UAC2_IN_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(in_terminal))
#define UAC2_MAX_PKT        ((48 + 1) * USB_FRAME_BYTES)   /* the 24-bit packet, the larger one */

extern volatile uint8_t uac2_sp1_alt;   /* usbd_uac2.c: 1 = 16-bit, 2 = 24-bit */
K_MEM_SLAB_DEFINE_STATIC(uac2_rx_slab, ROUND_UP(UAC2_MAX_PKT, UDC_BUF_GRANULARITY), 16, UDC_BUF_ALIGN);

static const struct device *const uac2_dev = DEVICE_DT_GET(DT_NODELABEL(uac2_speaker));

static void uac2_terminal_update_cb(const struct device *dev, uint8_t terminal, bool enabled,
				    bool microframes, void *user_data)
{
	ARG_UNUSED(dev); ARG_UNUSED(microframes); ARG_UNUSED(user_data);
	if (terminal != UAC2_IN_TERMINAL_ID) return;
	if (enabled) {
		k_sched_lock();
		ring_buf_reset(&usb_ring);
		k_sched_unlock();
		feedback_reset();
		g_fb_running = true;
		g_usb_streaming = true;
	} else {
		g_usb_streaming = false;
		g_fb_running = false;
	}
}

static void *uac2_get_recv_buf(const struct device *dev, uint8_t terminal, uint16_t size, void *user_data)
{
	ARG_UNUSED(dev); ARG_UNUSED(user_data); ARG_UNUSED(size);
	void *buf = NULL;
	if (terminal == UAC2_IN_TERMINAL_ID && g_usb_streaming &&
	    k_mem_slab_alloc(&uac2_rx_slab, &buf, K_NO_WAIT) != 0) {
		buf = NULL;
		g_rx_nobuf++;
	}
	return buf;
}

static void uac2_data_recv_cb(const struct device *dev, uint8_t terminal, void *buf, uint16_t size, void *user_data)
{
	ARG_UNUSED(dev); ARG_UNUSED(terminal); ARG_UNUSED(user_data);
	if (g_usb_streaming && size) {
		const uint8_t *src = (const uint8_t *)buf;
		static uint8_t wide[UAC2_MAX_PKT];
		if (uac2_sp1_alt == 1) {
			/* 16-bit: widen each sample to 24-bit (low byte 0) */
			uint32_t n = size / 2u;
			if (n > UAC2_MAX_PKT / 3u) n = UAC2_MAX_PKT / 3u;
			for (uint32_t i = 0; i < n; i++) {
				wide[3 * i] = 0;
				wide[3 * i + 1] = src[2 * i];
				wide[3 * i + 2] = src[2 * i + 1];
			}
			src = wide;
			size = (uint16_t)(n * 3u);
		}
		if (ring_buf_space_get(&usb_ring) >= size)
			(void)ring_buf_put(&usb_ring, src, size);
		else
			g_ring_over++;   /* drop the whole packet: one clean 1 ms gap */
	}
	k_mem_slab_free(&uac2_rx_slab, buf);
	if (g_fb_running) feedback_update();
}

static void uac2_buf_release_cb(const struct device *dev, uint8_t terminal, void *buf, void *user_data)
{
	ARG_UNUSED(dev); ARG_UNUSED(terminal); ARG_UNUSED(buf); ARG_UNUSED(user_data);
}

static uint32_t uac2_feedback_cb(const struct device *dev, uint8_t terminal, void *user_data)
{
	ARG_UNUSED(dev); ARG_UNUSED(terminal); ARG_UNUSED(user_data);
	return (uint32_t)atomic_get(&g_fb);
}

static void uac2_sof_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev); ARG_UNUSED(user_data);
}

static struct uac2_ops sp1_uac2_ops = {
	.sof_cb = uac2_sof_cb,
	.terminal_update_cb = uac2_terminal_update_cb,
	.get_recv_buf = uac2_get_recv_buf,
	.data_recv_cb = uac2_data_recv_cb,
	.buf_release_cb = uac2_buf_release_cb,
	.feedback_cb = uac2_feedback_cb,
};

static void usb_audio_start(void)
{
	if (!device_is_ready(uac2_dev)) return;
	usbd_uac2_set_ops(uac2_dev, &sp1_uac2_ops, NULL);
	struct usbd_context *usbd = sample_usbd_init_device(NULL);
	if (usbd == NULL) return;
	(void)usbd_device_set_bcd_device(usbd, 0x0200);
	(void)usbd_enable(usbd);
}

/* ------------------------------------------------------------ audio thread ----
 * I2S slave on both clocks (the oscillator drives SCLK, the CS42L42 masters LRCK at exactly
 * 48 kHz). 24-bit samples, one per 32-bit word, sign-extended. Blocks of 128 frames = 2.67 ms. */
#define BLK_FRAMES 128
#define BLK_BYTES  (BLK_FRAMES * 2 * (int)sizeof(int32_t))
K_MEM_SLAB_DEFINE(tx_slab, BLK_BYTES, 4, 4);
static const struct device *const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
static K_THREAD_STACK_DEFINE(audio_stack, 4096);
static struct k_thread audio_tcb;
static volatile bool g_audio_started;
static int g_i2s_rc = 1;

static fx_engine g_fx;                       /* a few KB of state */
static volatile float g_fader[4] = { 0.0f, 0.5f, 0.0f, 0.0f };
static volatile float g_param2[4] = { 0.5f, 0.5f, 0.5f, 0.5f };
static volatile int g_page;                  /* 0: Main amounts, 1: Secondary parameters */
static volatile bool g_bypass;
static volatile bool g_fx_off[4];            /* track buttons switch each effect off and on */
static volatile int g_vol_db = -6;           /* output volume, 3 dB steps, -60..0 */
static volatile bool g_muted;                /* power-off fade */
static float g_vol_lin;
static bool g_primed;

/* diagnostics: written by the audio thread, read by the main thread */
static volatile uint32_t g_blk_us_max, g_blk_us_last, g_blk_over, g_zero_pad, g_i2s_fail;

static float bufl[BLK_FRAMES], bufr[BLK_FRAMES];
static uint8_t g_in[BLK_FRAMES * USB_FRAME_BYTES];

static inline int32_t to_s24(float x)
{
	x *= 8388607.0f;
	if (x > 8388607.0f) x = 8388607.0f;
	if (x < -8388608.0f) x = -8388608.0f;
	return (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

static void audio_block(int32_t *out)
{
	uint32_t got = 0;
	static int underflow_count = 0;
	if (g_usb_streaming) {
		uint32_t fill = ring_buf_size_get(&usb_ring) / USB_FRAME_BYTES;
		if (!g_primed && fill >= FB_SETPOINT) {
			g_primed = true;
			underflow_count = 0;
		}
		if (g_primed) {
			got = ring_buf_get(&usb_ring, g_in, sizeof(g_in)) / USB_FRAME_BYTES;
			if (got < BLK_FRAMES) {
				g_ring_under++;
				if (got == 0) {
					if (++underflow_count >= 4) {
						g_primed = false;
					}
				} else {
					underflow_count = 0;
				}
			} else {
				underflow_count = 0;
			}
		}
	} else {
		g_primed = false;
		underflow_count = 0;
	}

	for (uint32_t i = 0; i < got; i++) {
		const uint8_t *p = &g_in[i * 6];
		int32_t l = (int32_t)(((uint32_t)p[0] << 8) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 24)) >> 8;
		int32_t r = (int32_t)(((uint32_t)p[3] << 8) | ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 24)) >> 8;
		bufl[i] = (float)l * (1.0f / 8388608.0f);
		bufr[i] = (float)r * (1.0f / 8388608.0f);
	}
	if (got < BLK_FRAMES) {
		g_zero_pad += BLK_FRAMES - got;
		float last_l = got > 0 ? bufl[got - 1] : 0.0f;
		float last_r = got > 0 ? bufr[got - 1] : 0.0f;
		for (uint32_t i = got; i < BLK_FRAMES; i++) {
			last_l *= 0.95f;
			last_r *= 0.95f;
			bufl[i] = last_l;
			bufr[i] = last_r;
		}
	}

	/* bypass and the track buttons send effects to rest through the engine, so they glide out
	 * and back; at rest the engine passes the input exactly (only delayed), and the latency never
	 * changes, so switching never clicks */
	static float last_f[4] = { -1.0f, -1.0f, -1.0f, -1.0f };
	static float last_p2[4] = { -1.0f, -1.0f, -1.0f, -1.0f };
	static const float rest[4] = { 0.0f, 0.5f, 0.0f, 0.0f };
	for (int k = 0; k < 4; k++) {
		float f = (g_bypass || g_fx_off[k]) ? rest[k] : g_fader[k];
		if (f != last_f[k]) {
			fx_set_fader(&g_fx, k, f);
			last_f[k] = f;
		}
		float p2 = g_param2[k];
		if (p2 != last_p2[k]) {
			fx_set_param2(&g_fx, k, p2);
			last_p2[k] = p2;
		}
	}
	fx_process(&g_fx, bufl, bufr, BLK_FRAMES);

	/* volume after the limiter, ramped across the block */
	float target = g_muted ? 0.0f : fx_db2lin((float)g_vol_db);
	float step = (target - g_vol_lin) * (1.0f / BLK_FRAMES);
	for (int i = 0; i < BLK_FRAMES; i++) {
		g_vol_lin += step;
		out[2 * i] = to_s24(bufl[i] * g_vol_lin);
		out[2 * i + 1] = to_s24(bufr[i] * g_vol_lin);
	}
	g_vol_lin = target;
}

static void audio_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	struct i2s_config cfg = {
		.word_size = 24,
		.channels = 2,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE,
		.frame_clk_freq = I2S_HZ,
		.mem_slab = &tx_slab,
		.block_size = BLK_BYTES,
		.timeout = 2000,
	};
	if (!device_is_ready(i2s_dev)) { g_i2s_rc = -100; return; }
	g_i2s_rc = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
	if (g_i2s_rc != 0) return;

	for (int i = 0; i < 4; i++) {
		void *blk;
		if (k_mem_slab_alloc(&tx_slab, &blk, K_FOREVER) != 0) continue;
		memset(blk, 0, BLK_BYTES);
		if (i2s_write(i2s_dev, blk, BLK_BYTES) != 0) k_mem_slab_free(&tx_slab, blk);
	}
	i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	g_audio_started = true;

	int wfail = 0;
	while (1) {
		void *blk;
		if (k_mem_slab_alloc(&tx_slab, &blk, K_FOREVER) != 0) continue;
		feed_wdt();
		uint32_t c0 = DWT->CYCCNT;
		audio_block((int32_t *)blk);
		uint32_t us = (DWT->CYCCNT - c0) / 64u;
		g_blk_us_last = us;
		if (us > g_blk_us_max) g_blk_us_max = us;
		if (us > 2666u) g_blk_over++;
		k_yield();
		if (i2s_write(i2s_dev, blk, BLK_BYTES) != 0) {
			g_i2s_fail++;
			k_mem_slab_free(&tx_slab, blk);
			/* failsafe: an I2S stream that stopped would stay silent; restart it */
			if (++wfail >= 8) {
				wfail = 0;
				(void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
				for (int i = 0; i < 4; i++) {
					void *pb;
					if (k_mem_slab_alloc(&tx_slab, &pb, K_NO_WAIT) != 0) break;
					memset(pb, 0, BLK_BYTES);
					if (i2s_write(i2s_dev, pb, BLK_BYTES) != 0) k_mem_slab_free(&tx_slab, pb);
				}
				(void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
			}
			continue;
		}
		wfail = 0;
	}
}

static void audio_init(void)
{
	fx_init(&g_fx, (float)I2S_HZ);
	g_vol_lin = 0.0f;   /* fades in from silence */
	pin_out(OSC_EN_PORT, OSC_EN_PIN, 1);
	k_msleep(5);
	k_thread_create(&audio_tcb, audio_stack, K_THREAD_STACK_SIZEOF(audio_stack), audio_thread,
			NULL, NULL, NULL, K_PRIO_PREEMPT(0), K_FP_REGS, K_NO_WAIT);
	for (int i = 0; i < 100 && !g_audio_started; i++) k_msleep(2);
}

/* ----------------------------------------------------------- power / DFU ---- */
static void power_off(void)
{
	g_muted = true;   /* fade the output so the codecs go down on silence */
	for (int i = 3; i >= 0; i--) {
		led_set(g_sta_duty, i, 0);
		led_set(g_trk_duty, i, 0);
		feed_wdt();
		k_msleep(80);
	}
	leds_shutdown();
	while (pwr_pressed()) { feed_wdt(); k_msleep(20); }
	k_msleep(60);
	leds_shutdown();

	/* the amp, the codec, the flash rail and the oscillator are separate chips: put them down */
	(void)tpw(0x2001, 0x0D);
	tas_page(0x01);
	(void)tas_wr(0x30, 0x00);
	feed_wdt();
	k_msleep(30);
	tas_page(0x00);
	(void)tas_wr(0x01, 0x01);
	pin_out(CS42_RST_PORT, CS42_RST_PIN, 0);
	pin_out(EMMC_RST_PORT, EMMC_RST_PIN, 0);
	pin_out(NRF_P0, EMMC_VCCQ_PIN, 0);
	pin_out(OSC_EN_PORT, OSC_EN_PIN, 0);

	pin_in_pullup(PWR_PORT, PWR_PIN, 1);   /* the button wakes the chip */
	feed_wdt();
	if (usb_present()) NVIC_SystemReset(); /* plugged in: back to the charge standby */
	NRF_POWER->RESETREAS = 0xFFFFFFFFu;
	__DSB();
	NRF_POWER->SYSTEMOFF = 1u;
	__DSB();
	for (;;) { }
}

/* hold Track 1 + Track 4 through the reset: the bootloader's own scan takes it into DFU */
static void enter_dfu(void)
{
	g_muted = true;
	k_msleep(20);
	leds_off();
	for (int i = 0; i < 4; i++) led_set(g_trk_duty, i, LED_PERIOD);
	NRF_POWER->GPREGRET = 0x57u;
	__DSB();
	NVIC_SystemReset();
	for (;;) { }
}

/* ---------------------------------------------------------- status line ---- */
static const struct device *const cdc = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static bool cdc_open(void)
{
	uint32_t dtr = 0;
	if (!device_is_ready(cdc)) return false;
	if (uart_line_ctrl_get(cdc, UART_LINE_CTRL_DTR, &dtr) != 0) return false;
	return dtr != 0;
}

static void status_line(void)
{
	if (!cdc_open()) return;
	uint32_t fill = ring_buf_size_get(&usb_ring) / USB_FRAME_BYTES;
	printk("SP-1FX f=%d,%d,%d,%d byp=%d vol=%d usb=%d fill=%u under=%u over=%u nobuf=%u pad=%u "
	       "blk=%u/%uus over=%u i2s=%d/%u hp=%d/%d tas=%d gr=%d.%ddB bat=%d rr=%08x flt=%08x@%08x\n",
	       g_adc[2], g_adc[3], g_adc[4], g_adc[5], g_bypass, g_vol_db, g_usb_streaming, fill,
	       g_ring_under, g_ring_over, g_rx_nobuf, g_zero_pad, g_blk_us_last, g_blk_us_max,
	       g_blk_over, g_i2s_rc, g_i2s_fail, g_hp_on, g_hp_in, g_tas_ok,
	       (int)(-fx_lin2db(1.0f - g_fx.limiter.meter + 1e-6f)),
	       (int)(-fx_lin2db(1.0f - g_fx.limiter.meter + 1e-6f) * 10.0f) % 10,
	       g_adc[CH_BATT], g_resetreas, g_last_fault, g_last_fault_pc);
	g_blk_us_max = 0;
}

/* ----------------------------------------------------------------- main ---- */
static void wdt_prewarn(const struct device *dev, int ch)
{
	ARG_UNUSED(dev); ARG_UNUSED(ch);
	g_fault_reason = 0xD0900000u;
	g_fault_pc = (uint32_t)k_current_get();
	g_fault_key = 0xFA17FA17u;
}

/* off, but booted (button press, or plugged in): hold FUNCTION to power on, or show the battery
 * while charging; on battery with the button let go, power straight back off. */
static void standby(void)
{
	int64_t hold_t = -1;
	uint32_t blink = 0;
	int bavg = -1;
	static const int batt_thr[3] = { 2020, 2140, 2260 };
	while (1) {
		feed_wdt();
		if (pwr_pressed()) {
			int64_t now = k_uptime_get();
			if (hold_t < 0) hold_t = now;
			else if (now - hold_t >= HOLD_ON_MS) break;
			int lit = (int)(((now - hold_t) * 4) / HOLD_ON_MS) + 1;
			for (int i = 0; i < 4; i++) led_set(g_sta_duty, i, i < lit ? LED_DIM : 0);
		} else {
			hold_t = -1;
			if (!usb_present()) power_off();
			if (adc_scan()) bavg = bavg < 0 ? g_adc[CH_BATT] : bavg + (g_adc[CH_BATT] - bavg) / 8;
			int lvl = 1;
			for (int k = 0; k < 3; k++) if (bavg > batt_thr[k]) lvl = k + 2;
			int bl = ((++blink / 12u) & 1u) == 0u;
			for (int i = 0; i < 4; i++) {
				int on = i < lvl - 1 ? 1 : i == lvl - 1 ? (charging() ? bl : 1) : 0;
				led_set(g_sta_duty, i, on ? LED_DIM : 0);
			}
		}
		k_msleep(40);
	}
	leds_off();
	while (pwr_pressed()) { feed_wdt(); k_msleep(20); }
}

int main(void)
{
	g_resetreas = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = 0xFFFFFFFFu;   /* cleared every boot: the bootloader reads it */
	if (g_fault_key == 0xFA17FA17u) {
		g_last_fault = g_fault_reason;
		g_last_fault_pc = g_fault_pc;
		g_fault_key = 0u;
	}
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	k_thread_priority_set(k_current_get(), K_PRIO_PREEMPT(2));

	pin_in_pullup(PWR_PORT, PWR_PIN, 0);
	charger_init();
	leds_off();
	led_pwm_one(NRF_PWM2, sta_leds, g_sta_duty);
	led_pwm_one(NRF_PWM3, trk_leds, g_trk_duty);

	/* the bootloader started the watchdog (5 s); keep it fed */
	const struct device *wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	if (device_is_ready(wdt)) {
		wdt_install_timeout(wdt, &(struct wdt_timeout_cfg){ .window.max = 4000, .callback = wdt_prewarn });
		wdt_setup(wdt, 0);
	}
	feed_wdt();
	controls_init();

	/* after a watchdog reset or a crash, come straight back up; otherwise ask for the hold */
	if (!(g_resetreas & POWER_RESETREAS_DOG_Msk) && g_last_fault == 0xFFFFFFFFu) standby();

	pin_out(CS42_RST_PORT, CS42_RST_PIN, 1);
	pin_out(TAS_RST_PORT, TAS_RST_PIN, 1);
	k_msleep(20);
	audio_init();        /* oscillator on, I2S running: the codecs need the clock */
	if (device_is_ready(i2c_bus)) {
		tas2505_configure();
		hp_codec_init();
	}
	usb_audio_start();
	feed_wdt();

	/* hello: a sweep across the status row */
	for (int i = 0; i < 4; i++) { led_set(g_sta_duty, i, LED_DIM); feed_wdt(); k_msleep(70); }
	k_msleep(120);
	leds_off();

	float fpos[4] = { 0.0f, 0.5f, 0.0f, 0.0f };
	/* what the engine gets: fpos with a small deadband, so a fader at rest sends exactly the
	 * same value (ADC noise would otherwise make the engine recompute its filters all the time) */
	float fheld[4] = { 0.0f, 0.5f, 0.0f, 0.0f };
	/* pick-up: after a page change a fader only takes over its parameter once it reaches the
	 * value (or passes it); until then the parameter stays put. side[] is which side of the value
	 * the fader started on. */
	bool caught[4] = { true, true, true, true };
	int side[4] = { 0, 0, 0, 0 };
	int64_t pwr_t = -1, dfu_t = -1, vol_show_until = 0, page_show_until = 0, hp_next = 0, stat_next = 0;
	int last_vol = VOL_NONE, hp_votes = 0, play_n = 0, trk_cand = -1, trk_n = 0, odd_n = 0;
	bool play_armed = false, chord = false;
	uint32_t tick = 0;
	bool first = true;

	while (1) {
		feed_wdt();
		int64_t now = k_uptime_get();
		tick++;

		if (adc_scan()) {
			/* faders: 0..3700 -> 0..1, lightly smoothed (the engine smooths again) */
			for (int k = 0; k < 4; k++) {
				float p = (float)g_adc[CH_FADER0 + k] / FADER_TOP;
				p = p < 0.0f ? 0.0f : p > 1.0f ? 1.0f : p;
				fpos[k] = first ? p : fpos[k] + 0.35f * (p - fpos[k]);
				float q = fpos[k] > 0.996f ? 1.0f : fpos[k] < 0.004f ? 0.0f : fpos[k];
				float dq = q - fheld[k];
				if (first || dq > 0.003f || dq < -0.003f || q == 0.0f || q == 1.0f) fheld[k] = q;
				volatile float *dst = g_page == 0 ? &g_fader[k] : &g_param2[k];
				if (!caught[k]) {
					float d = fheld[k] - *dst;
					int sd = d < 0.0f ? -1 : 1;
					if (d < 0.03f && d > -0.03f) caught[k] = true;
					else if (side[k] == 0) side[k] = sd;
					else if (sd != side[k]) caught[k] = true;   /* swept past it */
				}
				if (caught[k]) *dst = fheld[k];
			}
			first = false;

			int vol = decode_vol(g_adc[CH_VOL]);

			/* Track 1 + Track 4 held: the bootloader */
			if (combo_1_4(g_adc[CH_TRACKS])) {
				if (dfu_t < 0) dfu_t = now;
				else if (now - dfu_t >= DFU_HOLD_MS) enter_dfu();
			} else {
				dfu_t = -1;
			}

			/* PLAY: bypass all; TRACK n: effect n off/on. A press counts once it has sat in its
			 * own band for two scans, and fires at true idle; anything else seen on the way (a
			 * chord, PLAY plus a track, the 1+4 bootloader hold) cancels it. */
			int tv = g_adc[CH_TRACKS];
			int st = single_track(tv);
			if (tv < 110) {
				if (!chord) {
					if (play_armed) g_bypass = !g_bypass;
					else if (trk_cand >= 0 && trk_n >= 2) g_fx_off[trk_cand] = !g_fx_off[trk_cand];
				}
				play_armed = false; trk_cand = -1; trk_n = 0; play_n = 0; odd_n = 0; chord = false;
			} else if (is_play(tv)) {
				if (++play_n >= 2 && trk_cand < 0) play_armed = true;
			} else if (st >= 0 && (trk_cand < 0 || trk_cand == st) && !play_armed) {
				trk_cand = st;
				trk_n++;
			} else if (st >= 0) {
				chord = true;   /* a second button, or PLAY then a track */
			} else {
				play_n = 0;
				/* between bands: a finger landing or lifting reads here for a scan; a code
				 * that stays here is a chord */
				odd_n++;
				if (odd_n >= 2) chord = true;
			}

			if (vol != last_vol) {
				if (vol == VOL_UP && g_vol_db < 0) g_vol_db += 3;
				if (vol == VOL_DOWN && g_vol_db > -60) g_vol_db -= 3;
				if (vol == VOL_UP || vol == VOL_DOWN) vol_show_until = now + 1000;
				int np = vol == VOL_FWD ? 1 : vol == VOL_RWD ? 0 : g_page;
				if (vol == VOL_FWD || vol == VOL_RWD) page_show_until = now + 600;
				if (np != g_page) {
					g_page = np;
					for (int k = 0; k < 4; k++) { caught[k] = false; side[k] = 0; }
				}
			}
			last_vol = vol;
		}

		/* FUNCTION held: power off */
		if (pwr_pressed()) {
			if (pwr_t < 0) pwr_t = now;
			else if (now - pwr_t >= HOLD_OFF_MS) power_off();
		} else {
			pwr_t = -1;
		}

		/* headphones in: speaker off */
		if (g_hp_on && now >= hp_next) {
			hp_next = now + 250;
			int c = hp_detect();
			if (c >= 0) {
				hp_votes += c ? 1 : -1;
				hp_votes = hp_votes > 3 ? 3 : hp_votes < -3 ? -3 : hp_votes;
				int in = hp_votes >= 2 ? 1 : hp_votes <= -2 ? 0 : g_hp_in;
				if (in != g_hp_in && in >= 0) {
					g_hp_in = in;
					if (g_tas_ok) tas_set_speaker(!in);
				}
			}
		}

		/* lights */
		if (g_bypass) {
			int b = (int)((tick % 100u) < 50u ? (tick % 50u) : 50u - (tick % 50u)) * 3;
			for (int i = 0; i < 4; i++) { led_set(g_sta_duty, i, b); led_set(g_trk_duty, i, 0); }
		} else {
			if (g_page == 0) {
				/* page 1: the filter's light is dark across its flat middle (0.4..0.6) */
				float f1 = g_fader[1];
				float fd = f1 < 0.5f ? 0.5f - f1 : f1 - 0.5f;
				fd = fd < 0.1f ? 0.0f : (fd - 0.1f) / 0.4f;
				float amt[4] = { g_fader[0], fd, g_fader[2], g_fader[3] };
				for (int i = 0; i < 4; i++) {
					int v = g_fx_off[i] ? 6 : (int)(amt[i] * amt[i] * LED_DIM);
					if (!caught[i] && tick % 30u < 10u) v = 0;   /* waiting for pick-up */
					led_set(g_trk_duty, i, v);
				}
			} else {
				/* page 2: each second parameter */
				for (int i = 0; i < 4; i++) {
					int v = 4 + (int)(g_param2[i] * g_param2[i] * LED_DIM);
					if (!caught[i] && tick % 30u < 10u) v = 0;   /* waiting for pick-up */
					led_set(g_trk_duty, i, v);
				}
			}

			if (now < page_show_until) {
				/* Page transition animation */
				if (g_page == 1) {
					led_set(g_sta_duty, 0, 0); led_set(g_sta_duty, 1, 0);
					led_set(g_sta_duty, 2, LED_DIM); led_set(g_sta_duty, 3, LED_DIM);
				} else {
					led_set(g_sta_duty, 0, LED_DIM); led_set(g_sta_duty, 1, LED_DIM);
					led_set(g_sta_duty, 2, 0); led_set(g_sta_duty, 3, 0);
				}
			} else if (now < vol_show_until) {
				int lv = (g_vol_db + 60) * 4 / 60;   /* 0..4 */
				for (int i = 0; i < 4; i++) led_set(g_sta_duty, i, i < lv ? LED_DIM : 0);
			} else {
				float gr = fx_gain_reduction_db(&g_fx);
				static const float thr[4] = { 0.5f, 2.0f, 4.0f, 8.0f };
				for (int i = 0; i < 4; i++) {
					int on = gr >= thr[i] ? LED_DIM : 0;
					if (g_page == 1 && i == 3 && on == 0) {
						on = (tick % 40u < 20u) ? 40 : 0;
					}
					led_set(g_sta_duty, i, on);
				}
				g_fx.limiter.meter *= 0.85f;   /* the meter falls back between readings */
			}
		}

		if (now >= stat_next) {
			stat_next = now + 1000;
			status_line();
		}
		k_msleep(10);
	}
	return 0;
}
