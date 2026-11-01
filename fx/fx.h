/* SP-1 multi-effects engine: four effects in series, one per fader.
 *
 *   fader 1  DUB      70s dub reggae: big low end, dark top, tape wow and flutter, vinyl dust,
 *                     punchy bus compression and tape saturation. Down = off, up = more.
 *   fader 2  FILTER   centre = flat; down sweeps a high-pass up, up sweeps a low-pass down.
 *   fader 3  REVERB   organic reverb (modulated feedback delay network). Down = dry, up = more
 *                     and longer.
 *   fader 4  LIMITER  mastering limiter, always on at -0.3 dBFS; the fader drives into it
 *                     (0 to +12 dB).
 *
 * Float, stereo, 48 kHz, any block size: parameters move every FX_CHUNK frames counted from the
 * start, so the output does not depend on how the audio is cut into blocks. No libm, no doubles,
 * no heap: an fx_engine is about 80 KB and should be a static. */
#ifndef SP1_FX_H
#define SP1_FX_H

#include <stdint.h>
#include "fx_math.h"

#define FX_SLOTS 4
#define FX_CHUNK 32

enum { FX_DUB, FX_FILTER, FX_REVERB, FX_LIMITER };

/* ---- 1: dub ---- */
#define DUB_DL 64      /* wow/flutter delay line, power of two */
#define DUB_CENTER 16  /* its centre tap: the slot's latency in frames */

typedef struct {
    float amt, gate, gate_target;
    int wpos;
    float dl[2][DUB_DL];
    float wow_c, wow_s, wow_rc, wow_rs, flt_c, flt_s, flt_rc, flt_rs;
    float wow_depth, flt_depth;
    fx_svf hp, shelf, cut, pres, lp;
    fx_svf_state st[2][5];
    float env, att, rel, thr, slope, knee, makeup;
    float drive, inv_drive, sat_mix, bias, bias_out, gain;
    float dc_x[2], dc_y[2], dc_r;
    uint32_t rng, rng_c[2];
    float dg;           /* the gain step between those */
    int cphase;         /* frames counted for the compressor's every-fourth-frame gain */
    float hiss, hiss_c, hiss_lp[2];
    uint32_t tick_th;   /* a tick when a 32-bit random number falls under this */
    float tick_p, tick_amp, tick_env[2], tick_dec;
    float pop_p, pop_amp, pop_env, pop_dec, pop_lp, pop_c;
    float trim;
    int dust_on; /* 1; the tests turn it off to look for clicks */
} fx_dub;

/* ---- 2: filter ---- */
typedef struct {
    float pos, gate, gate_target;
    int mode, mode_target; /* 0 high-pass, 1 low-pass */
    fx_svf s1, s2;
    fx_svf_state st[2][2];
} fx_filter;

/* ---- 3: reverb ---- */
/* Runs at half rate (24 kHz): the tail is darker than 7.5 kHz anyway, and it halves the CPU and
 * the memory. Lengths below are at 24 kHz. */
#define REV_LINES 8
#define REV_MAX 1408   /* longest line plus modulation room */
#define REV_PRE 512
#define REV_AP 4
#define REV_AP_MAX 256

typedef struct {
    float amt, wet;
    int live;
    float line[REV_LINES][REV_MAX];
    int len[REV_LINES], wp[REV_LINES];
    float gain[REV_LINES], damp_z[REV_LINES], damp;
    float pre[REV_PRE];
    int pre_len, pre_w;
    float ap[REV_AP][REV_AP_MAX];
    int ap_len[REV_AP], ap_pos[REV_AP];
    float lfo1_c, lfo1_s, lfo1_rc, lfo1_rs, lfo2_c, lfo2_s, lfo2_rc, lfo2_rs, mod_depth;
    float in_lp, in_lpc, in_hp, in_hpc;   /* input band: one-poles at 48 kHz */
    float half_in;                         /* the first frame of the pair */
    int half;                              /* 0: waiting for the pair's first frame */
    float wl0, wr0, wl1, wr1;              /* the last two wet outputs, for the 48 kHz in-between */
    float sr;                              /* the outside rate (48 kHz) */
} fx_reverb;

/* ---- 4: limiter ---- */
#define LIM_LA 48      /* look-ahead: 1 ms; latency LIM_LA - 1 frames */

typedef struct {
    float drive, drive_target, dstep, ceiling;
    float dl[2][LIM_LA];
    float red[LIM_LA];        /* gain reduction (1 - gain) needed per frame */
    int dq[LIM_LA], dq_head, dq_tail, dq_n; /* frames in the window whose reduction is not beaten */
    float box[LIM_LA], box_sum;
    int pos, n;
    float gr, rel;            /* reduction applied, release coefficient */
    float meter;              /* deepest reduction since last read, 0..1 */
} fx_limiter;

typedef struct {
    float sr;
    int phase;                /* frames into the current chunk */
    float fader[FX_SLOTS];    /* latest positions, 0..1 */
    fx_dub dub;
    fx_filter filter;
    fx_reverb reverb;
    fx_limiter limiter;
} fx_engine;

void fx_init(fx_engine *e, float sr);
void fx_set_fader(fx_engine *e, int slot, float pos);   /* pos 0..1 */
void fx_process(fx_engine *e, float *l, float *r, int n); /* in place */
int fx_latency(const fx_engine *e);                       /* frames */

#endif
