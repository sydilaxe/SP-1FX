/* SP-1 multi-effects engine: four effects in series, one per fader.
 *
 *   fader 1  DUB      70s dub reggae: big low end, dark top, tape wow and flutter, vinyl dust,
 *                     punchy bus compression and tape saturation. Down = off, up = more.
 *   fader 2  FILTER   the middle fifth is flat; down sweeps a low-pass down, up a high-pass up.
 *   fader 3  REVERB   organic reverb (modulated feedback delay network). Down = dry, up = more
 *                     and longer.
 *   fader 4  LIMITER  mastering limiter, always on at -0.3 dBFS; the fader drives into it
 *                     (0 to +12 dB).
 *
 * The sound: warm and a little lo-fi, but clean. In and out at 48 kHz; at the input a warm stage
 * (gentle tape saturation, a slight treble dip), then the effects run at 24 kHz between two
 * polyphase IIR half-band filters (flat to 11 kHz, -72 dB at 13 kHz, below -140 dB from 14 kHz):
 * a soft top without grit, and half the CPU.
 *
 * Float, stereo, any block size: parameters move every FX_CHUNK core frames counted from the start,
 * so the output does not depend on how the audio is cut into blocks. No libm, no doubles, no heap:
 * an fx_engine is about 55 KB and should be a static. */
#ifndef SP1_FX_H
#define SP1_FX_H

#include <stdint.h>
#include "fx_math.h"

#define FX_SLOTS 4
#define FX_CHUNK 16   /* core frames (24 kHz) between parameter updates */

enum { FX_DUB, FX_FILTER, FX_CRUSH, FX_LIMITER, FX_REVERB = FX_CRUSH };

/* ---- 1: dub ---- */
#define DUB_DL 64      /* wow/flutter delay line, power of two */
#define DUB_CENTER 16  /* its centre tap: the slot's latency in frames */

typedef struct {
    float amt, gate, gate_target;
    int wpos;
    float dl[2][DUB_DL];
    float wow_c, wow_s, wow_rc, wow_rs, flt_c, flt_s, flt_rc, flt_rs;
    float wow_depth, flt_depth;
    float hp_c;
    fx_svf shelf, cut, pres, lp;
    fx_svf_state st[2][5];
    float env, att, rel, thr, slope, knee, makeup;
    float sc_lp[2], sc_c;   /* the detector's high-pass (sidechain): one-poles at 150 Hz */
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

/* ---- 3: bit reducer (sample rate & bit depth crusher) ---- */
typedef struct {
    float amt, wet;
    float bits;             /* effective bit depth: 16 down to 3 */
    float q_step;           /* quantization step */
    float inv_q_step;
    float rate_ratio;       /* downsample ratio: 1.0 (24kHz) down to ~0.06 (1.5kHz) */
    float hold_l, hold_r;   /* sample-and-hold accumulator */
    float phase;
    float gate, gate_target;
} fx_bitcrush;

typedef fx_bitcrush fx_reverb; /* backward compatibility alias */

/* ---- 4: limiter ---- */
#define LIM_LA 24      /* look-ahead: 1 ms at 24 kHz; latency LIM_LA - 1 core frames */
#define LIM_SEG 8      /* the peak hold works in segments of 8 frames */
#define LIM_NSEG (LIM_LA / LIM_SEG)

typedef struct {
    float amt;                /* slider 4 position: 0 = off, 1 = max drive */
    int active;               /* 0 when bypassed (slider at 0), 1 when limiting */
    float drive, drive_target, dstep, ceiling;
    float dl[2][LIM_LA];
    float seg[LIM_NSEG];      /* the deepest reduction needed in each of the last segments */
    float cur, held;          /* this segment so far; the finished segments */
    int si, sn;               /* segment index, frames into it */
    float box[LIM_LA], box_sum;
    int pos;
    float gr, rel;            /* reduction applied, release coefficient */
    float meter;              /* deepest reduction since last read, 0..1 */
} fx_limiter;

#define HB_C 10        /* half-band allpass coefficients */

typedef struct {
    float sr, cr;             /* outside rate (48 kHz), core rate (24 kHz) */
    int phase;                /* core frames into the current chunk */
    /* warm stage, at the outside rate */
    float warm_lp[2], warm_c;
    /* 48 -> 24 kHz: allpass states */
    float dy[2][HB_C], dx0[2], dx1[2], dfirst[2];
    int dodd;
    /* 24 -> 48 kHz: the same, and the pair's second frame */
    float uy[2][HB_C], ux0[2], ux1[2], usecond[2];
    int uodd;
    float fader[FX_SLOTS];    /* latest positions, 0..1 */
    fx_dub dub;
    fx_filter filter;
    union {
        fx_bitcrush crush;
        fx_bitcrush reverb;
    };
    fx_limiter limiter;
} fx_engine;

void fx_init(fx_engine *e, float sr);
void fx_set_fader(fx_engine *e, int slot, float pos);   /* pos 0..1 */
void fx_process(fx_engine *e, float *l, float *r, int n); /* in place */
int fx_latency(const fx_engine *e);                       /* frames */

#endif
