/* SP-1 multi-effects engine: four effects in series, one per fader, each with a second
 * parameter on page 2.
 *
 *   fader 1  DUB      70s dub: big low end, dark top, tape wow and flutter, bus compression and
 *                     tape saturation. Down = off, up = more.            Page 2: tape wobble speed.
 *   fader 2  FILTER   the middle fifth is flat; down sweeps a low-pass down, up a high-pass up.
 *                                                                        Page 2: resonance.
 *   fader 3  CRUSH    bit and sample-rate reducer. The bit depth follows the signal's level, so
 *                     quiet passages are crushed too instead of dropping out.
 *                     Down = off, up = fewer bits and a lower rate.      Page 2: sample-rate range.
 *   fader 4  LA-2A    optical leveling amplifier. Down = off, up = more peak reduction (with
 *                     makeup gain). Slow optical attack, two-stage program-dependent release.
 *                                                                        Page 2: compress -> limit.
 *
 * After the four, an always-on safety ceiling (-1 dBFS, 1 ms look-ahead) that only acts on
 * overs: it protects the speaker and headphones and is otherwise inaudible.
 *
 * Full bandwidth: everything runs at the outside rate (48 kHz) in 32-bit float. With every fader
 * at rest the output is the input, bit for bit, fx_latency() frames later.
 *
 * Float, stereo, any block size: parameters move every FX_CHUNK frames counted from the start, so
 * the output does not depend on how the audio is cut into blocks. No libm, no doubles, no heap:
 * an fx_engine should be a static. */
#ifndef SP1_FX_H
#define SP1_FX_H

#include <stdint.h>
#include "fx_math.h"

#define FX_SLOTS 4
#define FX_CHUNK 32   /* frames (48 kHz) between parameter updates: 0.67 ms */

enum { FX_DUB, FX_FILTER, FX_CRUSH, FX_LIMITER };

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
    fx_svf shelf, cut, lp;
    fx_svf_state st[2][5];  /* [0] rumble, [1] shelf, [2] cut, [4] top */
    float env, att, rel, thr, slope, knee, makeup;
    float sc_lp[2], sc_c;   /* the detector's high-pass (sidechain): one-poles at 150 Hz */
    float drive, inv_drive, sat_mix, bias, bias_out, gain;
    float dc_x[2], dc_y[2], dc_r;
    float dg;               /* the compressor's gain step per frame */
    int cphase;             /* frames counted for the compressor's every-fourth-frame gain */
    float trim;
    float speed_mod;        /* page 2: tape wobble speed (0..1, default 0.5) */
} fx_dub;

/* ---- 2: filter ---- */
typedef struct {
    float pos, gate, gate_target;
    int mode, mode_target; /* 0 high-pass, 1 low-pass */
    float res;             /* page 2: resonance amount (0..1, default 0.5) */
    fx_svf s1, s2;
    fx_svf_state st[2][2];
} fx_filter;

/* ---- 3: bit and sample-rate reducer ---- */
typedef struct {
    float amt, wet;
    float q_step, inv_q_step; /* quantizer step for a signal normalised to its level */
    float rate_ratio;         /* hold clock / sample rate: 1 down to about 1/24 */
    float rate_mod;           /* page 2: sample-rate range (0..1, default 0.5) */
    float env, env_rel;       /* the level the quantizer follows (stereo-linked peak) */
    float hold_l, hold_r;     /* sample-and-hold */
    float phase;
    float gate, gate_target;
} fx_bitcrush;

/* ---- 4: LA-2A optical leveler, then the safety ceiling ---- */
#define LIM_LA 48      /* safety look-ahead: 1 ms, and its latency */
#define LIM_SEG 8      /* segment size for the look-ahead peak hold */
#define LIM_NSEG (LIM_LA / LIM_SEG)

typedef struct {
    /* LA-2A */
    float pos;                /* fader: peak reduction 0..1 (0 = off) */
    float amt;                /* the same, smoothed */
    float mode;               /* page 2: 0 = compress (3:1), 1 = limit (10:1), default 0.5 */
    float thr, slope, knee, makeup_db, fade, tube;
    float det, det_att, det_rel;      /* fast rectifier feeding the optical cell */
    float gr_f, gr_s;         /* the cell, dB of reduction: fast part, slow (memory) part */
    float c_att, c_rel_f, c_chg_s, c_rel_s;   /* per 4 frames */
    float g, dg;              /* applied gain, glided between the every-fourth-frame updates */
    int cphase;
    int active;
    /* safety ceiling */
    float ceiling;
    float dl[2][LIM_LA];
    float seg[LIM_NSEG];      /* each segment's needed reduction, a ring */
    float acc_pk;             /* the peak of the segment coming in */
    int si, sn, pos_dl;
    float sgr, s_rel;         /* reduction at the last segment point, its release */
    float sg, sg_step;        /* gain now, and its step per frame */
    float meter;              /* deepest total reduction since last read, 0..1 (1 - gain) */
} fx_limiter;

typedef struct {
    float sr;
    int phase;                /* frames into the current chunk */
    float fader[FX_SLOTS];    /* page 1 positions, 0..1 */
    float param2[FX_SLOTS];   /* page 2 positions, 0..1 */
    fx_dub dub;
    fx_filter filter;
    fx_bitcrush crush;
    fx_limiter limiter;
} fx_engine;

void fx_init(fx_engine *e, float sr);
void fx_set_fader(fx_engine *e, int slot, float pos);   /* page 1, 0..1 */
void fx_set_param2(fx_engine *e, int slot, float pos);  /* page 2, 0..1 */
void fx_process(fx_engine *e, float *l, float *r, int n); /* in place */
int fx_latency(const fx_engine *e);                       /* frames */
float fx_gain_reduction_db(const fx_engine *e);           /* LA-2A + ceiling, from the meter */

#endif
