/* SP-1 multi-effects engine. See fx.h for the faders. */
#include "fx.h"

#ifdef FX_PROFILE   /* tools/bench_m4.py: keep the stages apart so their cost can be read */
#define STAGE __attribute__((noinline))
#else
#define STAGE
#endif

/* quadrature oscillator: a rotation per frame, renormalised each chunk */
static void osc_init(float *c, float *s, float *rc, float *rs, float hz, float phase, float sr)
{
    /* cos and sin of small angles from the tan series: t = tan(w/2) */
    float t = fx_tanf(FX_PI * hz / sr), t2 = t * t;
    *rc = (1.0f - t2) / (1.0f + t2);
    *rs = 2.0f * t / (1.0f + t2);
    float tp = fx_tanf(0.5f * phase), tp2 = tp * tp; /* phase in 0..pi */
    *c = (1.0f - tp2) / (1.0f + tp2);
    *s = 2.0f * tp / (1.0f + tp2);
}

static inline void osc_step(float *c, float *s, float rc, float rs)
{
    float nc = *c * rc - *s * rs;
    *s = *s * rc + *c * rs;
    *c = nc;
}

static inline void osc_norm(float *c, float *s)
{
    float m = 1.5f - 0.5f * (*c * *c + *s * *s); /* one Newton step toward 1 */
    *c *= m; *s *= m;
}

/* one-pole smoothing coefficient for a time constant, applied once per chunk */
static float chunk_coef(float seconds, float sr)
{
    return 1.0f - fx_exp2f(-1.44269504f * (float)FX_CHUNK / (seconds * sr));
}

static float sample_coef(float seconds, float sr)
{
    return 1.0f - fx_exp2f(-1.44269504f / (seconds * sr));
}

/* ================================================================== 1: dub ==== */

#define DUB_DEAD 0.03f  /* fader below this: the slot is bypassed */

static void dub_init(fx_dub *d, float sr)
{
    *d = (fx_dub){0};
    d->dust_on = 1;
    d->rng = 0x9e3779b9u;
    d->rng_c[0] = 0x6d2b79f5u;
    d->rng_c[1] = 0x1b873593u;
    d->gain = 1.0f;
    osc_init(&d->wow_c, &d->wow_s, &d->wow_rc, &d->wow_rs, 0.55f, 0.0f, sr);
    osc_init(&d->flt_c, &d->flt_s, &d->flt_rc, &d->flt_rs, 7.3f, 1.0f, sr);
    d->att = sample_coef(0.010f, sr);   /* slow attack lets the transient through: the punch */
    d->rel = sample_coef(0.120f, sr);
    d->knee = 6.0f;
    d->dc_r = 1.0f - 2.0f * FX_PI * 4.0f / sr;
    d->hiss_c = sample_coef(1.0f / (2.0f * FX_PI * 5000.0f), sr);
    d->tick_dec = fx_exp2f(-1.44269504f / (0.00015f * sr));
    d->pop_dec = fx_exp2f(-1.44269504f / (0.0020f * sr));
    d->pop_c = sample_coef(1.0f / (2.0f * FX_PI * 900.0f), sr);
}

STAGE static void dub_control(fx_dub *d, float pos, float sr)
{
    float target = pos < DUB_DEAD ? 0.0f : (pos - DUB_DEAD) / (1.0f - DUB_DEAD);
    d->amt += (target - d->amt) * chunk_coef(0.03f, sr);
    if (fx_absf(target - d->amt) < 1e-5f) d->amt = target;
    d->gate_target = pos < DUB_DEAD ? 0.0f : 1.0f;
    float a = d->amt;

    /* EQ: the low end of a dub mix, a de-mudded low-mid, a forward mid, a dark top */
    fx_svf_highpass(&d->hp, 22.0f + 18.0f * a, 0.7f, sr);
    fx_svf_lowshelf(&d->shelf, 95.0f, 0.71f, 8.0f * a, sr);
    fx_svf_bell(&d->cut, 320.0f, 0.9f, -4.0f * a, sr);
    fx_svf_bell(&d->pres, 1500.0f, 0.8f, 3.0f * a, sr);
    fx_svf_lowpass(&d->lp, 20000.0f * fx_exp2f(-1.62f * a), 0.62f, sr); /* 20k -> 6.5k */

    /* punch: stereo bus compressor, slow attack, threshold and ratio rise with the fader */
    d->thr = -6.0f - 12.0f * a;
    d->slope = 1.0f / (1.0f + 3.0f * a) - 1.0f;
    d->makeup = -d->slope * (-d->thr) * 0.45f;

    /* tape saturation, slightly asymmetric for even harmonics */
    d->drive = 1.0f + 2.5f * a;
    d->sat_mix = fx_clampf(4.0f * a, 0.0f, 1.0f);
    d->bias = 0.12f * a;
    d->bias_out = fx_softclip(d->bias);
    d->inv_drive = 1.0f / d->drive;

    /* tape wow and flutter */
    d->wow_depth = 9.0f * a;
    d->flt_depth = 1.2f * a;

    /* dust comes in late on the fader */
    float s = a * fx_sqrtf(a);
    d->hiss = s * fx_db2lin(-42.0f);
    d->tick_p = (1.5f + 16.0f * s) / sr * (a > 0.0f);
    d->tick_th = (uint32_t)(d->tick_p * 4294967040.0f);
    d->tick_amp = 0.10f * s;
    d->pop_p = 0.7f * s / sr;
    d->pop_amp = 0.35f * s;

    d->trim = fx_db2lin(-5.5f * a);

    osc_norm(&d->wow_c, &d->wow_s);
    osc_norm(&d->flt_c, &d->flt_s);
}

static inline float hermite(const float *b, float idx)
{
    int i = (int)idx;
    float f = idx - (float)i;
    float ym1 = b[(i - 1) & (DUB_DL - 1)], y0 = b[i & (DUB_DL - 1)];
    float y1 = b[(i + 1) & (DUB_DL - 1)], y2 = b[(i + 2) & (DUB_DL - 1)];
    float c1 = 0.5f * (y1 - ym1);
    float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
    return ((c3 * f + c2) * f + c1) * f + y0;
}

STAGE static void dub_process(fx_dub *d, float *l, float *r, int n, float sr)
{
    float dry[2][FX_CHUNK], x[2][FX_CHUNK], gain[FX_CHUNK], pop[FX_CHUNK];
    int wet_off = d->gate == 0.0f && d->gate_target == 0.0f;

    /* 1: the tape: write the delay line, read the dry centre tap and the wobbling wet tap */
    {
        int wpos = d->wpos;
        float wc = d->wow_c, ws = d->wow_s, fc = d->flt_c, fs = d->flt_s;
        const float wrc = d->wow_rc, wrs = d->wow_rs, frc = d->flt_rc, frs = d->flt_rs;
        const float wd = d->wow_depth, fd = d->flt_depth;
        float *b0 = d->dl[0], *b1 = d->dl[1];
        for (int i = 0; i < n; i++) {
            b0[wpos] = l[i];
            b1[wpos] = r[i];
            int tap = (wpos - DUB_CENTER) & (DUB_DL - 1);
            dry[0][i] = b0[tap];
            dry[1][i] = b1[tap];
            float t = wc * wrc - ws * wrs; ws = ws * wrc + wc * wrs; wc = t;
            t = fc * frc - fs * frs; fs = fs * frc + fc * frs; fc = t;
            if (!wet_off) {
                float idx = (float)(wpos + DUB_DL) - ((float)DUB_CENTER + wd * ws + fd * fs);
                x[0][i] = hermite(b0, idx);
                x[1][i] = hermite(b1, idx);
            }
            wpos = (wpos + 1) & (DUB_DL - 1);
        }
        d->wpos = wpos;
        d->wow_c = wc; d->wow_s = ws; d->flt_c = fc; d->flt_s = fs;
    }
    if (wet_off) {
        for (int i = 0; i < n; i++) { l[i] = dry[0][i]; r[i] = dry[1][i]; }
        return;
    }

    /* 2: the EQ, one channel at a time so the filter states stay in registers */
    for (int c = 0; c < 2; c++) {
        fx_svf_state *st = d->st[c];
        float h1 = st[0].ic1, h2 = st[0].ic2, s1 = st[1].ic1, s2 = st[1].ic2;
        float c1 = st[2].ic1, c2 = st[2].ic2, p1 = st[3].ic1, p2 = st[3].ic2;
        const fx_svf H = d->hp, S = d->shelf, C = d->cut, P = d->pres;
        float *v = x[c];
        for (int i = 0; i < n; i++) {
            float y = fx_svf_tick(H.a1, H.a2, H.a3, H.m0, H.m1, H.m2, &h1, &h2, v[i]);
            y = fx_svf_tick(S.a1, S.a2, S.a3, S.m0, S.m1, S.m2, &s1, &s2, y);
            y = fx_svf_tick(C.a1, C.a2, C.a3, C.m0, C.m1, C.m2, &c1, &c2, y);
            v[i] = fx_svf_tick(P.a1, P.a2, P.a3, P.m0, P.m1, P.m2, &p1, &p2, y);
        }
        st[0].ic1 = h1; st[0].ic2 = h2; st[1].ic1 = s1; st[1].ic2 = s2;
        st[2].ic1 = c1; st[2].ic2 = c2; st[3].ic1 = p1; st[3].ic2 = p2;
    }

    /* 3: the compressor: a peak envelope every frame, the gain every fourth frame, glided
     *    between; and the record's pops (shared by both channels) */
    {
        float env = d->env, g = d->gain;
        const float att = d->att, rel = d->rel, thr = d->thr, slope = d->slope, knee = d->knee;
        const float mk = d->makeup;
        float dg = d->dg;
        uint32_t rng = d->rng;
        float pe = d->pop_env, pl = d->pop_lp;
        const float pp = d->pop_p, pa = d->pop_amp, pd = d->pop_dec, pc = d->pop_c;
        const int dust = d->dust_on;
        int ph = d->cphase;
        for (int i = 0; i < n; i++) {
            float a0 = fx_absf(x[0][i]), a1 = fx_absf(x[1][i]);
            float pk = a0 > a1 ? a0 : a1;
            env += (pk - env) * (pk > env ? att : rel);
            if (ph == 0) {
                float over = fx_lin2db(env + 1e-9f) - thr, gr;
                if (2.0f * over < -knee) gr = 0.0f;
                else if (2.0f * over > knee) gr = slope * over;
                else { float k = over + 0.5f * knee; gr = slope * k * k * (0.5f / knee); }
                dg = (fx_db2lin(gr + mk) - g) * 0.25f;
            }
            ph = (ph + 1) & 3;
            g += dg;
            gain[i] = g;
            if (dust) {
                if (fx_rand01(&rng) < pp) pe = pa * (0.4f + 0.6f * fx_rand01(&rng));
                pl += pc * (pe * fx_noise(&rng) - pl);
                pe *= pd;
            }
            pop[i] = pl;
        }
        d->env = env; d->gain = g; d->rng = rng; d->cphase = ph; d->dg = dg; d->pop_env = pe; d->pop_lp = pl;
    }

    /* 4: tape saturation, the record's hiss and ticks, the dark top, the DC blocker, then the
     *    fade between dry and wet */
    float gate0 = d->gate;
    for (int c = 0; c < 2; c++) {
        float *out = c ? r : l;
        float *v = x[c];
        const float drive = d->drive, idr = d->inv_drive, mix = d->sat_mix, bias = d->bias;
        const float bout = d->bias_out, trim = d->trim, dcr = d->dc_r;
        const float hiss = d->hiss, hc = d->hiss_c, tamp = d->tick_amp, tdec = d->tick_dec;
        const uint32_t tth = d->tick_th;
        const int dust = d->dust_on;
        const fx_svf LP = d->lp;
        float lp1 = d->st[c][4].ic1, lp2 = d->st[c][4].ic2;
        float hl = d->hiss_lp[c], te = d->tick_env[c], dx = d->dc_x[c], dy = d->dc_y[c];
        uint32_t rng = d->rng_c[c];
        float gate = gate0, gt = d->gate_target, gstep = 1.0f / (0.02f * sr);
        for (int i = 0; i < n; i++) {
            float y = v[i] * gain[i];
            float t = (fx_softclip(y * drive + bias) - bout) * idr;
            y += mix * (t - y);
            if (dust) {
                float w = fx_noise(&rng);
                hl += hc * (w * hiss - hl);
                if (fx_rng(&rng) < tth) {
                    float u = fx_rand01(&rng);
                    te = tamp * u * u * u * (w < 0.0f ? -1.0f : 1.0f);
                }
                y += hl + te * w + pop[i];
                te *= tdec;
            }
            y = fx_svf_tick(LP.a1, LP.a2, LP.a3, LP.m0, LP.m1, LP.m2, &lp1, &lp2, y);
            float o = y - dx + dcr * dy;
            dx = y; dy = o;
            o *= trim;
            if (gate != gt) {
                gate += gate < gt ? gstep : -gstep;
                gate = fx_clampf(gate, 0.0f, 1.0f);
            }
            out[i] = dry[c][i] + gate * (o - dry[c][i]);
        }
        d->st[c][4].ic1 = lp1; d->st[c][4].ic2 = lp2;
        d->hiss_lp[c] = hl; d->tick_env[c] = te; d->dc_x[c] = dx; d->dc_y[c] = dy;
        d->rng_c[c] = rng;
        if (c == 1) d->gate = gate;
    }

    if (d->gate == 0.0f && d->gate_target == 0.0f) {
        /* fully off: start clean next time */
        for (int c = 0; c < 2; c++) {
            for (int k = 0; k < 5; k++) d->st[c][k] = (fx_svf_state){0};
            d->dc_x[c] = d->dc_y[c] = d->hiss_lp[c] = d->tick_env[c] = 0.0f;
        }
        d->env = d->pop_env = d->pop_lp = 0.0f;
        d->gain = 1.0f;
    }
}

/* =============================================================== 2: filter ==== */

#define FLT_DEAD 0.04f /* half-width of the flat zone around the centre */

STAGE static void filter_control(fx_filter *f, float pos, float sr)
{
    f->pos += (pos - f->pos) * chunk_coef(0.02f, sr);
    if (fx_absf(pos - f->pos) < 1e-5f) f->pos = pos;
    float p = f->pos, t;
    int mode;
    if (p < 0.5f - FLT_DEAD)      { mode = 0; t = (0.5f - FLT_DEAD - p) / (0.5f - FLT_DEAD); }
    else if (p > 0.5f + FLT_DEAD) { mode = 1; t = (p - 0.5f - FLT_DEAD) / (0.5f - FLT_DEAD); }
    else                          { mode = f->mode; t = 0.0f; }

    f->mode_target = mode;
    f->gate_target = t > 0.0f && mode == f->mode ? 1.0f : 0.0f;
    if (f->gate == 0.0f && mode != f->mode) {
        f->mode = mode;
        f->st[0][0] = f->st[0][1] = f->st[1][0] = f->st[1][1] = (fx_svf_state){0};
    }
    t = fx_clampf(t, 0.0f, 1.0f);

    /* 24 dB/oct, resonance growing with the sweep */
    float q2 = 1.3066f * (1.0f + 1.6f * t);
    if (f->mode == 0) {           /* high-pass: 20 Hz up to 6 kHz */
        float hz = 20.0f * fx_exp2f(8.23f * t);
        fx_svf_highpass(&f->s1, hz, 0.5412f, sr);
        fx_svf_highpass(&f->s2, hz, q2, sr);
    } else {                      /* low-pass: 20 kHz down to 60 Hz */
        float hz = 20000.0f * fx_exp2f(-8.38f * t);
        fx_svf_lowpass(&f->s1, hz, 0.5412f, sr);
        fx_svf_lowpass(&f->s2, hz, q2, sr);
    }
}

STAGE static void filter_process(fx_filter *f, float *l, float *r, int n, float sr)
{
    if (f->gate == 0.0f && f->gate_target == 0.0f) return; /* flat: untouched */
    float gate[FX_CHUNK];
    float g = f->gate, gt = f->gate_target, gstep = 1.0f / (0.015f * sr);
    for (int i = 0; i < n; i++) {
        if (g != gt) { g += g < gt ? gstep : -gstep; g = fx_clampf(g, 0.0f, 1.0f); }
        gate[i] = g;
    }
    f->gate = g;
    const fx_svf A = f->s1, B = f->s2;
    for (int c = 0; c < 2; c++) {
        float *x = c ? r : l;
        float a1 = f->st[c][0].ic1, a2 = f->st[c][0].ic2, b1 = f->st[c][1].ic1, b2 = f->st[c][1].ic2;
        for (int i = 0; i < n; i++) {
            float y = fx_svf_tick(A.a1, A.a2, A.a3, A.m0, A.m1, A.m2, &a1, &a2, x[i]);
            y = fx_svf_tick(B.a1, B.a2, B.a3, B.m0, B.m1, B.m2, &b1, &b2, y);
            x[i] += gate[i] * (y - x[i]);
        }
        f->st[c][0].ic1 = a1; f->st[c][0].ic2 = a2; f->st[c][1].ic1 = b1; f->st[c][1].ic2 = b2;
    }
}

/* =============================================================== 3: reverb ==== */

#define REV_DEAD 0.03f

/* at 24 kHz: mutually prime-ish, 24 to 55 ms */
static const int rev_len24[REV_LINES] = { 577, 683, 779, 887, 991, 1103, 1201, 1321 };
static const int rev_ap24[REV_AP] = { 71, 53, 191, 139 };

static void reverb_init(fx_reverb *v, float sr)
{
    __builtin_memset(v, 0, sizeof *v); /* in place: a zeroed literal would put 50 KB on the stack */
    v->sr = sr;
    float rs = 0.5f * sr, k = rs / 24000.0f;
    for (int j = 0; j < REV_LINES; j++) {
        int len = (int)((float)rev_len24[j] * k);
        v->len[j] = len < REV_MAX - 16 ? len : REV_MAX - 16;
    }
    for (int j = 0; j < REV_AP; j++) {
        int len = (int)((float)rev_ap24[j] * k);
        v->ap_len[j] = len < REV_AP_MAX ? len : REV_AP_MAX;
    }
    v->pre_len = (int)(0.014f * rs);
    if (v->pre_len >= REV_PRE) v->pre_len = REV_PRE - 1;
    /* two slow, unrelated LFOs; their sines, cosines and signs move the eight lines */
    osc_init(&v->lfo1_c, &v->lfo1_s, &v->lfo1_rc, &v->lfo1_rs, 0.31f, 0.0f, rs);
    osc_init(&v->lfo2_c, &v->lfo2_s, &v->lfo2_rc, &v->lfo2_rs, 0.47f, 1.3f, rs);
    v->mod_depth = 3.0f * k;
    /* the input band: keeps the dub low end out of the tail, and the top out of the half rate */
    v->in_hpc = sample_coef(1.0f / (2.0f * FX_PI * 140.0f), sr);
    v->in_lpc = sample_coef(1.0f / (2.0f * FX_PI * 7000.0f), sr);
}

STAGE static void reverb_control(fx_reverb *v, float pos)
{
    float sr = v->sr, rs = 0.5f * sr;
    float target = pos < REV_DEAD ? 0.0f : (pos - REV_DEAD) / (1.0f - REV_DEAD);
    v->amt += (target - v->amt) * chunk_coef(0.03f, sr);
    if (fx_absf(target - v->amt) < 1e-5f) v->amt = target;
    if (v->amt == 0.0f && target == 0.0f) {
        if (v->live) reverb_init(v, sr); /* down and silent: clear the tail, then sleep */
        return;
    }
    v->live = 1;
    float a = v->amt;
    v->wet = a * fx_sqrtf(a) * 0.75f;   /* level comes in gently */
    float rt60 = 1.0f + 4.0f * a;       /* and the room grows */
    for (int j = 0; j < REV_LINES; j++)
        v->gain[j] = fx_exp2f(-3.0f * 3.32192809f * (float)v->len[j] / (rs * rt60));
    /* damping: darker as it grows, like a big soft room */
    v->damp = sample_coef(1.0f / (2.0f * FX_PI * (7500.0f - 3500.0f * a)), rs);
    osc_norm(&v->lfo1_c, &v->lfo1_s);
    osc_norm(&v->lfo2_c, &v->lfo2_s);
}

/* one step of the tank at 24 kHz; returns the wet pair */
static inline void reverb_tick(fx_reverb *v, float in, float *wl, float *wr)
{
    /* pre-delay */
    v->pre[v->pre_w] = in;
    int pr = v->pre_w - v->pre_len;
    in = v->pre[pr < 0 ? pr + REV_PRE : pr];
    v->pre_w = v->pre_w + 1 < REV_PRE ? v->pre_w + 1 : 0;

    /* diffusion: four short allpasses smear the attack */
    for (int j = 0; j < REV_AP; j++) {
        float *b = v->ap[j];
        int p = v->ap_pos[j];
        float bd = b[p];
        float y = bd - 0.62f * in;
        b[p] = in + 0.62f * y;
        v->ap_pos[j] = p + 1 < v->ap_len[j] ? p + 1 : 0;
        in = y;
    }

    float t = v->lfo1_c * v->lfo1_rc - v->lfo1_s * v->lfo1_rs;
    v->lfo1_s = v->lfo1_s * v->lfo1_rc + v->lfo1_c * v->lfo1_rs; v->lfo1_c = t;
    t = v->lfo2_c * v->lfo2_rc - v->lfo2_s * v->lfo2_rs;
    v->lfo2_s = v->lfo2_s * v->lfo2_rc + v->lfo2_c * v->lfo2_rs; v->lfo2_c = t;
    const float md = v->mod_depth;
    const float m[4] = { md * v->lfo1_s, md * v->lfo1_c, md * v->lfo2_s, md * v->lfo2_c };
    const float damp = v->damp;

    float y[REV_LINES];
    for (int j = 0; j < REV_LINES; j++) {
        /* read len + 4 + mod behind the write point (mod within +-3: never the write slot) */
        float dly = (float)(v->len[j] + 4) + (j & 4 ? -m[j & 3] : m[j & 3]);
        int di = (int)dly;
        float fr = dly - (float)di;
        int a0 = v->wp[j] - di;
        if (a0 < 0) a0 += REV_MAX;
        int a1 = a0 - 1 < 0 ? REV_MAX - 1 : a0 - 1;   /* one older */
        const float *b = v->line[j];
        float s = b[a0] + fr * (b[a1] - b[a0]);
        float z = v->damp_z[j] + damp * (s - v->damp_z[j]);
        v->damp_z[j] = z;
        y[j] = z * v->gain[j];
    }

    /* 8x8 Hadamard, scaled to stay lossless */
    float h0 = y[0] + y[1], h1 = y[0] - y[1], h2 = y[2] + y[3], h3 = y[2] - y[3];
    float h4 = y[4] + y[5], h5 = y[4] - y[5], h6 = y[6] + y[7], h7 = y[6] - y[7];
    float g0 = h0 + h2, g2 = h0 - h2, g1 = h1 + h3, g3 = h1 - h3;
    float g4 = h4 + h6, g6 = h4 - h6, g5 = h5 + h7, g7 = h5 - h7;
    const float hs = 0.35355339f, ii = 0.5f * in;
    float o[REV_LINES] = { (g0 + g4) * hs + ii, (g1 + g5) * hs - ii, (g2 + g6) * hs + ii, (g3 + g7) * hs - ii,
                           (g0 - g4) * hs + ii, (g1 - g5) * hs - ii, (g2 - g6) * hs + ii, (g3 - g7) * hs - ii };
    for (int j = 0; j < REV_LINES; j++) {
        v->line[j][v->wp[j]] = o[j];
        v->wp[j] = v->wp[j] + 1 < REV_MAX ? v->wp[j] + 1 : 0;
    }

    *wl = (y[0] - y[2] + y[4] - y[6] + 0.5f * (y[1] + y[7])) * 0.6f;
    *wr = (y[1] - y[3] + y[5] - y[7] + 0.5f * (y[0] - y[6])) * 0.6f;
}

STAGE static void reverb_process(fx_reverb *v, float *l, float *r, int n)
{
    if (!v->live) return; /* asleep: dry passes untouched */
    const float wet = v->wet, hpc = v->in_hpc, lpc = v->in_lpc;
    for (int i = 0; i < n; i++) {
        /* the send: mono, band-limited, at 48 kHz */
        float in = 0.5f * (l[i] + r[i]);
        v->in_lp += lpc * (in - v->in_lp);
        v->in_hp += hpc * (v->in_lp - v->in_hp);
        in = v->in_lp - v->in_hp;
        /* every second frame, one tank step on the pair's average; the 48 kHz output glides
         * between the last two steps (one 24 kHz sample late) */
        float ol, orr;
        if (!v->half) {
            v->half_in = in;
            ol = v->wl1; orr = v->wr1;
        } else {
            v->wl0 = v->wl1; v->wr0 = v->wr1;
            reverb_tick(v, 0.5f * (v->half_in + in), &v->wl1, &v->wr1);
            ol = 0.5f * (v->wl0 + v->wl1); orr = 0.5f * (v->wr0 + v->wr1);
        }
        v->half ^= 1;
        l[i] += wet * ol;
        r[i] += wet * orr;
    }
}

/* ============================================================== 4: limiter ==== */

static void limiter_init(fx_limiter *m, float sr)
{
    *m = (fx_limiter){0};
    m->ceiling = fx_db2lin(-0.3f);
    m->rel = sample_coef(0.080f, sr);
    m->drive = m->drive_target = 1.0f;
}

STAGE static void limiter_control(fx_limiter *m, float pos)
{
    m->drive_target = fx_db2lin(12.0f * fx_clampf(pos, 0.0f, 1.0f));
    m->dstep = (m->drive_target - m->drive) * (1.0f / (float)FX_CHUNK);
    if (fx_absf(m->drive_target - m->drive) < 1e-6f) { m->drive = m->drive_target; m->dstep = 0.0f; }
    /* the box sum drifts by rounding; resum it once per chunk */
    float s = 0.0f;
    for (int k = 0; k < LIM_LA; k++) s += m->box[k];
    m->box_sum = s;
}

STAGE static void limiter_process(fx_limiter *m, float *l, float *r, int n)
{
    for (int i = 0; i < n; i++) {
        m->drive += m->dstep;
        float xl = l[i] * m->drive, xr = r[i] * m->drive;
        int p = m->pos;

        /* the reduction this frame needs to sit under the ceiling */
        float pk = fx_absf(xl) > fx_absf(xr) ? fx_absf(xl) : fx_absf(xr);
        float need = pk > m->ceiling ? 1.0f - m->ceiling / pk : 0.0f;

        /* the most reduction any frame in the look-ahead window needs (monotonic queue) */
        while (m->dq_n > 0 && m->red[m->dq[(m->dq_head + m->dq_n - 1) % LIM_LA]] <= need) m->dq_n--;
        m->red[p] = need;
        m->dq[(m->dq_head + m->dq_n) % LIM_LA] = p;
        m->dq_n++;
        float hold = m->red[m->dq[m->dq_head]];

        /* box filter of the held reduction over the window: smooth, and there in time */
        m->box_sum += hold - m->box[p];
        m->box[p] = hold;
        float box = m->box_sum * (1.0f / (float)LIM_LA);
        if (box < 1e-7f) box = 0.0f;

        if (box > m->gr) m->gr = box;
        else m->gr += m->rel * (box - m->gr);
        if (m->gr > m->meter) m->meter = m->gr;

        /* the delayed frame out, LIM_LA - 1 frames later */
        int q = (p + 1) % LIM_LA;
        float g = 1.0f - m->gr;
        float ol = m->dl[0][q], orr = m->dl[1][q];
        m->dl[0][p] = xl; m->dl[1][p] = xr;
        ol *= g; orr *= g;
        /* last guard against rounding: never past the ceiling */
        l[i] = fx_clampf(ol, -m->ceiling, m->ceiling);
        r[i] = fx_clampf(orr, -m->ceiling, m->ceiling);

        m->pos = q;
        /* expire the queue head when it falls out of the window (it is the slot written next) */
        if (m->dq_n > 0 && m->dq[m->dq_head] == q) { m->dq_head = (m->dq_head + 1) % LIM_LA; m->dq_n--; }
    }
}

/* ================================================================ engine ==== */

void fx_init(fx_engine *e, float sr)
{
    e->sr = sr;
    e->phase = 0;
    e->fader[FX_DUB] = 0.0f;
    e->fader[FX_FILTER] = 0.5f;
    e->fader[FX_REVERB] = 0.0f;
    e->fader[FX_LIMITER] = 0.0f;
    dub_init(&e->dub, sr);
    e->filter = (fx_filter){0};
    e->filter.pos = 0.5f;
    reverb_init(&e->reverb, sr);
    limiter_init(&e->limiter, sr);
}

void fx_set_fader(fx_engine *e, int slot, float pos)
{
    if (slot >= 0 && slot < FX_SLOTS) e->fader[slot] = fx_clampf(pos, 0.0f, 1.0f);
}

int fx_latency(const fx_engine *e)
{
    (void)e;
    return DUB_CENTER + LIM_LA - 1;
}

void fx_process(fx_engine *e, float *l, float *r, int n)
{
    while (n > 0) {
        if (e->phase == 0) {
            dub_control(&e->dub, e->fader[FX_DUB], e->sr);
            filter_control(&e->filter, e->fader[FX_FILTER], e->sr);
            reverb_control(&e->reverb, e->fader[FX_REVERB]);
            limiter_control(&e->limiter, e->fader[FX_LIMITER]);
        }
        int m = FX_CHUNK - e->phase;
        if (m > n) m = n;
        dub_process(&e->dub, l, r, m, e->sr);
        filter_process(&e->filter, l, r, m, e->sr);
        reverb_process(&e->reverb, l, r, m);
        limiter_process(&e->limiter, l, r, m);
        e->phase = (e->phase + m) % FX_CHUNK;
        l += m; r += m; n -= m;
    }
}
