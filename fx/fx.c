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
    __builtin_memset(d, 0, sizeof *d);
    d->dust_on = 0; /* disabled: eliminates CPU drain and annoying hiss/pops */
    d->gain = 1.0f;
    osc_init(&d->wow_c, &d->wow_s, &d->wow_rc, &d->wow_rs, 0.55f, 0.0f, sr);
    osc_init(&d->flt_c, &d->flt_s, &d->flt_rc, &d->flt_rs, 7.3f, 1.0f, sr);
    d->att = sample_coef(0.004f, sr);   /* 4ms punchy attack lets transients punch through */
    d->rel = sample_coef(0.080f, sr);
    d->knee = 5.0f;
    d->sc_c = sample_coef(1.0f / (2.0f * FX_PI * 150.0f), sr);
    d->dc_r = 1.0f - 2.0f * FX_PI * 4.0f / sr;
    d->hiss_c = sample_coef(1.0f / (2.0f * FX_PI * 5000.0f), sr);
}

STAGE static void dub_control(fx_dub *d, float pos, float sr)
{
    float target = pos < DUB_DEAD ? 0.0f : (pos - DUB_DEAD) / (1.0f - DUB_DEAD);
    d->amt += (target - d->amt) * chunk_coef(0.03f, sr);
    if (fx_absf(target - d->amt) < 1e-5f) d->amt = target;
    d->gate_target = pos < DUB_DEAD ? 0.0f : 1.0f;
    float a = d->amt;

    /* EQ: fat warm dub low end, de-mudded mid, punchy body */
    d->hp_c = 2.0f * FX_PI * (20.0f + 15.0f * a) / sr;
    fx_svf_lowshelf(&d->shelf, 95.0f, 0.75f, 11.5f * a, sr);
    fx_svf_bell(&d->cut, 300.0f, 0.9f, -4.5f * a, sr);
    fx_svf_bell(&d->pres, 1600.0f, 0.85f, 3.5f * a, sr);
    fx_svf_lowpass(&d->lp, 11000.0f * fx_exp2f(-1.40f * a), 0.65f, sr);

    /* punch: bus compressor with punchy attack, deeper knee and thick makeup */
    d->thr = -8.0f - 11.0f * a;
    d->slope = 1.0f / (1.0f + 2.5f * a) - 1.0f;
    d->makeup = -d->slope * (-d->thr) * 0.58f;

    /* tape saturation: gritty, warm and punchy */
    d->drive = 1.0f + 3.2f * a;
    d->sat_mix = fx_clampf(4.5f * a, 0.0f, 1.0f);
    d->bias = 0.16f * a;
    d->bias_out = fx_softclip(d->bias);
    d->inv_drive = 1.0f / d->drive;

    /* tape wow and flutter */
    d->wow_depth = 8.0f * a * sr / 48000.0f;
    d->flt_depth = 1.1f * a * sr / 48000.0f;

    /* subtle analog floor only at high drive, no crackles or ticks */
    d->dust_on = 0;
    d->hiss = 0.0f;
    d->tick_p = 0.0f;
    d->tick_th = 0;
    d->tick_amp = 0.0f;
    d->pop_p = 0.0f;
    d->pop_amp = 0.0f;

    d->trim = fx_db2lin(-9.0f * a);

    osc_norm(&d->wow_c, &d->wow_s);
    osc_norm(&d->flt_c, &d->flt_s);
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
                int i0 = (int)idx;
                float fr = idx - (float)i0;
                int k0 = i0 & (DUB_DL - 1), k1 = (i0 + 1) & (DUB_DL - 1);
                x[0][i] = b0[k0] + fr * (b0[k1] - b0[k0]);
                x[1][i] = b1[k0] + fr * (b1[k1] - b1[k0]);
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
        float hz = st[0].ic1, s1 = st[1].ic1, s2 = st[1].ic2;
        float c1 = st[2].ic1, c2 = st[2].ic2, p1 = st[3].ic1, p2 = st[3].ic2;
        const fx_svf S = d->shelf, C = d->cut, P = d->pres;
        const float hc = d->hp_c;
        float *v = x[c];
        for (int i = 0; i < n; i++) {
            hz += hc * (v[i] - hz);          /* rumble: a one-pole high-pass */
            float y = v[i] - hz;
            y = fx_svf_tick(S.a1, S.a2, S.a3, S.m0, S.m1, S.m2, &s1, &s2, y);
            y = fx_svf_tick(C.a1, C.a2, C.a3, C.m0, C.m1, C.m2, &c1, &c2, y);
            v[i] = fx_svf_tick(P.a1, P.a2, P.a3, P.m0, P.m1, P.m2, &p1, &p2, y);
        }
        st[0].ic1 = hz; st[1].ic1 = s1; st[1].ic2 = s2;
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
        float sl0 = d->sc_lp[0], sl1 = d->sc_lp[1];
        const float scc = d->sc_c;
        for (int i = 0; i < n; i++) {
            /* the detector hears the signal above 150 Hz: the bass stays big, the kick doesn't
             * pump the mix */
            sl0 += scc * (x[0][i] - sl0); sl1 += scc * (x[1][i] - sl1);
            float a0 = fx_absf(x[0][i] - sl0), a1 = fx_absf(x[1][i] - sl1);
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
        d->sc_lp[0] = sl0; d->sc_lp[1] = sl1;
        d->env = env; d->gain = g; d->rng = rng; d->cphase = ph; d->dg = dg; d->pop_env = pe; d->pop_lp = pl;
    }

    /* 4: tape saturation, the record's hiss and ticks, the dark top, the DC blocker, then the
     *    fade between dry and wet */
    float gates[FX_CHUNK];
    {
        float g = d->gate, gt = d->gate_target, gstep = 1.0f / (0.02f * sr);
        for (int i = 0; i < n; i++) {
            if (g != gt) { g += g < gt ? gstep : -gstep; g = fx_clampf(g, 0.0f, 1.0f); }
            gates[i] = g;
        }
        d->gate = g;
    }
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
            out[i] = dry[c][i] + gates[i] * (o - dry[c][i]);
        }
        d->st[c][4].ic1 = lp1; d->st[c][4].ic2 = lp2;
        d->hiss_lp[c] = hl; d->tick_env[c] = te; d->dc_x[c] = dx; d->dc_y[c] = dy;
        d->rng_c[c] = rng;
    }

    if (d->gate == 0.0f && d->gate_target == 0.0f) {
        /* fully off: start clean next time */
        for (int c = 0; c < 2; c++) {
            for (int k = 0; k < 5; k++) d->st[c][k] = (fx_svf_state){0};
            d->dc_x[c] = d->dc_y[c] = d->hiss_lp[c] = d->tick_env[c] = 0.0f;
        }
        d->env = d->pop_env = d->pop_lp = 0.0f;
        d->sc_lp[0] = d->sc_lp[1] = 0.0f;
        d->gain = 1.0f;
    }
}

/* =============================================================== 2: filter ==== */

#define FLT_DEAD 0.10f /* half-width of the flat zone around the centre: easy to find by feel */

STAGE static void filter_control(fx_filter *f, float pos, float sr)
{
    f->pos += (pos - f->pos) * chunk_coef(0.02f, sr);
    if (fx_absf(pos - f->pos) < 1e-5f) f->pos = pos;
    float p = f->pos, t;
    int mode;
    /* down: low-pass; up: high-pass */
    if (p < 0.5f - FLT_DEAD)      { mode = 1; t = (0.5f - FLT_DEAD - p) / (0.5f - FLT_DEAD); }
    else if (p > 0.5f + FLT_DEAD) { mode = 0; t = (p - 0.5f - FLT_DEAD) / (0.5f - FLT_DEAD); }
    else                          { mode = f->mode; t = 0.0f; }

    f->mode_target = mode;
    if (f->gate == 0.0f && mode != f->mode) {
        f->mode = mode;
        f->st[0][0] = f->st[0][1] = f->st[1][0] = f->st[1][1] = (fx_svf_state){0};
    }
    f->gate_target = t > 0.0f && mode == f->mode ? 1.0f : 0.0f;
    t = fx_clampf(t, 0.0f, 1.0f);

    /* 24 dB/oct, juicy analog resonance and harmonic character */
    float q2 = 1.3066f * (1.0f + 3.6f * t);
    if (f->mode == 0) {           /* high-pass: 20 Hz up to 6 kHz */
        float hz = 20.0f * fx_exp2f(8.23f * t);
        fx_svf_highpass(&f->s1, hz, 0.5412f, sr);
        fx_svf_highpass(&f->s2, hz, q2, sr);
    } else {                      /* low-pass: 11 kHz down to 60 Hz */
        float hz = 11000.0f * fx_exp2f(-7.52f * t);
        fx_svf_lowpass(&f->s1, hz, 0.5412f, sr);
        fx_svf_lowpass(&f->s2, hz, q2, sr);
    }
}

STAGE static void filter_process(fx_filter *f, float *l, float *r, int n, float sr)
{
    if (f->gate == 0.0f && f->gate_target == 0.0f) return; /* flat: untouched, 0 CPU */
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

/* ========================================================== 3: bit reducer ==== */

#define CRUSH_DEAD 0.03f

static void crush_init(fx_bitcrush *c, float sr)
{
    (void)sr;
    __builtin_memset(c, 0, sizeof *c);
    c->bits = 16.0f;
    c->q_step = 1.0f / 32768.0f;
    c->inv_q_step = 32768.0f;
    c->rate_ratio = 1.0f;
}

STAGE static void crush_control(fx_bitcrush *c, float pos, float sr)
{
    (void)sr;
    float target = pos < CRUSH_DEAD ? 0.0f : (pos - CRUSH_DEAD) / (1.0f - CRUSH_DEAD);
    c->amt += (target - c->amt) * chunk_coef(0.03f, sr);
    if (fx_absf(target - c->amt) < 1e-5f) c->amt = target;
    c->gate_target = pos < CRUSH_DEAD ? 0.0f : 1.0f;

    float a = c->amt;
    /* Bit depth reduces smoothly from 16-bit at 0 -> 12-bit -> 8-bit -> down to 3.5-bit */
    c->bits = 16.0f - 12.5f * a;
    float levels = fx_exp2f(c->bits);
    if (levels < 8.0f) levels = 8.0f;
    c->q_step = 2.0f / levels;
    c->inv_q_step = 1.0f / c->q_step;

    /* Downsample factor: 1.0 (no sample hold) down to ~0.08 (~1.9 kHz rate) */
    c->rate_ratio = 1.0f / (1.0f + 11.0f * a * a);
    c->wet = fx_clampf(1.25f * a, 0.0f, 1.0f);
}

STAGE static void crush_process(fx_bitcrush *c, float *l, float *r, int n, float sr)
{
    if (c->gate == 0.0f && c->gate_target == 0.0f) return; /* bypassed: 0 CPU */

    float g = c->gate, gt = c->gate_target, gstep = 1.0f / (0.015f * sr);
    const float wet = c->wet;
    const float q = c->q_step, inv_q = c->inv_q_step;
    const float rr = c->rate_ratio;
    float ph = c->phase;
    float hl = c->hold_l, hr = c->hold_r;

    for (int i = 0; i < n; i++) {
        if (g != gt) { g += g < gt ? gstep : -gstep; g = fx_clampf(g, 0.0f, 1.0f); }
        ph += rr;
        if (ph >= 1.0f) {
            ph -= 1.0f;
            if (ph >= 1.0f) ph = 0.0f;
            /* Quantize to bit depth */
            float ql = fx_clampf(l[i], -1.0f, 1.0f);
            float qr = fx_clampf(r[i], -1.0f, 1.0f);
            hl = (float)((int)(ql * inv_q + 0.5f * (ql >= 0 ? 1.0f : -1.0f))) * q;
            hr = (float)((int)(qr * inv_q + 0.5f * (qr >= 0 ? 1.0f : -1.0f))) * q;
        }
        float out_l = l[i] + wet * (hl - l[i]);
        float out_r = r[i] + wet * (hr - r[i]);
        l[i] += g * (out_l - l[i]);
        r[i] += g * (out_r - r[i]);
    }
    c->gate = g;
    c->phase = ph;
    c->hold_l = hl;
    c->hold_r = hr;
}

static inline void reverb_init(fx_reverb *v, float sr) { crush_init(v, sr); }
static inline void reverb_control(fx_reverb *v, float pos) { crush_control(v, pos, 24000.0f); }
static inline void reverb_process(fx_reverb *v, float *l, float *r, int n) { crush_process(v, l, r, n, 24000.0f); }

/* ============================================================== 4: limiter ==== */

static void limiter_init(fx_limiter *m, float sr)
{
    __builtin_memset(m, 0, sizeof *m);
    m->ceiling = fx_db2lin(-1.0f);   /* stays <= -0.3 dBFS after polyphase half-band upsampler */
    m->rel = sample_coef(0.080f, sr);
    m->drive = m->drive_target = 1.0f;
    m->active = 0;
}

#define LIM_DEAD 0.02f

STAGE static void limiter_control(fx_limiter *m, float pos)
{
    m->amt = pos;
    /* Slider 4 scales drive into the ceiling: 0 dB at bottom, +12 dB at top */
    m->drive_target = fx_db2lin(12.0f * fx_clampf(pos, 0.0f, 1.0f));
    m->active = pos > LIM_DEAD;
    m->dstep = (m->drive_target - m->drive) * (1.0f / (float)FX_CHUNK);
    if (fx_absf(m->drive_target - m->drive) < 1e-6f) { m->drive = m->drive_target; m->dstep = 0.0f; }
    /* the box sum drifts by rounding; resum it once per chunk */
    float s = 0.0f;
    for (int k = 0; k < LIM_LA; k++) s += m->box[k];
    m->box_sum = s;
}

STAGE static void limiter_process(fx_limiter *m, float *l, float *r, int n)
{
    if (!m->active && m->gr < 1e-4f && m->held < 1e-4f) {
        /* Slider 4 is at 0: Limiter is completely bypassed with zero CPU waste */
        m->drive = 1.0f;
        m->meter = 0.0f;
        int p = m->pos;
        for (int i = 0; i < n; i++) {
            int q = p + 1 < LIM_LA ? p + 1 : 0;
            float ol = m->dl[0][q], orr = m->dl[1][q];
            m->dl[0][p] = l[i]; m->dl[1][p] = r[i];
            l[i] = ol; r[i] = orr;
            p = q;
        }
        m->pos = p;
        return;
    }

    const float ceil = m->ceiling, rel = m->rel, inv_la = 1.0f / (float)LIM_LA;
    float drive = m->drive, cur = m->cur, held = m->held, sum = m->box_sum, gr = m->gr;
    float meter = m->meter;
    int p = m->pos, sn = m->sn, si = m->si;
    for (int i = 0; i < n; i++) {
        drive += m->dstep;
        float xl = l[i] * drive, xr = r[i] * drive;

        /* the reduction this frame needs to sit under the ceiling */
        float al = fx_absf(xl), ar = fx_absf(xr);
        float pk = al > ar ? al : ar;
        float need = pk > ceil ? 1.0f - ceil / pk : 0.0f;

        /* peak hold over the look-ahead window and up to one segment more */
        if (need > cur) cur = need;
        float hold = cur > held ? cur : held;
        if (++sn == LIM_SEG) {
            sn = 0;
            m->seg[si] = cur;
            si = si + 1 < LIM_NSEG ? si + 1 : 0;
            cur = 0.0f;
            float h = 0.0f;
            for (int k = 0; k < LIM_NSEG; k++) if (k != si && m->seg[k] > h) h = m->seg[k];
            held = h;
        }

        /* box filter of the held reduction over the window */
        sum += hold - m->box[p];
        m->box[p] = hold;
        float box = sum * inv_la;
        if (box < 1e-7f) box = 0.0f;

        if (box > gr) gr = box;
        else gr += rel * (box - gr);
        if (gr > meter) meter = gr;

        /* the delayed frame out, LIM_LA - 1 frames later */
        int q = p + 1 < LIM_LA ? p + 1 : 0;
        float g = 1.0f - gr;
        float ol = m->dl[0][q] * g, orr = m->dl[1][q] * g;
        m->dl[0][p] = xl; m->dl[1][p] = xr;
        l[i] = fx_clampf(ol, -ceil, ceil);
        r[i] = fx_clampf(orr, -ceil, ceil);
        p = q;
    }
    m->drive = drive; m->cur = cur; m->held = held; m->box_sum = sum; m->gr = gr;
    m->meter = meter; m->pos = p; m->sn = sn; m->si = si;
}

/* ================================================================ engine ==== */

/* Polyphase IIR half-band */
static const float hb_c[HB_C] = {
    0.0212186694f, 0.0816374655f, 0.1726362110f, 0.2829113996f, 0.4014960495f,
    0.5199743642f, 0.6334772202f, 0.7406996488f, 0.8434655794f, 0.9463427912f,
};

static void hb_run(float *a, float *b, int n, float *y, float *x0, float *x1)
{
    const float c0 = hb_c[0], c1 = hb_c[1], c2 = hb_c[2], c3 = hb_c[3], c4 = hb_c[4];
    const float c5 = hb_c[5], c6 = hb_c[6], c7 = hb_c[7], c8 = hb_c[8], c9 = hb_c[9];
    float y0 = y[0], y1 = y[1], y2 = y[2], y3 = y[3], y4 = y[4];
    float y5 = y[5], y6 = y[6], y7 = y[7], y8 = y[8], y9 = y[9];
    float xa = *x0, xb = *x1;
    for (int k = 0; k < n; k++) {
        float s = a[k], t;
        t = (s - y0) * c0 + xa; xa = s;  s = t;
        t = (s - y2) * c2 + y0; y0 = s;  s = t;
        t = (s - y4) * c4 + y2; y2 = s;  s = t;
        t = (s - y6) * c6 + y4; y4 = s;  s = t;
        t = (s - y8) * c8 + y6; y6 = s;  y8 = t;
        a[k] = t;
        s = b[k];
        t = (s - y1) * c1 + xb; xb = s;  s = t;
        t = (s - y3) * c3 + y1; y1 = s;  s = t;
        t = (s - y5) * c5 + y3; y3 = s;  s = t;
        t = (s - y7) * c7 + y5; y5 = s;  s = t;
        t = (s - y9) * c9 + y7; y7 = s;  y9 = t;
        b[k] = t;
    }
    y[0] = y0; y[1] = y1; y[2] = y2; y[3] = y3; y[4] = y4;
    y[5] = y5; y[6] = y6; y[7] = y7; y[8] = y8; y[9] = y9;
    *x0 = xa; *x1 = xb;
}

void fx_init(fx_engine *e, float sr)
{
    __builtin_memset(e, 0, sizeof *e);
    e->sr = sr;
    e->cr = 0.5f * sr;
    e->fader[FX_DUB] = 0.0f;
    e->fader[FX_FILTER] = 0.5f;
    e->fader[FX_CRUSH] = 0.0f;
    e->fader[FX_LIMITER] = 0.0f;
    e->warm_c = sample_coef(1.0f / (2.0f * FX_PI * 3500.0f), sr);
    dub_init(&e->dub, e->cr);
    e->filter.pos = 0.5f;
    crush_init(&e->crush, e->cr);
    limiter_init(&e->limiter, e->cr);
}

void fx_set_fader(fx_engine *e, int slot, float pos)
{
    if (slot >= 0 && slot < FX_SLOTS) e->fader[slot] = fx_clampf(pos, 0.0f, 1.0f);
}

int fx_latency(const fx_engine *e)
{
    (void)e;
    return 11 + 2 * (DUB_CENTER + LIM_LA - 1);
}

/* the effects, at the core rate, in chunks that line up with the parameter updates */
static void core_process(fx_engine *e, float *l, float *r, int n)
{
    while (n > 0) {
        if (e->phase == 0) {
            dub_control(&e->dub, e->fader[FX_DUB], e->cr);
            filter_control(&e->filter, e->fader[FX_FILTER], e->cr);
            crush_control(&e->crush, e->fader[FX_CRUSH], e->cr);
            limiter_control(&e->limiter, e->fader[FX_LIMITER]);
        }
        int m = FX_CHUNK - e->phase;
        if (m > n) m = n;
        dub_process(&e->dub, l, r, m, e->cr);
        filter_process(&e->filter, l, r, m, e->cr);
        crush_process(&e->crush, l, r, m, e->cr);
        limiter_process(&e->limiter, l, r, m);
        e->phase = (e->phase + m) % FX_CHUNK;
        l += m; r += m; n -= m;
    }
}

/* up to 2 * FX_CHUNK outside frames */
STAGE static void fx_piece(fx_engine *e, float *l, float *r, int n)
{
    float pa[2][FX_CHUNK + 1], pb[2][FX_CHUNK + 1];   /* each pair: newer, older */
    int nc = 0;

    /* the warm stage's treble dip, and the frames gathered in pairs */
    {
        const float wc = e->warm_c;
        float w0 = e->warm_lp[0], w1 = e->warm_lp[1];
        int odd = e->dodd;
        for (int i = 0; i < n; i++) {
            float x0 = l[i], x1 = r[i];
            /* a slight treble dip: a quarter of the way to a 3.5 kHz low-pass */
            w0 += wc * (x0 - w0); w1 += wc * (x1 - w1);
            x0 += 0.25f * (w0 - x0); x1 += 0.25f * (w1 - x1);
            if (odd) {
                pa[0][nc] = x0; pb[0][nc] = e->dfirst[0];
                pa[1][nc] = x1; pb[1][nc] = e->dfirst[1];
                nc++;
            } else {
                e->dfirst[0] = x0; e->dfirst[1] = x1;
            }
            odd ^= 1;
        }
        e->warm_lp[0] = w0; e->warm_lp[1] = w1; e->dodd = odd;
    }

    /* 48 -> 24 kHz */
    float cl[FX_CHUNK + 1], crr[FX_CHUNK + 1];
    for (int c = 0; c < 2; c++) {
        hb_run(pa[c], pb[c], nc, e->dy[c], &e->dx0[c], &e->dx1[c]);
        float *o = c ? crr : cl;
        /* and the warm stage's gentle tape saturation: half of a soft clip driven a little. Here,
         * band-limited at 24 kHz, its harmonics stay clean: the ones that would fold back come
         * from the top octave, which sits far below full scale in real music */
        for (int k = 0; k < nc; k++) {
            float x = 0.5f * (pa[c][k] + pb[c][k]);
            o[k] = x + 0.5f * (fx_softclip(1.4f * x) * 0.71428571f - x);
        }
    }

    core_process(e, cl, crr, nc);

    /* 24 -> 48 kHz: each core frame makes a pair, the first half out with the odd outside frame
     * that brought it, the second with the next */
    for (int c = 0; c < 2; c++) {
        const float *in = c ? crr : cl;
        for (int k = 0; k < nc; k++) pa[c][k] = pb[c][k] = in[k];
        hb_run(pa[c], pb[c], nc, e->uy[c], &e->ux0[c], &e->ux1[c]);
    }
    {
        int odd = e->uodd, k = 0;
        for (int i = 0; i < n; i++) {
            float o0, o1;
            if (odd) {
                o0 = pa[0][k]; o1 = pa[1][k];
                e->usecond[0] = pb[0][k]; e->usecond[1] = pb[1][k];
                k++;
            } else {
                o0 = e->usecond[0]; o1 = e->usecond[1];
            }
            odd ^= 1;
            l[i] = o0;
            r[i] = o1;
        }
        e->uodd = odd;
    }
}

void fx_process(fx_engine *e, float *l, float *r, int n)
{
    while (n > 0) {
        int m = n < 2 * FX_CHUNK ? n : 2 * FX_CHUNK;
        fx_piece(e, l, r, m);
        l += m; r += m; n -= m;
    }
}
