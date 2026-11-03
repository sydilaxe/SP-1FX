/* SP-1 multi-effects engine. See fx.h for the faders. */
#include "fx.h"

#ifdef FX_PROFILE   /* tools/bench_m4.py: keep the stages apart so their cost can be read */
#define STAGE __attribute__((noinline))
#else
#define STAGE
#endif

/* quadrature oscillator: a rotation per frame, renormalised each chunk */
static void osc_rate(float *rc, float *rs, float hz, float sr)
{
    /* cos and sin of a small angle from the tan half-angle: t = tan(w/2) */
    float t = fx_tanf(FX_PI * hz / sr), t2 = t * t;
    *rc = (1.0f - t2) / (1.0f + t2);
    *rs = 2.0f * t / (1.0f + t2);
}

static void osc_init(float *c, float *s, float *rc, float *rs, float hz, float phase, float sr)
{
    osc_rate(rc, rs, hz, sr);
    float tp = fx_tanf(0.5f * phase), tp2 = tp * tp; /* phase in 0..pi */
    *c = (1.0f - tp2) / (1.0f + tp2);
    *s = 2.0f * tp / (1.0f + tp2);
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

/* the same, applied every `every` frames */
static float step_coef(float seconds, float sr, float every)
{
    return 1.0f - fx_exp2f(-1.44269504f * every / (seconds * sr));
}

static float sample_coef(float seconds, float sr) { return step_coef(seconds, sr, 1.0f); }

/* move toward a target, snapping when close so a resting control settles exactly */
static inline float glide(float v, float target, float k)
{
    v += (target - v) * k;
    return fx_absf(target - v) < 1e-5f ? target : v;
}

static inline float fx_roundf(float x) { return (float)(int)(x + (x >= 0.0f ? 0.5f : -0.5f)); }

/* ================================================================== 1: dub ==== */

#define DUB_DEAD 0.03f  /* fader below this: the slot is bypassed */

static void dub_init(fx_dub *d, float sr)
{
    __builtin_memset(d, 0, sizeof *d);
    d->gain = 1.0f;
    d->speed_mod = 0.5f;
    d->set_a = -1.0f;
    /* the wow and flutter oscillators step once per chunk; the tap ramps between */
    osc_init(&d->wow_c, &d->wow_s, &d->wow_rc, &d->wow_rs, 0.55f * FX_CHUNK, 0.0f, sr);
    osc_init(&d->flt_c, &d->flt_s, &d->flt_rc, &d->flt_rs, 7.3f * FX_CHUNK, 1.0f, sr);
    d->att = sample_coef(0.004f, sr);   /* 4 ms: transients punch through */
    d->rel = sample_coef(0.080f, sr);
    d->knee = 5.0f;
    d->sc_c = sample_coef(1.0f / (2.0f * FX_PI * 150.0f), sr);
    d->dc_r = 1.0f - 2.0f * FX_PI * 4.0f / sr;
}

STAGE static void dub_control(fx_dub *d, float pos, float sr)
{
    float target = pos < DUB_DEAD ? 0.0f : (pos - DUB_DEAD) / (1.0f - DUB_DEAD);
    d->amt = glide(d->amt, target, chunk_coef(0.03f, sr));
    d->gate_target = pos < DUB_DEAD ? 0.0f : 1.0f;
    if (d->gate == 0.0f && d->gate_target == 0.0f) return;   /* off: nothing to set up */
    float a = d->amt;
    if (a == d->set_a && d->speed_mod == d->set_speed) goto wobble;   /* parked: nothing new */
    d->set_a = a;
    d->set_speed = d->speed_mod;

    /* page 2: tape wobble speed, 0.4x to 1.6x (wow 0.22 to 0.88 Hz) */
    osc_rate(&d->wow_rc, &d->wow_rs, 0.55f * (0.4f + 1.2f * d->speed_mod) * FX_CHUNK, sr);

    /* EQ: fat low end, de-mudded mid, a dark top */
    d->hp_c = 2.0f * FX_PI * (20.0f + 15.0f * a) / sr;
    fx_bq_lowshelf(&d->shelf, 95.0f, 0.75f, 6.5f * a, sr);
    fx_bq_bell(&d->cut, 300.0f, 0.9f, -2.5f * a, sr);
    fx_bq_lowpass(&d->lp, 11500.0f * fx_exp2f(-1.30f * a), 0.65f, sr);  /* 11.5 kHz down to 4.7 kHz */

    /* glue: a gentle bus compressor (2:1 at full), for the vibe rather than loudness */
    d->thr = -10.0f - 6.0f * a;
    d->slope = 1.0f / (1.0f + 1.0f * a) - 1.0f;
    d->makeup = -d->slope * (-d->thr) * 0.45f;

    /* tape saturation */
    d->drive = 1.0f + 0.75f * a;
    d->sat_mix = fx_clampf(2.0f * a, 0.0f, 0.85f);
    d->bias = 0.05f * a;
    d->bias_out = fx_softclip(d->bias);
    d->inv_drive = 1.0f / d->drive;

    /* wow and flutter, in frames */
    d->wow_depth = 8.0f * a * sr / 48000.0f;
    d->flt_depth = 1.1f * a * sr / 48000.0f;

    d->trim = fx_db2lin(-7.2f * a);   /* level-matched: full sounds as loud as off */

wobble:;

    /* the wobble for this chunk: from where the last one ended to the oscillators' next step */
    float t = d->wow_c * d->wow_rc - d->wow_s * d->wow_rs;
    d->wow_s = d->wow_s * d->wow_rc + d->wow_c * d->wow_rs; d->wow_c = t;
    t = d->flt_c * d->flt_rc - d->flt_s * d->flt_rs;
    d->flt_s = d->flt_s * d->flt_rc + d->flt_c * d->flt_rs; d->flt_c = t;
    osc_norm(&d->wow_c, &d->wow_s);
    osc_norm(&d->flt_c, &d->flt_s);
    float end = d->wow_depth * d->wow_s + d->flt_depth * d->flt_s;
    d->off_step = (end - d->off) * (1.0f / (float)FX_CHUNK);
}

STAGE static void dub_process(fx_dub *d, float *l, float *r, int n, float sr)
{
    float dry[2][FX_CHUNK], x[2][FX_CHUNK], gain[FX_CHUNK], gates[FX_CHUNK];
    int wpos = d->wpos;
    float *b0 = d->dl[0], *b1 = d->dl[1];

    if (d->gate == 0.0f && d->gate_target == 0.0f) {
        /* off: only the delay, so the latency does not change */
        for (int i = 0; i < n; i++) {
            b0[wpos] = l[i];
            b1[wpos] = r[i];
            int tap = (wpos - DUB_CENTER) & (DUB_DL - 1);
            l[i] = b0[tap];
            r[i] = b1[tap];
            wpos = (wpos + 1) & (DUB_DL - 1);
        }
        d->wpos = wpos;
        return;
    }

    /* fading in or out: the dry signal is needed for the crossfade */
    const int fading = d->gate != d->gate_target || d->gate != 1.0f;

    /* 1: the tape: write the delay line, read the wobbling wet tap */
    {
        const int w0 = wpos;
        float off = d->off;
        const float os = d->off_step;
        for (int i = 0; i < n; i++) {
            b0[wpos] = l[i];
            b1[wpos] = r[i];
            float idx = (float)(wpos + DUB_DL - DUB_CENTER) - off;
            off += os;
            int i0 = (int)idx;
            float fr = idx - (float)i0;
            int k0 = i0 & (DUB_DL - 1), k1 = (i0 + 1) & (DUB_DL - 1);
            float p0 = b0[k0], q0 = b1[k0];
            x[0][i] = p0 + fr * (b0[k1] - p0);
            x[1][i] = q0 + fr * (b1[k1] - q0);
            wpos = (wpos + 1) & (DUB_DL - 1);
        }
        d->wpos = wpos;
        d->off = off;
        if (fading) {
            /* the centre taps: still in the line (it is longer than a chunk plus the centre) */
            for (int i = 0; i < n; i++) {
                int tap = (w0 + i - DUB_CENTER) & (DUB_DL - 1);
                dry[0][i] = b0[tap];
                dry[1][i] = b1[tap];
            }
        }
    }

    /* 2: the EQ, one channel at a time so the filter states stay in registers */
    for (int c = 0; c < 2; c++) {
        fx_bq_state *st = d->st[c];
        float hz = d->hp_z[c], s1 = st[0].s1, s2 = st[0].s2;
        float c1 = st[1].s1, c2 = st[1].s2;
        const fx_bq S = d->shelf, C = d->cut;
        const float hc = d->hp_c;
        float *v = x[c];
        for (int i = 0; i < n; i++) {
            hz += hc * (v[i] - hz);          /* rumble: a one-pole high-pass */
            float y = v[i] - hz;
            y = fx_bq_tick(S.b0, S.b1, S.b2, S.a1, S.a2, &s1, &s2, y);
            v[i] = fx_bq_tick(C.b0, C.b1, C.b2, C.a1, C.a2, &c1, &c2, y);
        }
        d->hp_z[c] = hz; st[0].s1 = s1; st[0].s2 = s2;
        st[1].s1 = c1; st[1].s2 = c2;
    }

    /* 3: the compressor: a peak envelope every frame, the gain every fourth frame, glided
     *    between; and the fade between dry and wet */
    {
        float env = d->env, g = d->gain;
        const float att = d->att, rel = d->rel, thr = d->thr, slope = d->slope, knee = d->knee;
        const float mk = d->makeup;
        float dg = d->dg;
        int ph = d->cphase;
        float sl0 = d->sc_lp[0], sl1 = d->sc_lp[1];
        const float scc = d->sc_c;
        float gg = d->gate, gt = d->gate_target, gstep = 1.0f / (0.02f * sr);
        for (int i = 0; i < n; i++) {
            /* the detector hears the signal above 150 Hz: the bass stays big, the kick doesn't
             * pump the mix */
            sl0 += scc * (x[0][i] - sl0); sl1 += scc * (x[1][i] - sl1);
            uint32_t a0 = fx_abs_bits(x[0][i] - sl0), a1 = fx_abs_bits(x[1][i] - sl1);
            float pk = fx_from_bits(a0 > a1 ? a0 : a1);
            env += (pk - env) * (pk > env ? att : rel);
            if (ph == 0) {
                float over = fx_lin2db(env + 1e-9f) - thr, gr;
                if (2.0f * over < -knee) gr = 0.0f;
                else if (2.0f * over > knee) gr = slope * over;
                else { float k = over + 0.5f * knee; gr = slope * k * k * (0.5f / knee); }
                dg = (fx_db2lin(gr + mk) - g) * 0.125f;
            }
            ph = (ph + 1) & 7;
            g += dg;
            gain[i] = g;
        }
        if (fading) {
            for (int i = 0; i < n; i++) {
                if (gg != gt) { gg += gg < gt ? gstep : -gstep; gg = fx_clampf(gg, 0.0f, 1.0f); }
                gates[i] = gg;
            }
        }
        d->sc_lp[0] = sl0; d->sc_lp[1] = sl1;
        d->env = env; d->gain = g; d->cphase = ph; d->dg = dg; d->gate = gg;
    }

    /* 4: tape saturation, the dark top, the DC blocker, then the fade between dry and wet */
    for (int c = 0; c < 2; c++) {
        float *out = c ? r : l;
        float *v = x[c];
        /* the saturation, y + mix * ((softclip(y * drive + bias) - softclip(bias)) / drive - y),
         * folded: below the clip point that is y + k0 - k3 * u^3 with u = y * drive + bias */
        const float drive = d->drive, idr = d->inv_drive, mix = d->sat_mix, bias = d->bias;
        const float bout = d->bias_out, trim = d->trim, dcr = d->dc_r;
        const float k0 = mix * idr * (bias - bout), k3 = mix * idr * 0.14814815f;
        const fx_bq LP = d->lp;
        float lp1 = d->st[c][2].s1, lp2 = d->st[c][2].s2;
        float dx = d->dc_x[c], dy = d->dc_y[c];
        for (int i = 0; i < n; i++) {
            float y = v[i] * gain[i];
            float u = y * drive + bias;
            if (fx_abs_bits(u) <= 0x3fc00000u) y += k0 - k3 * u * u * u;
            else y += mix * (((u > 0.0f ? 1.0f : -1.0f) - bout) * idr - y);
            y = fx_bq_tick(LP.b0, LP.b1, LP.b2, LP.a1, LP.a2, &lp1, &lp2, y);
            float o = y - dx + dcr * dy;
            dx = y; dy = o;
            out[i] = o * trim;
        }
        if (fading) {
            const float *dr = dry[c];
            for (int i = 0; i < n; i++) out[i] = dr[i] + gates[i] * (out[i] - dr[i]);
        }
        d->st[c][2].s1 = lp1; d->st[c][2].s2 = lp2;
        d->dc_x[c] = dx; d->dc_y[c] = dy;
    }

    if (d->gate == 0.0f && d->gate_target == 0.0f) {
        /* fully off: start clean next time */
        for (int c = 0; c < 2; c++) {
            for (int k = 0; k < 3; k++) d->st[c][k] = (fx_bq_state){0};
            d->hp_z[c] = 0.0f;
            d->dc_x[c] = d->dc_y[c] = 0.0f;
        }
        d->env = 0.0f;
        d->sc_lp[0] = d->sc_lp[1] = 0.0f;
        d->gain = 1.0f;
        d->dg = 0.0f;
        d->off = d->off_step = 0.0f;
    }
}

/* =============================================================== 2: filter ==== */

#define FLT_DEAD 0.10f /* half-width of the flat zone around the centre: easy to find by feel */

STAGE static void filter_control(fx_filter *f, float pos, float sr)
{
    f->pos = glide(f->pos, pos, chunk_coef(0.02f, sr));
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
    if (f->gate == 0.0f && f->gate_target == 0.0f) return;
    t = fx_clampf(t, 0.0f, 1.0f);
    if (t == f->set_t && f->res == f->set_res && f->mode == f->set_mode) return;  /* parked */
    f->set_t = t;
    f->set_res = f->res;
    f->set_mode = f->mode;

    /* 24 dB/oct; page 2 sets how much resonance builds toward the ends */
    float res_scale = 0.3f + 1.4f * f->res;
    float q2 = 1.3066f * (1.0f + 3.6f * t * res_scale);
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
    if (f->gate == 0.0f && f->gate_target == 0.0f) return; /* flat: untouched, 0 CPU */
    float gate[FX_CHUNK], dry[2][FX_CHUNK];
    float g = f->gate, gt = f->gate_target, gstep = 1.0f / (0.015f * sr);
    int fading = g != gt || g != 1.0f;
    if (fading) {
        for (int i = 0; i < n; i++) {
            if (g != gt) { g += g < gt ? gstep : -gstep; g = fx_clampf(g, 0.0f, 1.0f); }
            gate[i] = g;
            dry[0][i] = l[i];
            dry[1][i] = r[i];
        }
        f->gate = g;
    }
    const fx_svf A = f->s1, B = f->s2;
    for (int c = 0; c < 2; c++) {
        float *x = c ? r : l;
        float a1 = f->st[c][0].ic1, a2 = f->st[c][0].ic2, b1 = f->st[c][1].ic1, b2 = f->st[c][1].ic2;
        if (f->mode) {
            for (int i = 0; i < n; i++) {
                float y = fx_svf_lp(A.a1, A.a2, A.a3, &a1, &a2, x[i]);
                x[i] = fx_svf_lp(B.a1, B.a2, B.a3, &b1, &b2, y);
            }
        } else {
            const float ka = -A.m1, kb = -B.m1;
            for (int i = 0; i < n; i++) {
                float y = fx_svf_hp(A.a1, A.a2, A.a3, ka, &a1, &a2, x[i]);
                x[i] = fx_svf_hp(B.a1, B.a2, B.a3, kb, &b1, &b2, y);
            }
        }
        f->st[c][0].ic1 = a1; f->st[c][0].ic2 = a2; f->st[c][1].ic1 = b1; f->st[c][1].ic2 = b2;
        if (fading) {
            const float *d = dry[c];
            for (int i = 0; i < n; i++) x[i] = d[i] + gate[i] * (x[i] - d[i]);
        }
    }
}

/* ======================================================= 3: bit reducer ==== */

#define CRUSH_DEAD 0.03f
#define CRUSH_FLOOR 1e-4f   /* -80 dBFS: below this the quantizer stops following the level */

static void crush_init(fx_bitcrush *c, float sr)
{
    __builtin_memset(c, 0, sizeof *c);
    c->q_step = 2.0f / 65536.0f;
    c->inv_q_step = 1.0f / c->q_step;
    c->rate_ratio = 1.0f;
    c->rate_mod = 0.5f;
    c->env_rel = sample_coef(0.012f, sr);   /* lets go fast: quiet sound after a hit keeps its steps */
}

STAGE static void crush_control(fx_bitcrush *c, float pos, float sr)
{
    float target = pos < CRUSH_DEAD ? 0.0f : (pos - CRUSH_DEAD) / (1.0f - CRUSH_DEAD);
    c->amt = glide(c->amt, target, chunk_coef(0.03f, sr));
    c->gate_target = pos < CRUSH_DEAD ? 0.0f : 1.0f;
    float a = c->amt;

    /* 16 bits at the bottom of the fader down to 4 at the top. The steps are relative to the
     * signal's level (see crush_process): the crunch is the same loud or quiet */
    float levels = fx_exp2f(16.0f - 12.0f * a);
    c->q_step = 2.0f / levels;
    c->inv_q_step = levels * 0.5f;

    /* sample rate: 48 kHz down to 2 kHz at the top (page 2 at its middle); page 2 scales the
     * range from 0.2x to 1.8x */
    float rate_scale = 0.2f + 1.6f * c->rate_mod;
    c->rate_ratio = 1.0f / (1.0f + 23.0f * a * a * rate_scale);
    c->wet = fx_clampf(1.25f * a, 0.0f, 1.0f);
}

STAGE static void crush_process(fx_bitcrush *c, float *l, float *r, int n, float sr)
{
    if (c->gate == 0.0f && c->gate_target == 0.0f) return; /* off: 0 CPU */

    float g = c->gate, gt = c->gate_target, gstep = 1.0f / (0.015f * sr);
    const float wet = c->wet, q = c->q_step, inv_q = c->inv_q_step, rr = c->rate_ratio;
    const float rel = c->env_rel;
    float ph = c->phase, env = c->env;
    float hl = c->hold_l, hr = c->hold_r;

    float mix[FX_CHUNK];
    const int fading = g != gt || g != 1.0f;
    if (fading) {
        for (int i = 0; i < n; i++) {
            if (g != gt) { g += g < gt ? gstep : -gstep; g = fx_clampf(g, 0.0f, 1.0f); }
            mix[i] = g * wet;
        }
    }
    for (int i = 0; i < n; i++) {
        uint32_t ul = fx_abs_bits(l[i]), ur = fx_abs_bits(r[i]);
        float pk = fx_from_bits(ul > ur ? ul : ur);
        env = pk > env ? pk : env + rel * (pk - env);   /* instant attack: |x| <= env always */
        ph += rr;
        if (ph >= 1.0f) {
            ph -= 1.0f;
            /* quantize the signal relative to its level: x / env lies in -1..1, so even a quiet
             * passage keeps all its steps instead of rounding to silence */
            float e = env > CRUSH_FLOOR ? env : CRUSH_FLOOR;
            float k = inv_q / e, s = q * e;
            hl = fx_roundf(l[i] * k) * s;
            hr = fx_roundf(r[i] * k) * s;
        }
        float m = fading ? mix[i] : wet;
        l[i] += m * (hl - l[i]);
        r[i] += m * (hr - r[i]);
    }
    c->gate = g;
    c->phase = ph;
    c->env = env;
    c->hold_l = hl;
    c->hold_r = hr;
    if (g == 0.0f && gt == 0.0f) { c->env = c->hold_l = c->hold_r = c->phase = 0.0f; }
}

/* ======================================================= 4: LA-2A leveler ==== */

/* The LA-2A's T4 cell: a lamp lit by the sidechain shines on a photoresistor. The lamp is
 * quick (about 10 ms to light up); the photoresistor lets go in two stages: about half the
 * reduction recovers in 60 ms, the rest over seconds, and the longer and harder it was lit the
 * longer that tail. Modelled as a fast part and a slow "memory" part that charges up from it;
 * the reduction is the larger of the two. Fader: Peak Reduction, with the makeup (Gain) set
 * along with it so the level stays roughly put while the sound gets denser. */

#define LA_RATE 8     /* the cell's state moves every eighth frame (6 kHz) */

static void limiter_init(fx_limiter *m, float sr)
{
    __builtin_memset(m, 0, sizeof *m);
    m->mode = 0.5f;
    m->g = 1.0f;
    m->det_att = sample_coef(0.0005f, sr);
    m->det_rel = sample_coef(0.020f, sr);
    m->c_att = step_coef(0.010f, sr, LA_RATE);           /* lamp: ~10 ms */
    m->c_rel_f = step_coef(0.087f, sr, LA_RATE);         /* half back in 60 ms */
    m->c_chg_s = step_coef(1.0f, sr, LA_RATE);           /* memory charges over ~1 s */
    m->c_rel_s = step_coef(2.0f, sr, LA_RATE);
    m->ceiling = fx_db2lin(-1.0f);
    m->s_rel = step_coef(0.050f, sr, LIM_SEG);
    m->sg = 1.0f;
}

#define LA_DEAD 0.02f

STAGE static void limiter_control(fx_limiter *m, float sr)
{
    float target = m->pos < LA_DEAD ? 0.0f : (m->pos - LA_DEAD) / (1.0f - LA_DEAD);
    m->amt = glide(m->amt, target, chunk_coef(0.05f, sr));
    float a = m->amt;
    m->active = a > 0.0f || fx_absf(m->g - 1.0f) > 1e-6f;

    /* compress (page 2 down): 3:1 with a wide knee; limit (page 2 up): 10:1, a narrower knee
     * and a quicker memory */
    float ratio = 3.0f + 7.0f * m->mode;
    m->slope = 1.0f - 1.0f / ratio;
    m->knee = 14.0f - 8.0f * m->mode;
    m->c_rel_s = step_coef(1.5f - 0.7f * m->mode, sr, LA_RATE);

    /* peak reduction: the sidechain's gain, as a threshold coming down from -6 dBFS */
    m->thr = -6.0f - 20.0f * a;
    m->makeup_db = 0.55f * m->slope * 20.0f * a;
    m->fade = fx_clampf(8.0f * a, 0.0f, 1.0f);      /* all of it from an eighth of the way up */
    m->tube = 0.5f * a;                              /* the output tube's warmth */

}

STAGE static void la2a_process(fx_limiter *m, float *l, float *r, int n)
{
    float meter = m->meter;
    const float da = m->det_att, dr = m->det_rel, thr = m->thr, slope = m->slope, knee = m->knee;
    const float ca = m->c_att, crf = m->c_rel_f, ccs = m->c_chg_s, crs = m->c_rel_s;
    const float mk = m->makeup_db, fade = m->fade, tube = m->tube;
    const float kt = tube * 0.14814815f;               /* the tube: y - tube * 0.148 y^3 */
    const uint32_t clip = 0x3fc00000u;                 /* 1.5f: above it the cubic is flat at 1 */
    float det = m->det, gf = m->gr_f, gs = m->gr_s, g = m->g, dg = m->dg;
    int ph = m->cphase;
    for (int i = 0; i < n; i++) {
        uint32_t ul = fx_abs_bits(l[i]), ur = fx_abs_bits(r[i]);
        float pk = fx_from_bits(ul > ur ? ul : ur);
        det += (pk - det) * (pk > det ? da : dr);
        if (ph == 0) {
            float over = fx_lin2db(det + 1e-9f) - thr, gt;
            if (2.0f * over < -knee) gt = 0.0f;
            else if (2.0f * over > knee) gt = slope * over;
            else { float k = over + 0.5f * knee; gt = slope * k * k * (0.5f / knee); }
            gf += (gt - gf) * (gt > gf ? ca : crf);
            float ts = 0.5f * gf;
            gs += (ts - gs) * (ts > gs ? ccs : crs);
            float gr = gf > gs ? gf : gs;
            float red = fade * gr;
            float u = 1.0f - fx_db2lin(-red);
            if (u > meter) meter = u;
            dg = (fx_db2lin(fade * mk - red) - g) * (1.0f / LA_RATE);
        }
        ph = (ph + 1) & (LA_RATE - 1);
        g += dg;
        float yl = l[i] * g, yr = r[i] * g;
        if (fx_abs_bits(yl) <= clip) yl -= kt * yl * yl * yl;
        else yl += tube * ((yl > 0.0f ? 1.0f : -1.0f) - yl);
        if (fx_abs_bits(yr) <= clip) yr -= kt * yr * yr * yr;
        else yr += tube * ((yr > 0.0f ? 1.0f : -1.0f) - yr);
        l[i] = yl; r[i] = yr;
    }
    m->det = det; m->gr_f = gf; m->gr_s = gs; m->g = g; m->dg = dg; m->cphase = ph;
    if (m->amt == 0.0f && fx_absf(g - 1.0f) < 1e-6f) {
        /* back to off: start clean next time */
        m->g = 1.0f; m->dg = 0.0f; m->det = m->gr_f = m->gr_s = 0.0f; m->cphase = 0;
        m->active = 0;
    }
    m->meter = meter;
}

/* the safety ceiling: always on, acts only on overs; at unity it passes the input exactly.
 * The input waits LIM_LA frames (6 segments of 8) in the delay line. Once a segment, the gain
 * the outgoing segment ends on is set from the peaks still to come: the full reduction for the
 * segments next in line, ramping down to a fifth of it for the newest. Between those points the
 * gain moves in a straight line, so every frame is at or under the ceiling and the gain has no
 * corners; afterwards it lets go over about 50 ms. */
STAGE static void ceiling_process(fx_limiter *m, float *l, float *r, int n)
{
    const float ceil = m->ceiling;
    const uint32_t ceil_bits = fx_abs_bits(ceil);
    uint32_t acc = fx_abs_bits(m->acc_pk);
    float g = m->sg, gs = m->sg_step, meter = m->meter;
    int p = m->pos_dl, sn = m->sn;
    /* the delay line is a whole number of segments, so a segment never wraps inside it */
    while (n > 0) {
        int run = LIM_SEG - sn;
        if (run > n) run = n;
        float *d0 = &m->dl[0][p], *d1 = &m->dl[1][p];
        for (int i = 0; i < run; i++) {
            float xl = l[i], xr = r[i];
            uint32_t ul = fx_abs_bits(xl), ur = fx_abs_bits(xr);
            uint32_t u = ul > ur ? ul : ur;
            acc = u > acc ? u : acc;
            float ol = d0[i] * g, orr = d1[i] * g;
            d0[i] = xl; d1[i] = xr;
            if (fx_abs_bits(ol) > ceil_bits) ol = ol > 0.0f ? ceil : -ceil;
            if (fx_abs_bits(orr) > ceil_bits) orr = orr > 0.0f ? ceil : -ceil;
            l[i] = ol; r[i] = orr;
            g += gs;
        }
        l += run; r += run; n -= run;
        p += run; if (p == LIM_LA) p = 0;
        sn += run;
        if (sn == LIM_SEG) {
            /* a segment in: its need joins the ring, newest at [si] */
            sn = 0;
            int si = m->si;
            m->seg[si] = acc > ceil_bits ? 1.0f - ceil / fx_from_bits(acc) : 0.0f;
            acc = 0;
            /* ages: 0 newest .. LIM_NSEG-1 the one now going out */
            static const float w[LIM_NSEG] = { 0.2f, 0.4f, 0.6f, 0.8f, 1.0f, 1.0f };
            float red = 0.0f;
            for (int a = 0; a < LIM_NSEG; a++) {
                int k = si - a; if (k < 0) k += LIM_NSEG;
                float v = m->seg[k] * w[a];
                if (v > red) red = v;
            }
            m->si = si + 1 < LIM_NSEG ? si + 1 : 0;
            float prev = m->sgr;
            if (red < prev) { red = prev + m->s_rel * (red - prev); if (red < 1e-6f) red = 0.0f; }
            m->sgr = red;
            if (red > meter) meter = red;
            float ge = 1.0f - red;
            g = 1.0f - prev;                       /* land exactly on the last point */
            gs = (ge - g) * (1.0f / (float)LIM_SEG);
            if (ge == 1.0f && g == 1.0f) gs = 0.0f;
        }
    }
    m->acc_pk = fx_from_bits(acc); m->sg = g; m->sg_step = gs; m->meter = meter;
    m->pos_dl = p; m->sn = sn;
}

static void limiter_process(fx_limiter *m, float *l, float *r, int n)
{
    if (m->active) la2a_process(m, l, r, n);
    ceiling_process(m, l, r, n);
}

/* ================================================================ engine ==== */

void fx_init(fx_engine *e, float sr)
{
    __builtin_memset(e, 0, sizeof *e);
    e->sr = sr;
    e->fader[FX_DUB] = 0.0f;
    e->fader[FX_FILTER] = 0.5f;
    e->fader[FX_CRUSH] = 0.0f;
    e->fader[FX_LIMITER] = 0.0f;
    for (int k = 0; k < FX_SLOTS; k++) e->param2[k] = 0.5f;
    dub_init(&e->dub, sr);
    e->filter.pos = 0.5f;
    e->filter.res = 0.5f;
    e->filter.set_t = -1.0f;
    crush_init(&e->crush, sr);
    limiter_init(&e->limiter, sr);
}

void fx_set_fader(fx_engine *e, int slot, float pos)
{
    if (slot >= 0 && slot < FX_SLOTS) e->fader[slot] = fx_clampf(pos, 0.0f, 1.0f);
}

void fx_set_param2(fx_engine *e, int slot, float pos)
{
    if (slot < 0 || slot >= FX_SLOTS) return;
    pos = fx_clampf(pos, 0.0f, 1.0f);
    e->param2[slot] = pos;
    if (slot == FX_DUB) e->dub.speed_mod = pos;
    else if (slot == FX_FILTER) e->filter.res = pos;
    else if (slot == FX_CRUSH) e->crush.rate_mod = pos;
    else e->limiter.mode = pos;
}

int fx_latency(const fx_engine *e)
{
    (void)e;
    return DUB_CENTER + LIM_LA;
}

float fx_gain_reduction_db(const fx_engine *e)
{
    return -fx_lin2db(1.0f - e->limiter.meter + 1e-6f);
}

void fx_process(fx_engine *e, float *l, float *r, int n)
{
    const float sr = e->sr;
    while (n > 0) {
        if (e->phase == 0) {
            dub_control(&e->dub, e->fader[FX_DUB], sr);
            filter_control(&e->filter, e->fader[FX_FILTER], sr);
            crush_control(&e->crush, e->fader[FX_CRUSH], sr);
            e->limiter.pos = e->fader[FX_LIMITER];
            limiter_control(&e->limiter, sr);
        }
        int m = FX_CHUNK - e->phase;
        if (m > n) m = n;
        dub_process(&e->dub, l, r, m, sr);
        filter_process(&e->filter, l, r, m, sr);
        crush_process(&e->crush, l, r, m, sr);
        limiter_process(&e->limiter, l, r, m);
        e->phase = (e->phase + m) % FX_CHUNK;
        l += m; r += m; n -= m;
    }
}
