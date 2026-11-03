/* Small float math for the SP-1 effects: no libm, no doubles, so the same code runs on the
 * nRF52840 (Cortex-M4F, single-precision FPU) and on the host. Accurate to well under what is
 * audible in the places it is used (filter coefficients, compressor gain). */
#ifndef SP1_FX_MATH_H
#define SP1_FX_MATH_H

#include <stdint.h>

#define FX_PI 3.14159265f

static inline float fx_clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static inline float fx_absf(float x) { return x < 0.0f ? -x : x; }
static inline float fx_sqrtf(float x) { return __builtin_sqrtf(x); } /* one VSQRT on the M4F */

/* 2^x, |x| < 126; about 1e-6 relative error */
static inline float fx_exp2f(float x)
{
    float fl = (float)(int)x;
    if (fl > x) fl -= 1.0f;
    float f = x - fl; /* 0..1 */
    float p = 1.0f + f * (0.69314718f + f * (0.24022650f + f * (0.05550411f
            + f * (0.00961813f + f * (0.00133336f + f * 0.00015404f)))));
    union { float f; uint32_t u; } v = { p };
    v.u += (uint32_t)((int)fl) << 23;
    return v.f;
}

/* log2(x), x > 0; about 1e-5 absolute error */
static inline float fx_log2f(float x)
{
    union { float f; uint32_t u; } v = { x };
    float e = (float)((int)((v.u >> 23) & 0xff) - 127);
    v.u = (v.u & 0x007fffff) | 0x3f800000; /* mantissa 1..2 */
    float m = v.f;
    float t = (m - 1.0f) / (m + 1.0f), t2 = t * t;
    return e + 2.88539008f * t * (1.0f + t2 * (0.33333333f + t2 * (0.2f + t2 * 0.14285714f)));
}

static inline float fx_db2lin(float db) { return fx_exp2f(db * 0.16609640f); }
static inline float fx_lin2db(float x) { return 6.02059991f * fx_log2f(x); }

/* tan(x) for 0 <= x < pi/2, from sin and cos series */
static inline float fx_tanf(float x)
{
    float x2 = x * x;
    float s = x * (1.0f + x2 * (-1.6666667e-1f + x2 * (8.3333333e-3f + x2 * (-1.9841270e-4f
            + x2 * (2.7557319e-6f + x2 * -2.5052108e-8f)))));
    float c = 1.0f + x2 * (-0.5f + x2 * (4.1666667e-2f + x2 * (-1.3888889e-3f
            + x2 * (2.4801587e-5f + x2 * (-2.7557319e-7f + x2 * 2.0876757e-9f)))));
    return s / c;
}

/* soft clip shaped like tanh, exactly +-1 from |x| >= 3 */
static inline float fx_tanh(float x)
{
    x = fx_clampf(x, -3.0f, 3.0f);
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* soft clip without a division: a cubic, smooth into exactly +-1 at |x| = 1.5 */
static inline float fx_softclip(float x)
{
    x = fx_clampf(x, -1.5f, 1.5f);
    return x - 0.14814815f * x * x * x;
}

/* |x| as its bit pattern: for magnitudes, integer order is float order, and an integer compare
 * on the Cortex-M4 avoids the FPU-flags round trip (VCMP + VMRS) */
static inline uint32_t fx_abs_bits(float x)
{
    union { float f; uint32_t u; } v = { x };
    return v.u & 0x7fffffffu;
}
static inline float fx_from_bits(uint32_t u)
{
    union { uint32_t u; float f; } v = { u };
    return v.f;
}

/* xorshift32: white noise in [-1, 1) and [0, 1) */
static inline uint32_t fx_rng(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return *s = x;
}
static inline float fx_noise(uint32_t *s) { return (float)(int32_t)fx_rng(s) * 4.6566129e-10f; }
static inline float fx_rand01(uint32_t *s) { return (float)(fx_rng(s) >> 8) * 5.9604645e-8f; }

/* Trapezoidal state-variable filter (Simper). out = m0*in + m1*band + m2*low */
typedef struct { float a1, a2, a3, m0, m1, m2; } fx_svf;
typedef struct { float ic1, ic2; } fx_svf_state;

/* the same on plain locals, so a loop keeps the state in registers */
static inline float fx_svf_tick(float a1, float a2, float a3, float m0, float m1, float m2,
                                float *ic1, float *ic2, float v0)
{
    float v3 = v0 - *ic2;
    float v1 = a1 * *ic1 + a2 * v3;
    float v2 = *ic2 + a2 * *ic1 + a3 * v3;
    *ic1 = 2.0f * v1 - *ic1;
    *ic2 = 2.0f * v2 - *ic2;
    return m0 * v0 + m1 * v1 + m2 * v2;
}

/* the core of a tick: updates the state, returns v1 (band) and v2 (low) */
static inline void fx_svf_step(float a1, float a2, float a3, float *ic1, float *ic2, float v0,
                               float *v1, float *v2)
{
    float v3 = v0 - *ic2;
    *v1 = a1 * *ic1 + a2 * v3;
    *v2 = *ic2 + a2 * *ic1 + a3 * v3;
    *ic1 = 2.0f * *v1 - *ic1;
    *ic2 = 2.0f * *v2 - *ic2;
}

/* low-pass: the low output alone */
static inline float fx_svf_lp(float a1, float a2, float a3, float *ic1, float *ic2, float v0)
{
    float v1, v2;
    fx_svf_step(a1, a2, a3, ic1, ic2, v0, &v1, &v2);
    return v2;
}

/* high-pass (m0 = 1, m1 = -k, m2 = -1) */
static inline float fx_svf_hp(float a1, float a2, float a3, float k, float *ic1, float *ic2,
                              float v0)
{
    float v1, v2;
    fx_svf_step(a1, a2, a3, ic1, ic2, v0, &v1, &v2);
    return v0 - k * v1 - v2;
}

static inline float fx_svf_run(const fx_svf *c, fx_svf_state *s, float v0)
{
    float v3 = v0 - s->ic2;
    float v1 = c->a1 * s->ic1 + c->a2 * v3;
    float v2 = s->ic2 + c->a2 * s->ic1 + c->a3 * v3;
    s->ic1 = 2.0f * v1 - s->ic1;
    s->ic2 = 2.0f * v2 - s->ic2;
    return c->m0 * v0 + c->m1 * v1 + c->m2 * v2;
}

static inline void fx_svf_core(fx_svf *c, float g, float k)
{
    c->a1 = 1.0f / (1.0f + g * (g + k));
    c->a2 = g * c->a1;
    c->a3 = g * c->a2;
}

static inline float fx_svf_g(float hz, float sr)
{
    return fx_tanf(FX_PI * fx_clampf(hz, 5.0f, 0.46f * sr) / sr);
}

static inline void fx_svf_lowpass(fx_svf *c, float hz, float q, float sr)
{
    fx_svf_core(c, fx_svf_g(hz, sr), 1.0f / q);
    c->m0 = 0.0f; c->m1 = 0.0f; c->m2 = 1.0f;
}

static inline void fx_svf_highpass(fx_svf *c, float hz, float q, float sr)
{
    float k = 1.0f / q;
    fx_svf_core(c, fx_svf_g(hz, sr), k);
    c->m0 = 1.0f; c->m1 = -k; c->m2 = -1.0f;
}

static inline void fx_svf_bell(fx_svf *c, float hz, float q, float db, float sr)
{
    float A = fx_exp2f(db * 0.08304820f); /* 10^(db/40) */
    float k = 1.0f / (q * A);
    fx_svf_core(c, fx_svf_g(hz, sr), k);
    c->m0 = 1.0f; c->m1 = k * (A * A - 1.0f); c->m2 = 0.0f;
}

static inline void fx_svf_lowshelf(fx_svf *c, float hz, float q, float db, float sr)
{
    float A = fx_exp2f(db * 0.08304820f);
    float k = 1.0f / q;
    fx_svf_core(c, fx_svf_g(hz, sr) / fx_sqrtf(A), k);
    c->m0 = 1.0f; c->m1 = k * (A - 1.0f); c->m2 = A * A - 1.0f;
}

/* The same responses as direct-form biquads (transposed form II): 5 multiplies and 4 adds a
 * frame instead of the state-variable filter's 9 and 7. For fixed or slowly moving settings; the
 * state-variable filter stays where a filter sweeps fast. Built from the same analog prototype,
 * H(s) = m0 + m1 s / D + m2 / D, D = s^2 + k s + 1, prewarped with g = tan(pi f / sr). */
typedef struct { float b0, b1, b2, a1, a2; } fx_bq;
typedef struct { float s1, s2; } fx_bq_state;

static inline void fx_bq_from(fx_bq *c, float g, float k, float m0, float m1, float m2)
{
    float B2 = m0, B1 = m0 * k + m1, B0 = m0 + m2, g2 = g * g;
    float inv = 1.0f / (1.0f + k * g + g2);
    c->b0 = (B2 + B1 * g + B0 * g2) * inv;
    c->b1 = 2.0f * (B0 * g2 - B2) * inv;
    c->b2 = (B2 - B1 * g + B0 * g2) * inv;
    c->a1 = 2.0f * (g2 - 1.0f) * inv;
    c->a2 = (1.0f - k * g + g2) * inv;
}

static inline void fx_bq_lowpass(fx_bq *c, float hz, float q, float sr)
{
    fx_bq_from(c, fx_svf_g(hz, sr), 1.0f / q, 0.0f, 0.0f, 1.0f);
}

static inline void fx_bq_bell(fx_bq *c, float hz, float q, float db, float sr)
{
    float A = fx_exp2f(db * 0.08304820f);
    float k = 1.0f / (q * A);
    fx_bq_from(c, fx_svf_g(hz, sr), k, 1.0f, k * (A * A - 1.0f), 0.0f);
}

static inline void fx_bq_lowshelf(fx_bq *c, float hz, float q, float db, float sr)
{
    float A = fx_exp2f(db * 0.08304820f);
    float k = 1.0f / q;
    fx_bq_from(c, fx_svf_g(hz, sr) / fx_sqrtf(A), k, 1.0f, k * (A - 1.0f), A * A - 1.0f);
}

static inline float fx_bq_tick(float b0, float b1, float b2, float a1, float a2,
                               float *s1, float *s2, float x)
{
    float y = b0 * x + *s1;
    *s1 = b1 * x - a1 * y + *s2;
    *s2 = b2 * x - a2 * y;
    return y;
}

#endif
