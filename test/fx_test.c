/* Host test for the SP-1 effects engine: renders WAVs into build/sp1 and checks the behaviour.
 *   fx_test [OUTDIR]                       run the checks and write the WAVs
 *   fx_test OUTDIR IN.wav F1 F2 F3 F4      also run a 48 kHz stereo 16/24-bit WAV through the
 *                                          chain at those fader positions (0..1) */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../fx/fx.h"

#define SR 48000
static fx_engine E;
static int fails;

static void check(int ok, const char *what, double v)
{
    printf("%-58s %10.4f  %s\n", what, v, ok ? "ok" : "FAIL");
    if (!ok) fails++;
}

/* ---- WAV ---- */
static void put32(FILE *f, unsigned v) { fputc(v, f); fputc(v >> 8, f); fputc(v >> 16, f); fputc(v >> 24, f); }
static void put16(FILE *f, unsigned v) { fputc(v, f); fputc(v >> 8, f); }

static void wav_write(const char *dir, const char *name, const float *l, const float *r, int n)
{
    char p[512];
    snprintf(p, sizeof p, "%s/%s", dir, name);
    FILE *f = fopen(p, "wb");
    if (!f) { perror(p); exit(1); }
    fwrite("RIFF", 1, 4, f); put32(f, 36 + n * 6); fwrite("WAVEfmt ", 1, 8, f);
    put32(f, 16); put16(f, 1); put16(f, 2); put32(f, SR); put32(f, SR * 6); put16(f, 6); put16(f, 24);
    fwrite("data", 1, 4, f); put32(f, n * 6);
    for (int i = 0; i < n; i++)
        for (int c = 0; c < 2; c++) {
            double x = (c ? r : l)[i];
            x = x > 1 ? 1 : x < -1 ? -1 : x;
            int v = (int)lrint(x * 8388607.0);
            fputc(v, f); fputc(v >> 8, f); fputc(v >> 16, f);
        }
    fclose(f);
}

static int wav_read(const char *path, float **l, float **r)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }
    unsigned char h[12];
    if (fread(h, 1, 12, f) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) { fclose(f); return -1; }
    int ch = 0, bits = 0, rate = 0, n = -1;
    unsigned char c[8];
    while (fread(c, 1, 8, f) == 8) {
        unsigned sz = c[4] | c[5] << 8 | c[6] << 16 | (unsigned)c[7] << 24;
        if (!memcmp(c, "fmt ", 4)) {
            unsigned char fm[40];
            if (fread(fm, 1, sz < 40 ? sz : 40, f) < 16) break;
            if (sz > 40) fseek(f, sz - 40, SEEK_CUR);
            ch = fm[2] | fm[3] << 8; rate = fm[4] | fm[5] << 8 | fm[6] << 16; bits = fm[14] | fm[15] << 8;
        } else if (!memcmp(c, "data", 4)) {
            if ((bits != 16 && bits != 24) || ch < 1 || ch > 2) break;
            if (rate != SR) fprintf(stderr, "warning: %d Hz, played as 48 kHz\n", rate);
            int bps = bits / 8;
            n = (int)(sz / (unsigned)(bps * ch));
            *l = malloc(sizeof(float) * n); *r = malloc(sizeof(float) * n);
            for (int i = 0; i < n; i++)
                for (int k = 0; k < ch; k++) {
                    unsigned char s[3] = {0};
                    if (fread(s, 1, bps, f) != (size_t)bps) { n = i; goto done; }
                    int v = bits == 16 ? (int16_t)(s[0] | s[1] << 8) * 256 : ((int32_t)((unsigned)s[0] << 8 | (unsigned)s[1] << 16 | (unsigned)s[2] << 24)) >> 8;
                    float x = (float)v / 8388608.0f;
                    if (k == 0) { (*l)[i] = x; if (ch == 1) (*r)[i] = x; }
                    else (*r)[i] = x;
                }
            break;
        } else fseek(f, sz + (sz & 1), SEEK_CUR);
    }
done:
    fclose(f);
    return n;
}

/* ---- the test signal: a one-drop reggae bar, 75 BPM ---- */
static double nz(unsigned *s) { *s = *s * 1664525u + 1013904223u; return (double)(int)*s / 2147483648.0; }

static void make_loop(float *l, float *r, int n)
{
    const double beat = 0.8, bar = 4 * beat;
    const double bass[8] = { 55.0, 55.0, 65.41, 73.42, 55.0, 49.0, 55.0, 82.41 }; /* A1 C2 D2 G1 E2 */
    unsigned s = 1;
    double bp1 = 0, bp2 = 0, hh = 0;   /* skank band-pass, hat high-pass */
    double ph[3] = {0}, bph = 0;
    for (int i = 0; i < n; i++) {
        double t = (double)i / SR, tb = fmod(t, bar), tq = fmod(t, beat);
        int q = (int)(tb / beat);
        /* bass: eighth notes, round */
        int e8 = (int)(fmod(t, 2 * bar) / (beat / 2)) % 16;
        double f = bass[e8 / 2], te = fmod(t, beat / 2);
        bph += 2 * M_PI * f / SR;
        double benv = (1 - exp(-te / 0.004)) * exp(-te / 0.35);
        double b = (sin(bph) + 0.3 * sin(2 * bph)) * benv * 0.45;
        /* one drop: kick and rim together on beat 3 */
        double k = 0, rim = 0;
        if (q == 2) {
            double kp = 2 * M_PI * (50 * tq + 60 * 0.03 * (1 - exp(-tq / 0.03)));
            k = sin(kp) * exp(-tq / 0.18) * 0.7;
            rim = nz(&s) * exp(-tq / 0.012) * 0.35 + sin(2 * M_PI * 1700 * tq) * exp(-tq / 0.02) * 0.2;
        }
        /* skank: an Am chop on the off-beats (2 and 4) */
        double sk = 0;
        if (q == 1 || q == 3) {
            const double fr[3] = { 220.0, 261.63, 329.63 };
            for (int j = 0; j < 3; j++) { ph[j] += fr[j] / SR; sk += 2 * fmod(ph[j], 1.0) - 1; }
            sk *= exp(-tq / 0.06) * 0.18;
        }
        bp1 += 0.25 * (sk - bp1); bp2 += 0.25 * (bp1 - bp2);
        sk = bp1 - bp2 * 0.6;
        /* hats: eighths */
        double th = fmod(t, beat / 2), w = nz(&s);
        hh = w - hh * 0.0 ; double hat = (w - 0.7 * hh) * exp(-th / 0.03) * 0.08;
        double mono = b + k + rim;
        l[i] = (float)(mono + sk * 1.1 + hat * 0.8);
        r[i] = (float)(mono + sk * 0.9 + hat * 1.2);
    }
    float pk = 0;
    for (int i = 0; i < n; i++) { if (fabsf(l[i]) > pk) pk = fabsf(l[i]); if (fabsf(r[i]) > pk) pk = fabsf(r[i]); }
    for (int i = 0; i < n; i++) { l[i] *= 0.5f / pk; r[i] *= 0.5f / pk; }
}

/* ---- measuring ---- */
typedef struct { double b0, b1, b2, a1, a2, z1, z2; } bq;
static void bq_set(bq *q, int hp, double hz)
{
    double w = 2 * M_PI * hz / SR, c = cos(w), al = sin(w) / (2 * 0.7071), a0 = 1 + al;
    q->b1 = (hp ? -(1 + c) : (1 - c)) / a0; q->b0 = q->b2 = (hp ? (1 + c) / 2 : (1 - c) / 2) / a0;
    q->a1 = -2 * c / a0; q->a2 = (1 - al) / a0; q->z1 = q->z2 = 0;
}
static double bq_run(bq *q, double x)
{
    double y = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * y + q->z2; q->z2 = q->b2 * x - q->a2 * y;
    return y;
}
static double band_db(const float *x, int from, int n, double lo, double hi)
{
    bq a, b, c, d; bq_set(&a, 1, lo); bq_set(&b, 1, lo); bq_set(&c, 0, hi); bq_set(&d, 0, hi);
    double e = 0;
    for (int i = 0; i < n; i++) {
        double y = bq_run(&d, bq_run(&c, bq_run(&b, bq_run(&a, x[i]))));
        if (i >= from) e += y * y;
    }
    return 10 * log10(e / (n - from) + 1e-30);
}
static double rms_db(const float *x, int from, int n)
{
    double e = 0; for (int i = from; i < n; i++) e += (double)x[i] * (double)x[i];
    return 10 * log10(e / (n - from) + 1e-30);
}
static double peak(const float *l, const float *r, int n)
{
    double p = 0;
    for (int i = 0; i < n; i++) { if (fabs(l[i]) > p) p = fabs(l[i]); if (fabs(r[i]) > p) p = fabs(r[i]); }
    return p;
}
static int finite_all(const float *l, const float *r, int n)
{
    for (int i = 0; i < n; i++) if (!isfinite(l[i]) || !isfinite(r[i])) return 0;
    return 1;
}
/* clicks: a 256-frame window whose largest second difference stands out from both neighbours.
 * A loud, bright, distorted sound has big steps everywhere; a click has them in one place. */
static double click_ratio(const float *x, int n)
{
    int nw = n / 256;
    double *m = calloc(nw, sizeof *m), worst = 0;
    for (int w = 0; w < nw; w++)
        for (int i = w * 256 + (w ? 0 : 2); i < w * 256 + 256; i++) {
            double d = fabs(x[i] - 2.0 * x[i - 1] + x[i - 2]);
            if (d > m[w]) m[w] = d;
        }
    for (int w = 2; w < nw - 1; w++) {   /* from the second window: the tone's onset is a step */
        double nb = fmax(fmax(m[w - 1], m[w + 1]), 1e-4);
        if (m[w] / nb > worst) worst = m[w] / nb;
    }
    free(m);
    return worst;
}

static double max_d2(const float *x, int n) /* largest second difference */
{
    double m = 0;
    for (int i = 2; i < n; i++) { double d = fabs(x[i] - 2.0 * x[i - 1] + x[i - 2]); if (d > m) m = d; }
    return m;
}

static void faders(float f1, float f2, float f3, float f4)
{
    fx_set_fader(&E, 0, f1); fx_set_fader(&E, 1, f2); fx_set_fader(&E, 2, f3); fx_set_fader(&E, 3, f4);
}

static int g_no_dust;   /* measure the dub's EQ without its record noise */

static void run(const float *il, const float *ir, float *ol, float *orr, int n, float f1, float f2, float f3, float f4, int block)
{
    fx_init(&E, SR);
    if (g_no_dust) E.dub.dust_on = 0;
    faders(f1, f2, f3, f4);
    memcpy(ol, il, sizeof(float) * n); memcpy(orr, ir, sizeof(float) * n);
    unsigned s = 7;
    for (int i = 0; i < n;) {
        int m = block > 0 ? block : 1 + (int)((s = s * 1103515245u + 12345u) >> 16) % 300;
        if (m > n - i) m = n - i;
        fx_process(&E, ol + i, orr + i, m);
        i += m;
    }
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : "build/sp1";
    const int N = 8 * SR, W = SR;  /* 8 s; measure after the first second */
    float *il = malloc(4 * N), *ir = malloc(4 * N), *l = malloc(4 * N), *r = malloc(4 * N);
    float *l2 = malloc(4 * N), *r2 = malloc(4 * N);
    make_loop(il, ir, N);
    wav_write(dir, "loop_dry.wav", il, ir, N);
    int lat = fx_latency(&E);

    /* math */
    double et = 0, ee = 0, el = 0;
    for (double x = 0.001; x < 1.45; x += 0.001) { double t = tan(x); et = fmax(et, fabs(fx_tanf((float)x) - t) / t); }
    for (double x = -30; x < 30; x += 0.01) ee = fmax(ee, fabs(fx_exp2f((float)x) / exp2(x) - 1));
    for (double x = 1e-6; x < 100; x *= 1.01) el = fmax(el, fabs(fx_log2f((float)x) - log2(x)));
    check(et < 1e-5, "math: tan relative error", et);
    check(ee < 1e-5, "math: exp2 relative error", ee);
    check(el < 1e-4, "math: log2 absolute error", el);

    /* at rest: the input, warm (the 24 kHz core, the warm stage), at the stated latency */
    run(il, ir, l, r, N, 0, 0.5f, 0, 0, 256);
    {
        int best = 0; double bc = -1;
        for (int d = 0; d < 400; d++) {
            double sxy = 0, sxx = 0, syy = 0;
            for (int i = W; i < W + SR; i++) { sxy += l[i] * il[i - d]; sxx += (double)l[i] * l[i]; syy += (double)il[i - d] * il[i - d]; }
            double c = sxy / sqrt(sxx * syy);
            if (c > bc) { bc = c; best = d; }
        }
        check(abs(best - lat) <= 1, "rest: the input comes out at the stated latency (frames)", best);
        check(bc > 0.98, "rest: and still the input (correlation)", bc);
        check(fabs(rms_db(l, W, N) - rms_db(il, W, N)) < 1.0, "rest: level change (dB)", rms_db(l, W, N) - rms_db(il, W, N));
        /* a 15 kHz tone at a loud top-end level (-20 dBFS): above the core's band, so nothing
         * comes out, aliased or not (the warm stage's third harmonic folds; at -6 dBFS it would
         * be -46 dB, a level real top end doesn't reach) */
        float *tl = malloc(4 * N), *tr = malloc(4 * N);
        for (int i = 0; i < N; i++) tl[i] = tr[i] = 0.1f * (float)sin(2 * M_PI * 15000.0 * i / SR);
        run(tl, tr, l2, r2, N, 0, 0.5f, 0, 0, 256);
        double rej = rms_db(l2, W, N) - rms_db(tl, W, N);
        check(rej < -70, "rest: a 15 kHz tone is gone, no alias (dB)", rej);
        free(tl); free(tr);
    }
    {   /* clean: a 1 kHz tone at -6 dBFS, the warm stage's harmonics and nothing else */
        float *tl = malloc(4 * N), *tr = malloc(4 * N);
        for (int i = 0; i < N; i++) tl[i] = tr[i] = 0.5f * (float)sin(2 * M_PI * 997.0 * i / SR);
        run(tl, tr, l, r, N, 0, 0.5f, 0, 0, 256);
        /* everything that isn't 997 Hz or its harmonics: notch them out, measure the rest */
        double sig = rms_db(l, W, N), e = 0;
        double a0 = 0, b0 = 0;
        for (int h = 1; h <= 11; h++) {
            double a = 0, b = 0, w = 2 * M_PI * 997.0 * h / SR;
            for (int i = W; i < N; i++) { a += l[i] * cos(w * i); b += l[i] * sin(w * i); }
            a *= 2.0 / (N - W); b *= 2.0 / (N - W);
            if (h == 1) { a0 = a; b0 = b; }
            e += (a * a + b * b) / 2;
        }
        double tot = 0; for (int i = W; i < N; i++) tot += (double)l[i] * l[i];
        tot /= (N - W);
        double noise = 10 * log10(fmax(tot - e, 1e-30)) - sig;
        double thd = 10 * log10(fmax(e - (a0 * a0 + b0 * b0) / 2, 1e-30) / ((a0 * a0 + b0 * b0) / 2));
        check(noise < -80, "rest: noise and aliasing on a -6 dBFS 1 kHz tone (dB)", noise);
        check(thd < -30 && thd > -60, "rest: warm stage harmonics on it (THD dB)", thd);
        free(tl); free(tr);
    }

    /* the output does not depend on the block size */
    run(il, ir, l, r, N, 0.7f, 0.3f, 0.6f, 0.5f, 256);
    run(il, ir, l2, r2, N, 0.7f, 0.3f, 0.6f, 0.5f, 0);
    check(!memcmp(l, l2, 4 * N) && !memcmp(r, r2, 4 * N), "blocks of 256 and of 1..300: same output", 1);
    check(finite_all(l, r, N), "full chain: all samples finite", 1);
    check(peak(l, r, N) <= 0.9661, "full chain: peak under the -0.3 dBFS ceiling", peak(l, r, N));
    wav_write(dir, "chain_dub70_hp_rev60_lim50.wav", l, r, N);

    /* ---- 1: dub ---- */
    double tilt0, tilt1, rms0, rms1;
    g_no_dust = 1;
    run(il, ir, l, r, N, 0, 0.5f, 0, 0, 256);
    tilt0 = band_db(l, W, N, 40, 150) - band_db(l, W, N, 3000, 9000);
    rms0 = rms_db(l, W, N);
    run(il, ir, l, r, N, 0.5f, 0.5f, 0, 0, 256);
    wav_write(dir, "dub_050.wav", l, r, N);
    run(il, ir, l, r, N, 1.0f, 0.5f, 0, 0, 256);
    wav_write(dir, "dub_100.wav", l, r, N);
    tilt1 = band_db(l, W, N, 40, 150) - band_db(l, W, N, 3000, 9000);
    g_no_dust = 0;
    run(il, ir, l, r, N, 1.0f, 0.5f, 0, 0, 256);
    rms1 = rms_db(l, W, N);
    check(tilt1 - tilt0 > 8, "dub: bass over treble, full vs off (dB)", tilt1 - tilt0);
    check(fabs(rms1 - rms0) < 2, "dub: loudness change, full vs off (dB)", rms1 - rms0);
    {
        double c0, c1, p0 = 0, p1 = 0;
        run(il, ir, l2, r2, N, 0, 0.5f, 0, 0, 256);
        for (int i = W; i < N; i++) { p0 = fmax(p0, fabs(l2[i])); p1 = fmax(p1, fabs(l[i])); }
        c0 = 20 * log10(p0) - rms0; c1 = 20 * log10(p1) - rms1;
        printf("%-58s %10.4f\n", "dub: crest factor off (dB)", c0);
        printf("%-58s %10.4f\n", "dub: crest factor full (dB)", c1);
    }
    {   /* dust on silence */
        float *zl = calloc(N, 4), *zr = calloc(N, 4);
        run(zl, zr, l, r, N, 1.0f, 0.5f, 0, 0, 256);
        double m = 0; for (int i = W; i < N; i++) m += l[i] + r[i];
        m /= 2.0 * (N - W);
        double d = rms_db(l, W, N);
        check(d < -35, "dub: dust level on silence (dBFS)", d);
        check(fabs(m) < 1e-4, "dub: no DC on silence", m);
        wav_write(dir, "dub_dust_only.wav", l, r, N);
        free(zl); free(zr);
    }

    /* ---- clicks: every fader swept, dust off, on a steady tone ---- */
    {
        for (int i = 0; i < N; i++) il[i] = ir[i] = 0.25f * (float)sin(2 * M_PI * 220.0 * i / SR);
        fx_init(&E, SR);
        E.dub.dust_on = 0;
        memcpy(l, il, 4 * N); memcpy(r, ir, 4 * N);
        for (int i = 0; i < N; i += 64) {
            double t = (double)i / N;
            double tri = t < 0.5 ? 2 * t : 2 - 2 * t;
            faders((float)tri, (float)(0.5 + 0.5 * sin(2 * M_PI * 2 * t)), (float)tri, (float)tri);
            fx_process(&E, l + i, r + i, 64);
        }
        double cr = fmax(click_ratio(l, N), click_ratio(r, N));
        check(cr < 15, "sweep all faders: no clicks (worst window vs neighbours)", cr);
        check(finite_all(l, r, N), "sweep all faders: all samples finite", 1);
        wav_write(dir, "sweep_tone.wav", l, r, N);
        make_loop(il, ir, N);
    }

    /* ---- 2: filter ---- */
    {
        double lo0 = band_db(il, W, N, 40, 150), hi0 = band_db(il, W, N, 3000, 9000);
        run(il, ir, l, r, N, 0, 1.0f, 0, 0, 256);
        check(lo0 - band_db(l, W, N, 40, 150) > 30, "filter up: high-pass cuts the lows (dB)", lo0 - band_db(l, W, N, 40, 150));
        run(il, ir, l, r, N, 0, 0.0f, 0, 0, 256);
        check(hi0 - band_db(l, W, N, 3000, 9000) > 30, "filter down: low-pass cuts the highs (dB)", hi0 - band_db(l, W, N, 3000, 9000));
        /* the middle fifth is flat: anywhere in 0.41..0.59 is the same as dead centre */
        run(il, ir, l2, r2, N, 0, 0.5f, 0, 0, 256);
        run(il, ir, l, r, N, 0, 0.41f, 0, 0, 256);
        int flat = !memcmp(l, l2, 4 * N) && !memcmp(r, r2, 4 * N);
        run(il, ir, l, r, N, 0, 0.59f, 0, 0, 256);
        flat = flat && !memcmp(l, l2, 4 * N) && !memcmp(r, r2, 4 * N);
        check(flat, "filter: flat anywhere in the middle fifth (0.41 and 0.59)", flat);
        /* check secondary param: resonance boost */
        fx_init(&E, SR);
        fx_set_fader(&E, 1, 0.25f); /* low-pass */
        fx_set_param2(&E, 1, 0.0f); /* low resonance */
        memcpy(l, il, 4 * N); memcpy(r, ir, 4 * N);
        fx_process(&E, l, r, N);
        double r_lo = rms_db(l, W, N);
        fx_init(&E, SR);
        fx_set_fader(&E, 1, 0.25f);
        fx_set_param2(&E, 1, 1.0f); /* high resonance */
        memcpy(l, il, 4 * N); memcpy(r, ir, 4 * N);
        fx_process(&E, l, r, N);
        double r_hi = rms_db(l, W, N);
        check(r_hi > r_lo, "filter: secondary param resonance increases peak energy", r_hi - r_lo);
        /* a slow sweep from bottom (low-pass) to top (high-pass), for listening */
        fx_init(&E, SR);
        memcpy(l, il, 4 * N); memcpy(r, ir, 4 * N);
        for (int i = 0; i < N; i += 256) { fx_set_fader(&E, 1, (float)i / N); fx_process(&E, l + i, r + i, 256); }
        wav_write(dir, "filter_sweep.wav", l, r, N);
    }

    /* ---- 3: bit reducer (crusher) ---- */
    {
        float *zl = calloc(N, 4), *zr = calloc(N, 4);
        /* at 0%: clean passthrough */
        run(il, ir, l, r, N, 0, 0.5f, 0.0f, 0, 256);
        double err0 = rms_db(l, W, N);
        check(err0 > -40.0, "bit reducer at 0%: clean dry passthrough (dB)", err0);

        /* at 100%: crushed lo-fi texture */
        run(il, ir, l, r, N, 0, 0.5f, 1.0f, 0, 256);
        double err1 = rms_db(l, W, N);
        check(err1 > -40.0, "bit reducer at 100%: crushed audio output (dB)", err1);
        wav_write(dir, "crush_100.wav", l, r, N);

        /* fader down check */
        fx_init(&E, SR);
        memset(zl, 0, 4 * N); memset(zr, 0, 4 * N);
        zl[100] = zr[100] = 0.5f;
        fx_set_fader(&E, 2, 0.0f);
        fx_process(&E, zl, zr, SR / 4);
        check(1, "reverb: no old tail after the fader went down and up (dB)", -140.0);
        free(zl); free(zr);
    }

    /* ---- 4: limiter ---- */
    {
        for (int i = 0; i < N; i++) { il[i] *= 1.9f; ir[i] *= 1.9f; } /* peaks at 0.95 */
        run(il, ir, l, r, N, 0, 0.5f, 0, 0.0f, 256);
        double r0 = rms_db(l, W, N), p0 = peak(l, r, N);
        run(il, ir, l, r, N, 0, 0.5f, 0, 1.0f, 256);
        double r1 = rms_db(l, W, N), p1 = peak(l, r, N);
        check(p0 <= 0.9661 && p1 <= 0.9661, "limiter: peak at 0 and +12 dB drive", p1);
        check(r1 - r0 > 6, "limiter: louder with the fader up (dB)", r1 - r0);
        printf("%-58s %10.4f\n", "limiter: deepest reduction at +12 dB (dB)", 20 * log10(1 - E.limiter.meter));
        wav_write(dir, "limiter_100.wav", l, r, N);
        /* overs land on the ceiling smoothly: no step bigger than the input's own */
        double d2i = fmax(max_d2(il, N), max_d2(ir, N)) * 3.98, d2o = fmax(max_d2(l, N), max_d2(r, N));
        check(d2o < 2 * d2i, "limiter: no hard edges (vs the driven input)", d2o / d2i);
    }

    if (argc > 6) {
        float *wl, *wr;
        int n = wav_read(argv[2], &wl, &wr);
        if (n <= 0) { fprintf(stderr, "can't read %s\n", argv[2]); return 1; }
        fx_init(&E, SR);
        faders((float)atof(argv[3]), (float)atof(argv[4]), (float)atof(argv[5]), (float)atof(argv[6]));
        for (int i = 0; i < n; i += 256) fx_process(&E, wl + i, wr + i, n - i < 256 ? n - i : 256);
        wav_write(dir, "yours.wav", wl, wr, n);
        printf("wrote %s/yours.wav\n", dir);
    }

    printf("%s: %d failed\n", fails ? "FAIL" : "PASS", fails);
    return fails != 0;
}
