/* Cortex-M4 cycle estimate for the effects: built for the M4F like the firmware, run in an
 * emulator by tools/bench_m4.py. Worst case: every effect in, a full-scale noise input. */
#include "fx.h"

static fx_engine E;
static float L[128], R[128];
static uint32_t s = 1;

void *memset(void *d, int c, unsigned n)
{
    unsigned char *p = d;
    while (n--) *p++ = (unsigned char)c;
    return d;
}

static void fill(void)
{
    for (int i = 0; i < 128; i++) {
        s = s * 1664525u + 1013904223u; L[i] = (float)(int32_t)s * 4.0e-10f;
        s = s * 1664525u + 1013904223u; R[i] = (float)(int32_t)s * 4.0e-10f;
    }
}

void bench_init(void)
{
    fx_init(&E, 48000.0f);
    fx_set_fader(&E, 0, 1.0f);
    fx_set_fader(&E, 1, 0.15f);
    fx_set_fader(&E, 2, 1.0f);
    fx_set_fader(&E, 3, 1.0f);
}

void bench_block(void)   /* one firmware block: 128 frames */
{
    fill();
    fx_process(&E, L, R, 128);
}
