# SP-1FX

Firmware that turns the Teenage Engineering SP-1 into a four-effect processor. Play audio into it
over USB-C (it shows up as a sound card, "SP-1FX", on a computer, phone or tablet), shape it with
the four faders, and hear it on headphones or the speaker.

**Firmware:** [`sp1fx.bin`](sp1fx.bin)

> **Not yet run on a real SP-1.** It builds, the engine passes its tests, and the platform code
> follows firmware that does run on the SP-1 (see Credits), but this exact binary has not been on
> hardware. Before flashing, read [Flashing and getting back](#flashing-and-getting-back).

## The effects

| Fader | Effect | Down | Up |
|---|---|---|---|
| 1 | **Dub** | off | 70s dub: big low end, a dark top, tape wow and flutter, vinyl hiss, ticks and pops, punchy bus compression and tape saturation |
| 2 | **Filter** | low-pass, sweeping down | high-pass, sweeping up; the middle fifth of the fader is flat, resonance grows toward the ends |
| 3 | **Reverb** | dry | an organic reverb (a modulated 8-line feedback delay network): more, bigger and darker as it goes up, up to about 5 s |
| 4 | **Limiter** | 0 dB drive | +12 dB of drive into a mastering limiter (always on: 1 ms look-ahead, -1 dBFS ceiling) |

The whole thing sounds a little warm and lo-fi, cleanly: a gentle tape saturation and a slight
treble dip at the input, then the effects run at 24 kHz between two half-band filters (flat to
11 kHz, -72 dB at 13 kHz, below -140 dB from 14 kHz). Audio stays 24-bit in and out; inside it is
32-bit floating point.

## Controls

| | |
|---|---|
| Faders 1-4 | the effects |
| TRACK 1-4 | switch that effect off and on (it glides to rest and back) |
| PLAY | bypass: all effects off and on (the warm stage and the limiter stay) |
| VOL + / VOL - | output volume, 3 dB steps |
| FUNCTION, hold 1.5 s | power on |
| FUNCTION, hold 2.5 s | power off |
| TRACK 1 + TRACK 4, hold 3 s | restart into the bootloader, for flashing |

Lights: the track lights show how far each effect is in (a faint glow when it's switched off);
the four status lights show the limiter's gain reduction (0.5, 2, 4 and 8 dB), the volume for a
second after you change it, and breathe slowly in bypass. Plug in headphones and the speaker
turns off.

## Flashing and getting back

The SP-1 is flashed with the [Solderless updater](https://solderless.engineering) in Chrome or Edge:

1. Connect the SP-1 with a USB-C cable.
2. Put it in firmware-loading mode: **hold TRACK 1 + TRACK 4 while you plug in USB** (the
   bootloader's own check, so it works whatever firmware is on it), or, with SP-1FX or the tape
   looper running, hold them for 3 seconds. The four track lights come on.
3. Choose `sp1fx.bin`, flash, unplug, plug back in.

To go back to other firmware, the same steps with that firmware's `.bin`. The SP-1 has no reset
button: if anything ever hangs, the watchdog restarts it within 4 seconds; if it won't start
at all, step 2's plug-in hold still reaches the bootloader.

## Status line

Open the SP-1's USB serial port (any baud rate) for a line a second:
`f=` fader readings, `usb=` streaming, `fill=` the input buffer, `under= over=` buffer
under/overruns, `blk=last/max us` the time one 2.67 ms audio block takes (the CPU check: it must
stay well under 2667), `hp=` headphones, `gr=` limiter reduction, `flt=` a crash from the last
boot. If something is wrong on the hardware, this line is the thing to send.

## CPU and memory

- RAM: about 91 KB of 256 KB (the engine is about 55 KB, mostly the reverb).
- CPU, worst case (every effect fully in, full-scale noise): about 70% of the 64 MHz Cortex-M4,
  estimated by `tools/bench_m4.py`, which runs the engine built for the M4 in an emulator and
  counts cycles from the M4's instruction timings. The real figure is the status line's `blk=`.

## Building

```bash
make test        # the engine on your computer: checks, and WAVs in build/test
make firmware    # sp1fx.bin; needs ZEPHYR_BASE (Zephyr v4.3.1) and the Zephyr SDK 0.17.4
make bench       # the Cortex-M4 cycle estimate (pip install unicorn capstone)
```

`.github/workflows/build.yml` does all three on every push and keeps `sp1fx.bin` and the WAVs as
artifacts.

| Path | |
|---|---|
| `fx/` | the effects engine: plain C, float, no libm, no heap; runs the same on the host and the SP-1 |
| `test/` | host checks (sound, block-size independence, clicks, aliasing, ceilings) and renders |
| `firmware/` | Zephyr application: USB audio (UAC2), I2S, codecs, faders, buttons, lights, power |
| `boards/` | the SP-1 board definition |
| `tools/bench_m4.py` | the CPU estimate |

## Credits

- [chattock/sp1-tape-looper](https://github.com/chattock/sp1-tape-looper) (MIT): the SP-1 board
  definition, the USB audio class backport, and the platform code this firmware follows: codec
  setup, I2S clocking, USB feedback, buttons, power and the bootloader rules. Its licence is in
  `firmware/LICENSE-sp1-tape-looper`.
- [timknapen/SP-1-dev](https://github.com/timknapen/SP-1-dev): the SP-1's pinout, peripherals and
  bootloader documentation.
- The half-band filters use the polyphase IIR design from Laurent de Soras' HIIR.
