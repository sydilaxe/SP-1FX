# SP-1FX

Firmware that turns the Teenage Engineering SP-1 into a four-effect processor: audio comes in over
USB-C (class compliant, 48 kHz / 24-bit), goes through one effect per fader, and out of the
headphones or the speaker.

| Fader | Effect | Down | Up |
|---|---|---|---|
| 1 | Dub | off | 70s dub: big low end, dark top, tape wow and flutter, vinyl dust, punchy compression and tape saturation |
| 2 | Filter | high-pass | low-pass (centre = flat) |
| 3 | Reverb | dry | organic reverb, bigger and longer |
| 4 | Limiter | 0 dB drive | +12 dB into a -0.3 dBFS mastering limiter (always on) |

Work in progress: the firmware builds, the engine passes its host tests, and the worst case does
not fit the CPU yet (see `make bench`). No `sp1fx.bin` until it does.

Built on chattock/sp1-tape-looper (MIT) for the SP-1 platform code and Tim Knapen's SP-1-dev
documentation.
