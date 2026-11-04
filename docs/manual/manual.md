---
title: SP-1FX
subtitle: A four-effect processor. Dub, Filter, Crush and an LA-2A style leveler on four faders.
pdf-name: SP-1FX-manual
---

<!-- Keep this manual in step with the firmware: any change to what a control does, a light shows,
     a range, a timing or the flashing steps belongs here too. CI builds it into
     SP-1FX-manual.pdf and attaches it to every release. -->

# Before you start

SP-1FX is replacement firmware for the Teenage Engineering SP-1. It turns the SP-1 into an audio
effects processor: you play audio into it over USB-C, shape it with the four faders, and hear the
result on headphones or the built-in speaker.

> **Test build.** This firmware builds, its effects engine passes its tests, and its platform code
> follows firmware that does run on the SP-1, but it has not yet been run on a real SP-1. Read
> [Flashing and getting back](#flashing-and-getting-back) before you install it: the SP-1 can
> always be returned to other firmware.

## What you need

- A Teenage Engineering SP-1.
- A USB-C cable.
- An audio source that can play to a USB sound card: a computer, phone, tablet or Linux handheld.
- For installing: a computer with Chrome or Edge.

## Quick start

1. Install the firmware (see [Flashing and getting back](#flashing-and-getting-back)).
2. Hold **FUNCTION** for 1.5 seconds to power on.
3. Connect the SP-1 to your computer or phone with USB-C. It appears as a sound card called
   **SP-1FX**. Choose it as the audio output and play something.
4. Plug in headphones, or listen on the speaker.
5. Push a fader up to bring in its effect. All four faders down means no effect at all.

# The effects

The sound passes through the four effects in fader order, left to right: **Dub**, **Filter**,
**Crush**, then **LA-2A**. With a fader at rest (down, or centred for the Filter) its effect is out
of the signal completely.

Each fader also has a **second parameter** on page 2 (see [Pages](#pages-and-pick-up)).

| Fader | Effect | At rest | Pushed up | Page 2 |
|---|---|---|---|---|
| 1 | **Dub** | down (off) | 1970s dub warmth | tape wobble speed |
| 2 | **Filter** | centre (flat) | down: low-pass, up: high-pass | resonance |
| 3 | **Crush** | down (off) | fewer bits, lower sample rate | sample-rate range |
| 4 | **LA-2A** | down (off) | more leveling, with makeup gain | compress (3:1) to limit (10:1) |

## 1 · Dub

A 1970s dub warmth: a big low end, a dark top (rolling off down to 4.7 kHz at the top of the
fader), tape wow and flutter, gentle 2:1 glue compression and tape saturation. It is
level-matched: pushing it up changes the colour, never the loudness.

**Page 2: wobble speed.** How fast the tape wow and flutter moves.

## 2 · Filter

One fader, two filters. The middle fifth of the fader is flat (no filtering). Moving **down** from
the centre sweeps a low-pass filter down from 20 kHz; moving **up** sweeps a high-pass filter up to
6 kHz.

**Page 2: resonance.** The emphasis at the cutoff frequency, from smooth to peaky.

## 3 · Crush

A bit and sample-rate reducer. Pushing up reduces the resolution from 16 bits down to 4 and the
sample rate from 48 kHz down to 2 kHz. The bit steps follow the signal's level, so quiet passages
are crushed too instead of dropping out to silence.

**Page 2: sample-rate range.** How far the sample rate falls as the fader goes up.

## 4 · LA-2A

An optical leveling amplifier in the style of the LA-2A. Pushing up adds more peak reduction,
with makeup gain so the level stays up. It reacts in about 10 ms and releases in two stages:
about half back in 60 ms, the rest over 0.5 to 2 seconds, longer after long, hard passages.

**Page 2: compress to limit.** From a gentle 3:1 compression at the bottom to 10:1 limiting at the
top.

## The safety ceiling

After the four effects, a safety limiter at -1 dBFS (1 ms look-ahead) is always on. It only acts
on overs, to protect the speaker and your ears, and leaves everything below the ceiling untouched.

# Controls

| Control | Action |
|---|---|
| Faders 1-4 | the effects (page 1) or their second parameters (page 2) |
| FWD (>>) | go to page 2 |
| RWD (<<) | go to page 1 |
| TRACK 1-4 | switch that effect off and on (it glides to rest and back) |
| PLAY | bypass: all four effects glide to rest and back |
| VOL + / VOL - | output volume, 3 dB steps |
| FUNCTION, hold 1.5 s | power on |
| FUNCTION, hold 2.5 s | power off |
| TRACK 1 + TRACK 4, hold 3 s | restart into the bootloader, for flashing |

## Pages and pick-up

The faders have two pages. **Page 1** sets how much of each effect you hear. **Page 2** sets each
effect's second parameter. Press **FWD (>>)** for page 2 and **RWD (<<)** to go back to page 1.
For 0.6 seconds after you press either, the status lights show the page: the left pair for
page 1, the right pair for page 2.

When you change page, the faders are usually not where that page's values are. So nothing
jumps, a parameter keeps its value until you move its fader to that value (or sweep past it).
Its track light blinks until the fader has picked it up.

## Switching effects off and bypass

**TRACK 1-4** switch one effect off and back on, keeping its fader setting. The effect glides out
and back in, so there are no clicks. While an effect is switched off, its track light glows
faintly.

**PLAY** bypasses all four effects at once, with the same glide. The safety ceiling stays on. In
bypass the status lights breathe slowly.

## Volume

**VOL +** and **VOL -** change the output volume in 3 dB steps, from 0 dB down to -60 dB. The
SP-1FX starts at -6 dB. For a second after a change the status lights show the volume.

## Headphones and speaker

Plug in headphones and the speaker turns off. Unplug them and the speaker comes back.

# Lights

| Lights | Page 1 | Page 2 |
|---|---|---|
| Track lights | how far each effect is in; a faint glow while it is switched off | the second parameters |
| Status lights | the LA-2A's gain reduction | the LA-2A's gain reduction, the fourth light blinking |

**Track lights** blink while a fader is waiting to pick up its parameter after a page change.

**Status lights** normally meter the LA-2A's gain reduction: one light from 0.5 dB, two from 2 dB,
three from 4 dB and four from 8 dB. They switch to other information for a moment:

- for 0.6 s after FWD or RWD: the page (left pair = page 1, right pair = page 2);
- for 1 s after VOL + or VOL -: the volume;
- in bypass: all four breathe slowly.

# Power and charging

- **Power on:** hold **FUNCTION** for 1.5 seconds. The status lights fill up as you hold.
- **Power off:** hold **FUNCTION** for 2.5 seconds.
- **Charging:** while the SP-1 is off and plugged in, the status lights show the battery level,
  with the top lit light blinking while it charges.
- **Settings:** the effects, page and volume are not stored. Each time you power on, the SP-1FX
  starts on page 1 with the volume at -6 dB and the effects set by where the faders are.

If the firmware ever hangs, a watchdog restarts it within 4 seconds and it comes straight back
up.

# Flashing and getting back

The SP-1 is flashed with the [Solderless updater](https://solderless.engineering) in Chrome or
Edge. Download `sp1fx.bin` from the release page first.

1. Connect the SP-1 to the computer with a USB-C cable.
2. Put it in firmware-loading mode: **hold TRACK 1 + TRACK 4 while you plug in USB**. This is the
   bootloader's own check, so it works whatever firmware is on the SP-1. With SP-1FX (or the tape
   looper) already running, you can instead hold TRACK 1 + TRACK 4 for 3 seconds. The four track
   lights come on.
3. In the updater, choose `sp1fx.bin` and flash it.
4. Unplug the SP-1 and plug it back in.

**Going back** to other firmware uses the same steps with that firmware's `.bin` file. The SP-1
has no reset button: if it won't start at all, the plug-in hold in step 2 still reaches the
bootloader.

# Troubleshooting

| Problem | Try |
|---|---|
| No sound | Check that **SP-1FX** is selected as the output on your computer or phone, that PLAY bypass is not hiding an effect you expect, and that the volume is up (VOL +). |
| The SP-1FX doesn't appear as a sound card | Use a USB-C cable that carries data, not a charge-only one. Power the SP-1 on with FUNCTION. |
| An effect does nothing | Its fader may be at rest, or the effect switched off with its TRACK button (faint track light). |
| A fader doesn't respond after a page change | It is waiting to pick up: move it to where the light's value is, or sweep it past. |
| Crackles or dropouts | Send the status line (below). |
| It won't start | Hold TRACK 1 + TRACK 4 while plugging in USB and reflash. |

## The status line

For a detailed report, open the SP-1's USB serial port in any serial terminal (any baud rate). It
prints one line a second:

| Field | Meaning |
|---|---|
| `f=` | fader readings |
| `usb=` | whether audio is streaming |
| `fill=` | how full the input buffer is |
| `under=` `over=` | input buffer underruns and overruns |
| `blk=last/max us` | how long one 2.67 ms audio block takes; must stay well under 2667 |
| `hp=` | headphones detected |
| `gr=` | the LA-2A's gain reduction |
| `flt=` | a crash from the previous boot, if any |

If something is wrong on the hardware, this line is the thing to send with your report. If the
Filter's flat zone doesn't sit at the centre of the fader, send the `f=` readings with the faders
at the bottom, middle and top.

# Specifications

| | |
|---|---|
| Audio in | USB-C, class-compliant USB audio (UAC2), 48 kHz stereo, 16- or 24-bit (the host picks) |
| Audio out | headphones or speaker, 48 kHz / 24-bit |
| Processing | 32-bit floating point at 48 kHz |
| Latency | 64 samples (1.3 ms); with every fader at rest the output is the input, bit for bit |
| Safety ceiling | -1 dBFS, 1 ms look-ahead, always on |
| Volume | 0 to -60 dB in 3 dB steps |
| CPU | about 42% of the 64 MHz Cortex-M4 worst case (estimated) |
| RAM | about 37 KB of 256 KB |

# Credits

- [chattock/sp1-tape-looper](https://github.com/chattock/sp1-tape-looper) (MIT): the SP-1 board
  definition, the USB audio class backport, and the platform code this firmware follows.
- [timknapen/SP-1-dev](https://github.com/timknapen/SP-1-dev): the SP-1's pinout, peripherals and
  bootloader documentation.

SP-1FX is not made or endorsed by Teenage Engineering.
