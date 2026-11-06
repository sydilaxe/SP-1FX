SP-1FX test build for the Teenage Engineering SP-1: a four-effect processor. Audio in over USB-C (a
class-compliant sound card, 48 kHz, 16- or 24-bit: the host picks, so Linux handhelds such as the TrimUI Brick
work too), out on headphones or the speaker.

**Not yet tried on a real SP-1.** It builds, the engine passes its tests, and the platform code follows
firmware that runs on the SP-1, but this binary has not been on hardware yet.

**Flash:** open the [Solderless updater](https://solderless.engineering) in Chrome or Edge, hold **TRACK 1 +
TRACK 4** while you plug the SP-1 in (the four track lights come on), choose `sp1fx.bin`, flash, then unplug
and plug back in. The same plug-in hold always gets back to the bootloader, whatever firmware is on it.

**The faders**, in the order the sound passes through them (each has a second parameter on page 2):
- **1 DUB:** 1970s dub warmth: a big low end, a dark top, tape wow and flutter, gentle glue compression
  and tape saturation, level-matched so it never gets louder. Down is off. Page 2: wobble speed.
- **2 FILTER:** the middle fifth is flat; up sweeps a high-pass, down a low-pass. Page 2: resonance.
- **3 CRUSH:** fewer bits and a lower sample rate; quiet passages are crushed, never dropped. Down is off.
  Page 2: sample-rate range.
- **4 LA-2A:** an optical leveler: up adds peak reduction (with makeup). Down is off. Page 2: compress
  to limit.

A safety ceiling at -1 dBFS stays on after the four and only acts on overs.

**Buttons:** FWD / RWD switch to page 2 / page 1 (a parameter picks up when its fader reaches it, so
nothing jumps), TRACK 1-4 switch each effect off and on, PLAY bypasses all four, VOL + / - set the
volume (3 dB steps), hold FUNCTION to power on (1.5 s) or off (2.5 s), hold TRACK 1 + 4 for 3 s to
restart into the bootloader.

**The sound:** full bandwidth: 48 kHz, 16- or 24-bit in, 24-bit out, 32-bit float inside. With every fader at
rest the output is the input, bit for bit, 1.3 ms later.

**If something is wrong**, open the SP-1's USB serial port: a status line a second shows the faders, the
USB stream, the audio load (`blk=`, microseconds per 2.67 ms block) and the last crash.

**The full user manual** is attached below as `SP-1FX-manual.pdf`.
