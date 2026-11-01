SP-1FX test build for the Teenage Engineering SP-1: a four-effect processor. Audio in over USB-C (a
class-compliant sound card, 48 kHz / 24-bit), out on headphones or the speaker.

**Not yet tried on a real SP-1.** It builds, the engine passes its tests, and the platform code follows
firmware that runs on the SP-1, but this binary has not been on hardware yet.

**Flash:** open the [Solderless updater](https://solderless.engineering) in Chrome or Edge, hold **TRACK 1 +
TRACK 4** while you plug the SP-1 in (the four track lights come on), choose `sp1fx.bin`, flash, then unplug
and plug back in. The same plug-in hold always gets back to the bootloader, whatever firmware is on it.

**The faders**, in the order the sound passes through them:
- **1 DUB:** 70s dub reggae: a big low end, a dark top, tape wow and flutter, vinyl hiss, ticks and pops,
  punchy compression and tape saturation. Down is off.
- **2 FILTER:** the middle fifth is flat; up sweeps a high-pass, down a low-pass, resonance growing toward
  the ends.
- **3 REVERB:** an organic reverb, louder, bigger and darker as it goes up (to about 5 s). Down is dry.
- **4 LIMITER:** a mastering limiter, always on at -1 dBFS; up drives into it, to +12 dB.

**Buttons:** TRACK 1-4 switch each effect off and on, PLAY bypasses all four, VOL + / - set the volume
(3 dB steps), hold FUNCTION to power on (1.5 s) or off (2.5 s), hold TRACK 1 + 4 for 3 s to restart into the
bootloader.

**The sound:** a little warm and lo-fi, cleanly: gentle tape saturation and a slight treble dip, and the
effects run at 24 kHz between two steep half-band filters, so the top rolls off softly above 11 kHz with no
grit (noise and aliasing below -125 dB).

**If something is wrong**, open the SP-1's USB serial port: a status line a second shows the faders, the
USB stream, the audio load (`blk=`, microseconds per 2.67 ms block) and the last crash.
