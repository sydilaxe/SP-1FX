# Keeping the docs in step

The user manual is `docs/manual/manual.md`. CI turns it into `SP-1FX-manual.pdf` (`scripts/build-manual.sh`)
and attaches it to every GitHub release, so it must always describe the firmware as it is.

Whenever a change affects what a player sees, hears or does (a control, a light, a model or effect,
a range or timing, the status line, the flashing steps, the specs such as CPU, RAM or latency),
update in the same commit:

- `docs/manual/manual.md`: the full manual, written for players, not developers;
- `README.md`;
- `.github/release-notes.md`.

Check the PDF with `./scripts/build-manual.sh` (needs pandoc and Chrome or Chromium). CI warns on a
pull request that changes `firmware/src/` or `fx/` without touching the manual.
