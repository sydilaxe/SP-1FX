#!/usr/bin/env bash
# The user manual as a PDF: docs/manual/manual.md -> build/manual/<name>.pdf
#
#   scripts/build-manual.sh            version from `git describe` (or VERSION=v1.2.3)
#
# Needs pandoc and a Chrome or Chromium (CHROME=/path/to/chrome to pick one).
set -euo pipefail
cd "$(dirname "$0")/.."

src=docs/manual
out=build/manual
name=$(sed -n 's/^pdf-name: *//p' $src/manual.md | head -1)
version=${VERSION:-$(git describe --tags --always --dirty 2>/dev/null || echo dev)}
date=$(git log -1 --format=%cs 2>/dev/null || date +%F)

chrome=${CHROME:-}
if [ -z "$chrome" ]; then
	for c in google-chrome google-chrome-stable chromium chromium-browser \
		/opt/pw-browsers/chromium-*/chrome-linux*/chrome; do
		if command -v "$c" >/dev/null 2>&1; then chrome=$(command -v "$c"); break; fi
	done
fi
[ -n "$chrome" ] || { echo "no Chrome or Chromium found (set CHROME)" >&2; exit 1; }

mkdir -p $out
pandoc $src/manual.md \
	--from markdown-implicit_figures \
	--to html5 --standalone \
	--template $src/template.html \
	--toc --toc-depth=2 \
	--metadata version="$version" --metadata date="$date" \
	--css style.css \
	-o $out/manual.html
cp $src/style.css $out/

"$chrome" --headless --no-sandbox --disable-gpu --no-pdf-header-footer \
	--print-to-pdf="$PWD/$out/$name.pdf" "file://$PWD/$out/manual.html" 2>/dev/null
test -s $out/$name.pdf
echo "$out/$name.pdf ($version)"
