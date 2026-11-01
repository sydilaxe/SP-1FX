# SP-1FX
#   make test       the effects engine on the host: checks, and WAVs in build/test
#   make firmware   sp1fx.bin (needs ZEPHYR_BASE and the Zephyr SDK; see README)
#   make bench      Cortex-M4 cycle estimate for the effects (needs the SDK and unicorn)
CC ?= cc
CFLAGS := -O2 -std=gnu11 -Wall -Wextra -fno-math-errno -ffp-contract=off

test: build/fx_test
	mkdir -p build/test
	./build/fx_test build/test

build/fx_test: test/fx_test.c fx/fx.c fx/fx.h fx/fx_math.h
	mkdir -p build
	$(CC) $(CFLAGS) -o $@ test/fx_test.c fx/fx.c -lm

firmware:
	./scripts/build-firmware.sh

bench:
	python3 tools/bench_m4.py -v

.PHONY: test firmware bench
