#!/usr/bin/env python3
"""Estimate Cortex-M4 cycles per 128-frame block for the SP-1FX effects.

Builds tools/bench_m4.c with the Zephyr SDK's arm-zephyr-eabi-gcc (as the firmware does: -O2,
Cortex-M4F hard float), runs it in Unicorn, and weights each executed instruction with its M4
cycle count (ARM Cortex-M4 TRM, FPU timing: VDIV/VSQRT 14, VMLA/VFMA 3, loads 2, taken branch 3).
Wait states: the nRF52840 runs code from flash with the cache on; this assumes cache hits.

  tools/bench_m4.py [GCC]     GCC defaults to $ZEPHYR_SDK_INSTALL_DIR/arm-zephyr-eabi/bin/...
"""
import os, subprocess, sys, tempfile, collections
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import *
import capstone

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.dirname(here)
args = [a for a in sys.argv[1:] if a != "-v"]
gcc = args[0] if args else os.path.join(
    os.environ.get("ZEPHYR_SDK_INSTALL_DIR", ""), "arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc")
tmp = tempfile.mkdtemp()
elf = os.path.join(tmp, "bench.elf")
subprocess.check_call([gcc, "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16",
                       "-O2", "-fno-math-errno", "-DFX_PROFILE", "-ffreestanding", "-nostdlib", "-Wl,-e,bench_block",
                       "-Wl,-Ttext=0x10000", "-Wl,-Tdata=0x20000000", "-I", os.path.join(root, "fx"),
                       os.path.join(here, "bench_m4.c"), os.path.join(root, "fx", "fx.c"), "-lgcc", "-o", elf])
objcopy = gcc.replace("gcc", "objcopy")
nm = gcc.replace("gcc", "nm")
syms = {}
for line in subprocess.check_output([nm, elf]).decode().splitlines():
    p = line.split()
    if len(p) == 3: syms[p[2]] = int(p[0], 16)
text = os.path.join(tmp, "text.bin"); data = os.path.join(tmp, "data.bin")
subprocess.check_call([objcopy, "-O", "binary", "-j", ".text", "-j", ".rodata", elf, text])
subprocess.check_call([objcopy, "-O", "binary", "-j", ".data", elf, data])

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
uc.mem_map(0x10000, 0x100000)
uc.mem_map(0x20000000, 0x80000)
uc.mem_write(0x10000, open(text, "rb").read())
uc.mem_write(0x20000000, open(data, "rb").read())
# enable the FPU: CPACR full access to CP10/CP11, FPEXC.EN
uc.reg_write(UC_ARM_REG_C1_C0_2, uc.reg_read(UC_ARM_REG_C1_C0_2) | (0xf << 20))
uc.reg_write(UC_ARM_REG_FPEXC, 0x40000000)

md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB | capstone.CS_MODE_MCLASS)
cache = {}
counts = collections.Counter()

def cost(ins):
    m = ins.mnemonic.split(".")[0]
    if m in ("vdiv", "vsqrt"): return 14
    if m in ("vmla", "vmls", "vfma", "vfms", "vnmla", "vnmls"): return 3
    if m in ("vldr", "vstr", "vmov", "vcvt"): return 1 if m == "vcvt" else (2 if m != "vmov" else 1)
    if m in ("vpush", "vpop", "vldm", "vstm", "push", "pop", "ldm", "stm", "ldmia", "stmia", "stmdb"):
        return 1 + max(1, ins.op_str.count(",") + 1)
    if m.startswith("ldr"): return 2
    if m in ("sdiv", "udiv"): return 7
    if m in ("b", "bl", "blx", "bx") or m.startswith("b") and len(m) <= 3 or m in ("cbz", "cbnz"): return 2  # +1 if taken
    return 1

last = [None]
on = [False]
fn = collections.Counter()
funcs = sorted((a, n) for n, a in syms.items() if 0x10000 <= a < 0x20000000)
import bisect
fstarts = [a for a, _ in funcs]
fcache = {}
def func_of(addr):
    f = fcache.get(addr)
    if f is None:
        i = bisect.bisect_right(fstarts, addr & ~1) - 1
        f = fcache[addr] = funcs[i][1] if i >= 0 else "?"
    return f
def hook(uc, addr, size, _):
    if not on[0]: return
    c = cache.get(addr)
    if c is None:
        ins = next(md.disasm(bytes(uc.mem_read(addr, size)), addr))
        c = cache[addr] = (cost(ins), ins.mnemonic)
    counts["ins"] += 1
    counts["cyc"] += c[0]
    fn[func_of(addr)] += c[0]
    # a taken branch: this instruction does not follow the last one
    if last[0] is not None and addr != last[0][0] + last[0][1]:
        counts["cyc"] += 1
    last[0] = (addr, size)

uc.mem_map(0x0, 0x10000)
uc.hook_add(UC_HOOK_CODE, hook)
def call(name):
    uc.reg_write(UC_ARM_REG_SP, 0x20070000)
    uc.reg_write(UC_ARM_REG_LR, 0x0FFF1)   # returning there ends emu_start
    last[0] = None
    uc.emu_start(syms[name] | 1, 0x0FFF0)

call("bench_init")
# settle the smoothed parameters (they glide over ~30 ms), then measure
for _ in range(40):
    call("bench_block")
on[0] = True
N = 20
for _ in range(N):
    call("bench_block")
cyc = counts["cyc"] / N
us = cyc / 64.0
print(f"instructions per block: {counts['ins'] / N:.0f}")
print(f"estimated cycles per 128-frame block: {cyc:.0f} = {us:.0f} us at 64 MHz")
if "-v" in sys.argv:
    for name, c in fn.most_common(12):
        print(f"  {name:28s} {c / N / 128:8.0f} cycles/frame")
print(f"block budget 2667 us: {100 * us / 2667:.0f}% of one core")
