#!/usr/bin/env bash
# Regenerate the disassembly excerpts in research/evidence/disasm/ from the vendored binaries.
#
# Every "confirmed in the binary" claim in docs/can-protocol-comparison.md and
# docs/l3-executor-interface.md points at one of these files. They are committed so the claims
# can be checked without re-running anything; this script exists so they can be re-derived.
#
# Needs llvm-objdump with the AArch64 target (LLVM 18 was used) and python3.
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
lib="$root/vendor/sdk/dual-arm-app/0.6.4/usr/lib"
out="$root/research/evidence/disasm"
objdump="${LLVM_OBJDUMP:-llvm-objdump}"
mkdir -p "$out"

exe=libexecutor.so.0.6.4
drv=librk3576_can_canfd.so.0.6.4

# Whole functions, by mangled name.
fn() {  # fn <library> <output-name> <mangled symbol>
    (cd "$lib" && "$objdump" -d --no-show-raw-insn --disassemble-symbols="$3" "$1") >"$out/$2.txt"
}
# Address ranges, for functions too large to keep whole (the ExecutorJuxie constructor).
range() {  # range <library> <output-name> <start> <stop>
    (cd "$lib" && "$objdump" -d --no-show-raw-insn --start-address="$3" --stop-address="$4" "$1") >"$out/$2.txt"
}

# libexecutor: bot_executor::ExecutorJuxie
fn "$exe" executor.MoveEnd                    _ZN12bot_executor13ExecutorJuxie7MoveEndEdi
fn "$exe" executor.ucas_can0_task_send_thread _ZN12bot_executor13ExecutorJuxie26ucas_can0_task_send_threadEPv
fn "$exe" executor.ucas_can1_task_send_thread _ZN12bot_executor13ExecutorJuxie26ucas_can1_task_send_threadEPv
fn "$exe" executor.sendCommandThread0         _ZN12bot_executor13ExecutorJuxie18sendCommandThread0Ev
fn "$exe" executor.SetSending                 _ZN12bot_executor13ExecutorJuxie10SetSendingEbi
range "$exe" executor.ctor.thread-start 0xec5c0 0xec6a4   # which member functions get a thread
range "$exe" executor.ctor.flags        0xec7e0 0xec8a8   # left_used_/right_used_ = 1, m_useLimit = 1
range "$exe" executor.ctor.resample     0xed22c 0xed25c   # resample_delta <- YAML "Resample"
range "$exe" executor.ctor.limits-parse 0xed4b0 0xed838   # UseLimit, LeftLimits, RightLimits (N x 2)
range "$exe" executor.ctor.limits-apply 0xee480 0xee6a8   # limits columns -> JointVelocityPlanner

# librk3576_can_canfd: rk3576_can_canfd::RK3576CanCanfdImpl
fn "$drv" driver.sendJointControl        _ZN16rk3576_can_canfd18RK3576CanCanfdImpl16sendJointControlE16JointControlType
fn "$drv" driver.sendJointBreak          _ZN16rk3576_can_canfd18RK3576CanCanfdImpl14sendJointBreakERKSt6vectorIiSaIiEE9BreakType
fn "$drv" driver.rk3576_canfd_recv_frame_data _ZN16rk3576_can_canfd18RK3576CanCanfdImpl28rk3576_canfd_recv_frame_dataE6Can_If
fn "$drv" driver.ucas_can0_task_send_thread   _ZN16rk3576_can_canfd18RK3576CanCanfdImpl26ucas_can0_task_send_threadEPv

# .rodata constants and strings referenced by the excerpts above.
python3 - "$lib" >"$out/rodata.txt" <<'EOF'
import struct, sys
lib = sys.argv[1]
def dump(name, off, n, what):
    b = open(f"{lib}/{name}", "rb").read()[off:off + n]
    print(f"{name} @ {off:#x} ({what})")
    print("  bytes :", b.hex(" "))
    if n == 8:
        print("  double:", repr(struct.unpack("<d", b)[0]))
    printable = [s.decode() for s in b.split(b"\0") if s and all(32 <= c < 127 for c in s)]
    if printable and n > 8:
        print("  strings:", printable)
    print()
exe = "libexecutor.so.0.6.4"
dump(exe, 0x164830, 8, "pi")
dump(exe, 0x164838, 8, "2*pi; MoveEnd and sendCommandThread0 load it via adrp 0x164000 + #0x838")
dump(exe, 0x164818, 8, "resample_delta default, stored at this+0x10 before the YAML override")
dump(exe, 0x163ac0, 7, "0x200 sub-frame template copied by ExecutorJuxie::ucas_can0_task_send_thread")
dump(exe, 0x163470, 16, "YAML key read into this+0x10")
dump(exe, 0x1634c0, 0x80, "YAML keys read in the constructor limits block")
EOF

echo "wrote $(ls "$out" | wc -l) files to ${out#$root/}"
