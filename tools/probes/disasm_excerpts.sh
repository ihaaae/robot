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

# libjuxie_controller: Juxie::State* and the ControllerJuxieImpl code that drives them
# (docs/robot-state-machine.md).
ctl=libjuxie_controller.so.0.6.4
# Which State method is which address. Many are 4-8 byte stubs that the linker folded together
# (identical code folding), so one address can stand for several symbols; this table is the key.
(cd "$lib" && "$objdump" -T "$ctl" | grep -E '_ZN5Juxie[0-9]+State' \
    | awk '{print $1, $5, $NF}' | c++filt | sort) >"$out/controller.state-symbols.txt"
range "$ctl" controller.state-stubs.a8408  0xa8408 0xa8410   # return true
range "$ctl" controller.state-stubs.a8538  0xa8538 0xa85b8   # -> SelfCheck(), -101, -19, -1, -103
range "$ctl" controller.state-stubs.ad768  0xad768 0xad7d8   # tail calls to m_MoveEnd / m_Break*
range "$ctl" controller.state-stubs.b4f30  0xb4f30 0xb4f54   # IK: every state -> i_IK
for st in PowerOff Ready Idle Running Fault; do
    m="_ZN5Juxie$((${#st} + 5))State${st}"
    fn "$ctl" "controller.State${st}.SelfCheck" "${m}9SelfCheckEv"
done
fn "$ctl" controller.StatePowerOff.OnRobot    _ZN5Juxie13StatePowerOff7OnRobotEv
fn "$ctl" controller.StateReady.EnableRobot   _ZN5Juxie10StateReady11EnableRobotEv
fn "$ctl" controller.StateReady.OffRobot      _ZN5Juxie10StateReady8OffRobotEv
fn "$ctl" controller.StateIdle.DisableRobot   _ZN5Juxie9StateIdle12DisableRobotEv
fn "$ctl" controller.StateIdle.Stop           _ZN5Juxie9StateIdle4StopEv
fn "$ctl" controller.StateIdle.OffRobot       _ZN5Juxie9StateIdle8OffRobotEv
fn "$ctl" controller.StateRunning.DisableRobot _ZN5Juxie12StateRunning12DisableRobotEv
fn "$ctl" controller.StateRunning.Stop        _ZN5Juxie12StateRunning4StopEv
fn "$ctl" controller.StateRunning.OffRobot    _ZN5Juxie12StateRunning8OffRobotEv
fn "$ctl" controller.StateFault.OffRobot      _ZN5Juxie10StateFault8OffRobotEv
fn "$ctl" controller.State.ClearFault         _ZN5Juxie5State10ClearFaultEv
fn "$ctl" controller.State.m_off_robot        _ZN5Juxie5State11m_off_robotEv
fn "$ctl" controller.Impl.changeState         _ZN5Juxie19ControllerJuxieImpl11changeStateESt10shared_ptrINS_5StateEE
fn "$ctl" controller.Impl.OnRobot             _ZN5Juxie19ControllerJuxieImpl7OnRobotEv
fn "$ctl" controller.Impl.InitRobot           _ZN5Juxie19ControllerJuxieImpl9InitRobotEv
fn "$ctl" controller.Impl.EnableRobot         _ZN5Juxie19ControllerJuxieImpl11EnableRobotEv
fn "$ctl" controller.Impl.UpdateStateThread   _ZN5Juxie19ControllerJuxieImpl17UpdateStateThreadEv
fn "$ctl" controller.GetRobotState            _ZN5Juxie15ControllerJuxie13GetRobotStateEv

# libexecutor: the four predicates UpdateStateThread polls.
fn "$exe" executor.isConnected _ZN12bot_executor13ExecutorJuxie11isConnectedEv
fn "$exe" executor.isEnabled   _ZN12bot_executor13ExecutorJuxie9isEnabledEv
fn "$exe" executor.isMoving    _ZN12bot_executor13ExecutorJuxie8isMovingEv
fn "$exe" executor.isInFault   _ZN12bot_executor13ExecutorJuxie9isInFaultEv

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
ctl = "libjuxie_controller.so.0.6.4"
def timespec(name, off, what):
    b = open(f"{lib}/{name}", "rb").read()[off:off + 16]
    sec, nsec = struct.unpack("<qq", b)
    print(f"{name} @ {off:#x} ({what})")
    print(f"  timespec: {{tv_sec = {sec}, tv_nsec = {nsec}}}")
    print()
timespec(ctl, 0x126ab0, "UpdateStateThread poll period, loaded via adrp 0x126000 + #0xab0")
timespec(ctl, 0x126a90, "StateRunning::DisableRobot / OffRobot wait-for-idle poll, adrp 0x126000 + #0xa90")
dump(ctl, 0x123d28, 0x49, "StateRunning::SelfCheck message")
dump(ctl, 0x123da8, 0x3f, "StatePowerOff::SelfCheck message")
dump(ctl, 0x123e18, 0x2c, "StateIdle::SelfCheck message")
dump(ctl, 0x123e78, 0x3d, "StateFault::SelfCheck message")
dump(ctl, 0x123ee8, 0x41, "StateReady::SelfCheck message")
dump(ctl, 0x1259d8, 0x10, "UpdateStateThread: not connected -> power_off")
dump(ctl, 0x1259e8, 0x14, "UpdateStateThread: -> fault")
dump(ctl, 0x125a00, 0x14, "UpdateStateThread: -> running")
dump(ctl, 0x125a18, 0x12, "UpdateStateThread: -> ready")
dump(ctl, 0x125a30, 0x11, "UpdateStateThread: -> idle")
EOF

echo "wrote $(ls "$out" | wc -l) files to ${out#$root/}"
