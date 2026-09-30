# Disassembly excerpts

AArch64 disassembly of the vendor libraries in `vendor/sdk/dual-arm-app/0.6.4/usr/lib/`, kept so
that every "found in the binary" claim in `docs/` can be checked by reading a file instead of
re-running a disassembler.

Regenerate with `tools/probes/disasm_excerpts.sh` (needs `llvm-objdump` with the AArch64 target;
LLVM 18.1.3 was used). The files are the tool's plain output; nothing is edited by hand.

## Reading them

Two traps caused wrong claims before, so they are worth stating:

* `adrp xN, 0x8d000` followed by `add xN, xN, #0x580` computes the **address** `0x8d580`. It is not
  a comparison with CAN ID `0x580`. A CAN ID test looks like `cmp w0, #0x301` or an `orr` that
  builds an ID which is then passed as the third argument (`w2`) of `can_send_frame`.
* `add x0, x0, #0x110` after loading `this` is a **member offset**. Only a value that reaches the
  `can_id` argument of `can_send_frame` is a CAN ID.

`can_send_frame(Can_If can, uint32_t id, uint32_t len, uint8_t *data)` is called through the vtable
(`ldr x5, [x1, #0x18]` … `blr x5`), so at each `blr x5` read `w1` = bus, `w2` = CAN ID,
`w3` = length, `x4` = data.

## What each file supports

| File | Claim (in `docs/can-protocol-comparison.md` unless noted) |
|---|---|
| `executor.ctor.thread-start.txt` | Only `ucas_can0_task_send_thread`, `ucas_can1_task_send_thread` and `watchdog` get a thread; `sendCommandThread0/1` and `listenStateThread` are never called (§1.1) |
| `executor.ctor.resample.txt` + `rodata.txt` | `this+0x10` (`resample_delta`) defaults to 0.005 and is overwritten from YAML `Resample` (§1.1) |
| `executor.ucas_can0_task_send_thread.txt`, `executor.ucas_can1_task_send_thread.txt` | One frame per tick: `0x200` DLC 64 when the command queue has a point, otherwise `0x80` DLC 8 (all zero) if `left_used_`/`right_used_`; sub-frame template `C6 …`; Dev_IDs at `[56+i]`; sleep step `this+0x10 × 1e9` ns (§1.1) |
| `executor.ctor.flags.txt` | `left_used_` / `right_used_` (`+0x4bc` / `+0x4bd`) set to 1; `m_useLimit` (`+0x5f8`) default 1 |
| `executor.ctor.limits-parse.txt`, `executor.ctor.limits-apply.txt` | `UseLimit` → `+0x5f8`; `LeftLimits` / `RightLimits` read as N×2; column 0 → `JointVelocityPlanner::max_velocity`, column 1 → `max_acc` (`hardware-acceptance.md` P0-4) |
| `executor.sendCommandThread0.txt` | The dead-code per-joint frame `0x200 \| (i+1)`, DLC 8, `0F 00 00 00` + little-endian int32 (§1.1) |
| `executor.MoveEnd.txt` | `0x108` DLC 7 = `C4 HI LO 03 E8 00 00`; `part` 0 → both buses, 1 → CAN0, 2 → CAN1 (§2) |
| `executor.SetSending.txt` | `SetSending(b, 1)` / `(b, 2)` write the atomics at `+0x4ba` / `+0x4bb` (`l3-executor-interface.md` §8) |
| `driver.sendJointControl.txt` | SDO `0x600 \| id`; control word bytes per `JointControlType`, including `StopMotion` = `0F 10` (§3); no `0x110` |
| `driver.sendJointBreak.txt` | `0x100 \| id` (§1, §4) |
| `driver.rk3576_canfd_recv_frame_data.txt` | Receive dispatch compares only `0x301`…`0x307`; nothing for `0x580` (§1) |
| `driver.ucas_can0_task_send_thread.txt` | The `adrp`/`add #0x580` address arithmetic; `clock_nanosleep` with `SLEEP_TIME` = 200000 ns (`l3-executor-interface.md` §2.1) |
| `rodata.txt` | π / 2π constants, the 0.005 default, the `0x200` sub-frame template, the YAML key strings |

Member names such as `left_used_` come from matching offsets against the declaration order in
`usr/include/executor/ExecutorJuxie.hpp`; the offsets themselves are what the code shows.
