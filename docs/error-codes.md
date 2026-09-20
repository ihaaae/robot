# Error codes and failure reporting

Four different encodings report failure, and they do not share a scale. Most of the confusion
in this area comes from mixing them up.

| What | Where it appears | Range |
|---|---|---|
| **SDK return codes** | the `int` returned by `MoveJ`, `MoveL`, `MoveJ_P`, `IK`, `MoveJ_Canfd`, … | `bot_common::ErrorCode`, `0` = OK, negatives are failures |
| **Lifecycle returns** | `OnRobot`, `EnableRobot`, `DisableRobot`, `Stop`, `ClearFault`, `setJointZeroPosition` | `bool` — these do **not** use the code table |
| **Robot state** | `GetRobotState()` | `Juxie::RobotState`, `0..4` — a state, **not** an error code |
| **Joint status words** | `getJointerrcode()` | 14 `unsigned short` per-joint codes |

> `GetFaultType(x)` takes an **error code**, not a robot state. Passing `GetRobotState()` into
> it returns an unrelated string with no warning — verified: `GetFaultType(4)` answers
> `"Robot goes well"`, because 4 is not an error code.

## SDK return codes

The authoritative numeric list is the vendor's own header,
`vendor/sdk/dual-arm-app/0.6.4/usr/include/state/error_code.h`. The names are self-describing;
this table groups them and records which ones we have actually observed.

**Observed** (in this repository, under emulation):

| Code | Name | When |
|---|---|---|
| `0` | `OK` | success |
| `-1` | `Error` | every motion call, when there is no CAN bus -- the expected result in emulation. `IK` is *not* in this group: it does not touch CAN and solves normally, returning -1 only when a pose is invalid (`-100` in either arm) |

**From the header, not observed by us** — treat the meaning as the vendor's naming, not as
documented behaviour:

| Range | Names | Area |
|---|---|---|
| `-2` … `-8` | `HeadConnectFailed`, `ArmJointCommunicationFailed`, `ArmJointTargetExceedLimits`, `ArmJointTargetSingular`, `ArmRealTimeKernelWrong`, `ArmJointBusWrong`, `ArmPlanningFailed` | hardware / planning |
| `-10` … `-25` | `ArmJointVelExceedLimits`, `ArmEndBoardConnectionWrong`, `ArmVelExceedLimits`, `ArmAccExceedLimits`, `ArmBrakeHold`, `ArmTeachTooFast`, `ArmCollisionHappend`, `ArmNoSuchWorkFrame`, `ArmNoSuchToolFrame`, `ArmNotEnabled`, `ArmControllerTemperatureHigh`, … | limits, brakes, frames, controller health |
| `-26` … `-42` | `ArmJointFOCWrong`, `ArmJointVoltageHigh`, `ArmJointVoltageLow`, `ArmJointTemperatureHigh`, `ArmJointSetupFailed`, `ArmJointEncoderWorng`, `ArmJointCurrentHigh`, … | per-joint faults (note the vendor's spelling: `Worng`, `LosLoop`) |
| `-43` … `-47` | `ROSError`, `IkExceedMaxDis`, `IKFailed`, `CartesianPlanningFailed`, `TrajectoryPlanningFailed` | software / IK / planning |
| `-48` … `-52` | `EmptyPath`, `InCollision`, `OutLimitation`, `NotValid`, `CartesianPlanningTimeout` | the validator |
| `-100` … `-104` | `ExecuteFailed`, `RobotConnectFailed`, `ServerConnectFailed`, `ArmMoving`, `EmergencyStop` | control layer |

**Do not invent retry semantics.** We have no evidence about which of these are transient.
`-1` without CAN means "nothing is attached"; on hardware, seeing it means the call did not
land. Anything else, report it and find out.

> An earlier version of `sdk-usage.md` said motion calls return `-101` `RobotConnectFailed`.
> That was wrong: measured, they return `-1`. `-101` is in the vendor's table but we have never
> observed it. Check the value rather than assuming the name.

## Robot state

`Juxie::RobotState` from `juxie_state.hpp` (that header cannot be included directly — it pulls
in headers the vendor did not ship — so the values are reproduced here):

| Value | Name | Meaning for you |
|---|---|---|
| `0` | `power_off` | `OnRobot()` has not run. Telemetry reads return the `-100` sentinel. |
| `1` | `ready` | powered, not enabled for motion |
| `2` | `idle` | enabled; motion commands are accepted |
| `3` | `running` | a motion is executing |
| `4` | `fault` | **motion is refused.** Clear the fault and find out why before retrying. |

The demos accept `ready` or `idle` as movable and refuse everything else.

## Three useful distinctions

1. **"Refused" is not "failed".** A motion command in the wrong state returns a failure code but
   nothing was attempted. Read `GetRobotState()` first.
2. **A `-100` is a sentinel, not an error.** In a *command* it means "leave this joint alone"; in
   a *read* it means "no data yet". It is not a member of the error table.
3. **Per-joint codes are a separate namespace.** `getJointerrcode()` returns 14 status words;
   `GetAxisFault(code)` decodes one against the vendor's CANopen fault table. A non-zero word
   there is a joint-level condition, not an SDK return code.
