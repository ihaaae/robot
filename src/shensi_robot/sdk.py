"""Call the vendor SDK from Python, through the C ABI bridge.

The SDK is a C++ library whose public signatures use ``std::array``, ``std::vector`` and
``Eigen``, none of which ctypes can express. ``python/juxie_sdk_bridge.cpp`` wraps it in plain
C, and this module drives that wrapper: it is the Python counterpart of
``cmake/juxie-sdk.cmake``, and it needs no network and no vendor node.

Both halves are aarch64 Linux, so this runs on the robot's board, not on a workstation. The
build script decides by itself whether to compile natively or cross, and the module finds what
it built, so no environment variables are needed in a checkout:

    ./python/build_bridge.sh
    python3 examples/python/sdk_min_example.py

    from shensi_robot.sdk import Controller
    with Controller() as robot:
        print(robot.state_name(), robot.joint_positions())

``JUXIE_SDK_BRIDGE`` overrides which bridge to load, and ``DUAL_ARM_SDK_CONFIG`` which
configuration root to use. Without the second one, :func:`config_root` points it at the tree
the bridge was built against -- recorded next to the bridge at build time, so the library and
its configuration cannot drift apart.

Read ``docs/sdk-usage.md`` before calling anything that moves the robot: ``on_robot()``
powers the low-level board, several methods need it and segfault without it, and ``fk_pose``
has a hard input limit. The bridge turns those into exceptions instead of crashes, as it does
for C++ exceptions raised across the ABI, but it cannot make an unverified motion safe.

Use one controller from one thread. The bridge serializes creation and destruction; calls in
flight are not protected, and neither is the SDK underneath.
"""
from __future__ import annotations

import ctypes
import os
import threading
from pathlib import Path

import numpy as np

from .config import ConfigNotFoundError

__all__ = [
    "ARM_JOINT_COUNT",
    "CONFIG_ENV",
    "Controller",
    "JOINT_COUNT",
    "SdkError",
    "TCP_QUAT_COUNT",
    "TCP_RPY_COUNT",
    "bridge_path",
    "config_root",
    "state_name",
]

#: Shapes of the SDK's own types, in the order the header documents them.
JOINT_COUNT = 17          # waist 1 + left 7 + right 7 + head 2
ARM_JOINT_COUNT = 14      # left 7 + right 7
TCP_QUAT_COUNT = 14       # (x, y, z, qw, qx, qy, qz) per arm -- what MoveJ_P / MoveL / IK take
TCP_RPY_COUNT = 12        # (x, y, z, rx, ry, rz) per arm -- what getFKpose returns

#: The bridge refuses more values than this, because the SDK copies (n - 7) doubles into a
#: fixed 7-double buffer. In bounds only while n <= 14; the bridge also refuses n < 7.
MAX_FK_VALUES = 14

#: The SDK's own return codes run from -104 to 0; these are the bridge's, in a range the SDK
#: never uses.
ERR_NO_INSTANCE = -1000
ERR_ALREADY_EXISTS = -1001
ERR_NO_ON_ROBOT = -1002
ERR_BAD_SIZE = -1003
ERR_NO_MEMORY = -1004
ERR_EXCEPTION = -1005
ERR_NEEDS_NEW_CONTROLLER = -1006

_BRIDGE_CODES = {
    ERR_NO_INSTANCE: "the bridge has no controller (it was destroyed, or create() failed)",
    ERR_ALREADY_EXISTS: "this process already has a controller; the SDK allows exactly one",
    ERR_NO_ON_ROBOT: "call on_robot() first -- this method segfaults in the SDK without it",
    ERR_BAD_SIZE: f"wrong number or length of values (getFKpose accepts 7..{MAX_FK_VALUES})",
    ERR_NO_MEMORY: "the SDK could not allocate",
    ERR_EXCEPTION: "the SDK raised a C++ exception; the bridge caught it at the ABI boundary",
    ERR_NEEDS_NEW_CONTROLLER: (
        "the SDK allows one on_robot()/off_robot() cycle per controller; repeating it would "
        "abort or segfault the process. close() this controller and create a new one"
    ),
}

#: The environment variable the vendor's own binaries read for their configuration root.
CONFIG_ENV = "DUAL_ARM_SDK_CONFIG"

#: GetRobotState() values, from docs/error-codes.md.
_STATES = {0: "power_off", 1: "ready", 2: "idle", 3: "running", 4: "fault"}


def state_name(value: int) -> str:
    """Name for a GetRobotState() value; the number itself if it is not one we know."""
    return _STATES.get(value, f"state {value}")


class SdkError(RuntimeError):
    """A call into the SDK failed, or the bridge refused it."""

    def __init__(self, method: str, code: int, detail: str = "") -> None:
        self.method = method
        self.code = code
        self.detail = detail
        message = f"{method}() returned {code}"
        if code in _BRIDGE_CODES:
            message += f": {_BRIDGE_CODES[code]}"
        if detail:
            message += f" -- {detail}"
        super().__init__(message)


def bridge_path() -> Path:
    """Where the compiled bridge is. ``JUXIE_SDK_BRIDGE`` wins; otherwise look nearby.

    Raises FileNotFoundError with the build command if it is nowhere to be found.
    """
    candidates: list[Path] = []
    from_env = os.environ.get("JUXIE_SDK_BRIDGE")
    if from_env:
        candidates.append(Path(from_env))
    here = Path(__file__).resolve().parent
    candidates += [
        Path.cwd() / "juxie_sdk_bridge.so",            # wherever you happen to be
        here / "juxie_sdk_bridge.so",                  # copied in next to the package
        here.parent.parent / "python" / "build" / "juxie_sdk_bridge.so",   # a checkout
        # where build_bridge.sh puts it when the checkout itself is not writable
        Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
        / "shensi_robot" / "juxie_sdk_bridge.so",
    ]
    for candidate in candidates:
        if candidate.is_file():
            # Absolute: CDLL() would otherwise hand a bare name to the loader's search path
            # instead of loading the file we just checked.
            return candidate.resolve()
    raise FileNotFoundError(
        "juxie_sdk_bridge.so not found. Build it with ./python/build_bridge.sh and set "
        "JUXIE_SDK_BRIDGE to the result, or leave it at python/build/juxie_sdk_bridge.so. "
        f"Looked at: {', '.join(str(c) for c in candidates)}"
    )


_lib: ctypes.CDLL | None = None


def config_root() -> Path:
    """The vendor configuration root to use, setting ``DUAL_ARM_SDK_CONFIG`` if it is unset.

    An explicit ``DUAL_ARM_SDK_CONFIG`` always wins. Otherwise this reads the tree recorded
    next to the bridge at build time (``juxie_sdk_bridge.sdk``): the configuration then matches
    the tree the bridge was built against, rather than being a guess about which SDK is
    installed. It can still disagree with what is actually loaded if something overrides the
    library search path at run time (``LD_LIBRARY_PATH``).

    Without it the SDK aborts the whole process with ``basic_string::_M_construct null not
    valid`` the moment a controller is constructed, so this raises instead.
    """
    configured = os.environ.get(CONFIG_ENV)
    if configured:
        return Path(configured)

    sidecar = bridge_path().with_suffix(".sdk")
    if sidecar.is_file():
        root = Path(sidecar.read_text().strip())
        etc = root / "usr" / "etc"
        if (etc / "params.yml").is_file():
            os.environ[CONFIG_ENV] = str(etc)
            return etc

    raise ConfigNotFoundError(
        f"no vendor configuration root. Set {CONFIG_ENV}=<sdk-root>/usr/etc, or build the "
        "bridge with ./python/build_bridge.sh, which records the tree it used next to the "
        f"bridge. Looked for {sidecar}."
    )


def _load() -> ctypes.CDLL:
    """Load the bridge and declare every prototype. Cached; loading is not cheap."""
    global _lib
    if _lib is not None:
        return _lib

    lib = ctypes.CDLL(str(bridge_path()))
    dbl_p = ctypes.POINTER(ctypes.c_double)
    ushort_p = ctypes.POINTER(ctypes.c_ushort)
    int_p = ctypes.POINTER(ctypes.c_int)
    handle = ctypes.c_void_p

    lib.juxie_create.argtypes = [ctypes.POINTER(handle)]
    lib.juxie_create.restype = ctypes.c_int
    lib.juxie_destroy.argtypes = [handle]
    lib.juxie_destroy.restype = None

    for name in ("juxie_on_robot", "juxie_off_robot", "juxie_enable_robot",
                 "juxie_disable_robot", "juxie_stop", "juxie_clear_fault",
                 "juxie_get_robot_state", "juxie_get_dof", "juxie_set_joint_zero_position"):
        getattr(lib, name).argtypes = [handle]
        getattr(lib, name).restype = ctypes.c_int

    lib.juxie_get_joint_positions.argtypes = [handle, dbl_p]
    lib.juxie_get_single_torques.argtypes = [handle, dbl_p]
    lib.juxie_get_tcp_pose.argtypes = [handle, dbl_p]
    lib.juxie_get_config.argtypes = [handle, dbl_p, dbl_p]
    lib.juxie_get_joint_err_codes.argtypes = [handle, ushort_p]
    for name in ("juxie_get_joint_positions", "juxie_get_single_torques",
                 "juxie_get_tcp_pose", "juxie_get_config", "juxie_get_joint_err_codes"):
        getattr(lib, name).restype = ctypes.c_int

    lib.juxie_get_fault_type.argtypes = [handle, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
    lib.juxie_get_fault_type.restype = ctypes.c_int
    lib.juxie_get_axis_fault.argtypes = [handle, ctypes.c_ushort, ctypes.c_char_p, ctypes.c_int]
    lib.juxie_get_axis_fault.restype = ctypes.c_int

    lib.juxie_get_fk_pose.argtypes = [handle, dbl_p, ctypes.c_int, ctypes.c_int,
                                      ctypes.c_int, dbl_p]
    lib.juxie_get_fk_pose.restype = ctypes.c_int
    lib.juxie_ik.argtypes = [handle, dbl_p, dbl_p, int_p]
    lib.juxie_ik.restype = ctypes.c_int

    for name in ("juxie_move_j", "juxie_move_j_p", "juxie_move_l"):
        getattr(lib, name).argtypes = [handle, dbl_p, ctypes.c_int]
        getattr(lib, name).restype = ctypes.c_int
    for name in ("juxie_move_j_canfd", "juxie_move_p_canfd"):
        getattr(lib, name).argtypes = [handle, dbl_p, ctypes.c_int]
        getattr(lib, name).restype = ctypes.c_int
    lib.juxie_move_end.argtypes = [handle, ctypes.c_double, ctypes.c_int]
    lib.juxie_move_end.restype = ctypes.c_int
    for name in ("juxie_break_engage", "juxie_break_release"):
        getattr(lib, name).argtypes = [handle, int_p, ctypes.c_int]
        getattr(lib, name).restype = ctypes.c_int

    _lib = lib
    return lib


def _doubles(values, size: int, what: str) -> np.ndarray:
    """Validate and marshal an input array. Length must match exactly."""
    array = np.ascontiguousarray(values, dtype=np.float64).ravel()
    if array.size != size:
        raise ValueError(f"{what} needs {size} values, got {array.size}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{what} contains a non-finite value")
    return array


def _out(size: int, dtype=np.float64) -> np.ndarray:
    return np.zeros(size, dtype=dtype)


def _ptr(array: np.ndarray, ctype=ctypes.c_double):
    return array.ctypes.data_as(ctypes.POINTER(ctype))


class Controller:
    """The vendor's ``Juxie::ControllerJuxie``, driven from Python.

    One per process: the SDK keeps its state in a static, and a second controller plus
    ``on_robot()`` segfaults. Constructing a second one here raises ``SdkError`` instead.

    Use it as a context manager, or call ``close()``.
    """

    def __init__(self) -> None:
        self._handle: ctypes.c_void_p | None = None
        self._lock = threading.Lock()
        # Before anything is constructed: the SDK aborts the process, not just the call, when
        # its configuration root is missing. See config_root().
        config_root()
        handle = ctypes.c_void_p()
        code = int(_load().juxie_create(ctypes.byref(handle)))
        if code != 0:
            raise SdkError("juxie_create", code)
        self._handle = handle

    # ------------------------------------------------------------------ lifetime

    def close(self) -> None:
        """Release the controller. Safe to call twice; the object is unusable afterwards.

        Do not call this while another thread is inside a call on this controller: the bridge
        protects creation and destruction, not calls in flight. Use one controller from one
        thread -- the SDK underneath is not thread-safe either.
        """
        with self._lock:
            if self._handle:
                _load().juxie_destroy(self._handle)
                self._handle = None

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    def _raw(self, method: str, *args) -> int:
        """Call a bridge function. Raises only if there is no controller at all."""
        if self._handle is None:
            raise SdkError(method, ERR_NO_INSTANCE)
        return int(getattr(_load(), method)(self._handle, *args))

    def _call(self, method: str, *args) -> int:
        """Call a method whose return code is the result; raise SdkError if it is not 0.

        ``SdkError.code`` carries the SDK's own code (0 for success, -1 for refused, and so
        on), so a caller can branch; ``fault_type(code)`` decodes it.
        """
        code = self._raw(method, *args)
        if code != 0:
            raise SdkError(method, code)
        return code

    def _value(self, method: str, *args) -> int:
        """Call a query whose return value *is* the answer, such as GetRobotState().

        Those return 0..4 and 7, where 0 and any other non-zero are data rather than failure,
        so only the bridge's own refusals (-1000 and below) raise.
        """
        code = self._raw(method, *args)
        if code <= ERR_NO_INSTANCE:
            raise SdkError(method, code)
        return code

    def _flag(self, method: str, *args) -> bool:
        """Call a method that returns bool. False is a legitimate answer, not an exception.

        ``OnRobot()`` returns false when the board does not come up -- there is no CAN bus, for
        instance -- and the SDK is still usable for FK, configuration and telemetry.
        """
        return self._value(method, *args) == 0

    # ------------------------------------------------------------- state machine

    def on_robot(self) -> bool:
        """Switch the robot to ``ready``. **This powers the low-level board.**

        Returns false when the board does not come up, which is normal with no CAN bus. What
        matters for the methods below is that it was *called*: without it ``get_dof()``,
        ``ik()``, ``joint_err_codes()`` and ``set_joint_zero_position()`` segfault inside the
        SDK, and the bridge refuses them instead.

        Call it **once per controller**. The first call that gets far enough starts the SDK's
        state thread (with no CAN bus that is the second call, because the first one stops in
        ``fault``); after that another ``on_robot()`` would abort the process. After
        ``off_robot()``, calling it again leaves a controller whose destructor segfaults. In both
        cases the bridge raises ``SdkError`` with code ``ERR_NEEDS_NEW_CONTROLLER`` instead. To
        start again, ``close()`` and create a new controller.
        """
        return self._flag("juxie_on_robot")

    def off_robot(self) -> bool:
        """Shut the low-level board down again.

        Only possible while the SDK's state thread has not started (see ``on_robot()``). Once it
        has, ``OffRobot()`` segfaults the process, so the bridge raises ``SdkError`` with code
        ``ERR_NEEDS_NEW_CONTROLLER``. Use ``close()`` then: the destructor stops the thread first.
        """
        return self._flag("juxie_off_robot")

    def enable_robot(self) -> bool:
        """``ready`` -> ``idle``: motion commands are accepted from here."""
        return self._flag("juxie_enable_robot")

    def disable_robot(self) -> bool:
        """``idle`` -> ``ready``."""
        return self._flag("juxie_disable_robot")

    def stop(self) -> bool:
        """Stop a running motion and go back to ``idle``. Not an emergency stop."""
        return self._flag("juxie_stop")

    def clear_fault(self) -> bool:
        """Clear the fault flag. It does not fix the cause."""
        return self._flag("juxie_clear_fault")

    def state(self) -> int:
        """Raw ``GetRobotState()``: 0 power_off, 1 ready, 2 idle, 3 running, 4 fault."""
        return self._value("juxie_get_robot_state")

    def state_name(self) -> str:
        return state_name(self.state())

    # ---------------------------------------------------------------------- reads

    def get_dof(self) -> int:
        """Degrees of freedom, 7 on this machine. Needs ``on_robot()``.

        The number *is* the answer, so it comes back as-is; the bridge refuses the call with
        SdkError if ``on_robot()`` has not run.
        """
        return self._value("juxie_get_dof")

    def joint_positions(self) -> np.ndarray:
        """17 joint angles in radians. Slots for joints this machine lacks stay ``-100``."""
        out = _out(JOINT_COUNT)
        self._call("juxie_get_joint_positions", _ptr(out))
        return out

    def torques(self) -> np.ndarray:
        """14 joint torques, left arm then right arm."""
        out = _out(ARM_JOINT_COUNT)
        self._call("juxie_get_single_torques", _ptr(out))
        return out

    def tcp_pose(self) -> np.ndarray:
        """14 values: (x, y, z, qw, qx, qy, qz) per arm, each relative to its own base."""
        out = _out(TCP_QUAT_COUNT)
        self._call("juxie_get_tcp_pose", _ptr(out))
        return out

    def joint_err_codes(self) -> np.ndarray:
        """14 per-joint status words. Needs ``on_robot()``."""
        out = _out(ARM_JOINT_COUNT, dtype=np.uint16)
        self._call("juxie_get_joint_err_codes", _ptr(out, ctypes.c_ushort))
        return out

    def config(self) -> tuple[np.ndarray, np.ndarray]:
        """``(upper, lower)`` joint limits, 14 each."""
        upper, lower = _out(ARM_JOINT_COUNT), _out(ARM_JOINT_COUNT)
        self._call("juxie_get_config", _ptr(upper), _ptr(lower))
        return upper, lower

    def fault_type(self, code: int) -> str:
        """Description for an SDK return code, from the SDK's own table."""
        return self._string("juxie_get_fault_type", int(code))

    def axis_fault(self, code: int) -> str:
        """Description for a joint fault word, from the vendor's CANopen fault table."""
        return self._string("juxie_get_axis_fault", int(code))

    def _string(self, method: str, code: int) -> str:
        buf = ctypes.create_string_buffer(512)
        length = int(getattr(_load(), method)(self._handle, code, buf, len(buf)))
        if length < 0:
            raise SdkError(method, length)
        if length >= len(buf):
            return buf.value.decode("utf-8", "replace")  # truncated, but still useful
        return buf.value.decode("utf-8", "replace")

    # ----------------------------------------------------------------- kinematics

    def fk_pose(self, joints, left_num: int = 7, right_num: int = 7) -> np.ndarray:
        """Forward kinematics: 12 values, (x, y, z, rx, ry, rz) per arm, ZYX RPY.

        ``joints`` must hold 7 to 14 values. The SDK copies ``len - 7`` doubles into a fixed
        7-double buffer, so more than 14 corrupts the heap; the bridge refuses that instead.
        """
        values = np.ascontiguousarray(joints, dtype=np.float64).ravel()
        if values.size < 7 or values.size > MAX_FK_VALUES:
            raise SdkError("juxie_get_fk_pose", ERR_BAD_SIZE,
                           f"got {values.size}, expected 7..{MAX_FK_VALUES}")
        out = _out(TCP_RPY_COUNT)
        self._call("juxie_get_fk_pose", _ptr(values), int(values.size), int(left_num),
                   int(right_num), _ptr(out))
        return out

    def ik(self, pose) -> np.ndarray:
        """Inverse kinematics for a 14-value quaternion pose. Needs ``on_robot()``.

        Both arms must be solvable: a single ``-100`` in either arm makes the whole call
        return -1, which raises SdkError. On success the array holds one angle per arm joint.
        """
        values = _doubles(pose, TCP_QUAT_COUNT, "pose")
        out = _out(ARM_JOINT_COUNT)
        size = ctypes.c_int(0)
        self._call("juxie_ik", _ptr(values), _ptr(out), ctypes.byref(size))
        return out[: size.value]

    # --------------------------------------------------------------------- motion
    #
    # Read docs/hardware-acceptance.md before any of these. They return the SDK's own code:
    # 0 accepted, -1 refused (usually the wrong state, or no CAN bus).

    def move_j(self, joints, v: int = 10) -> int:
        """Joint move, blocking. 17 values; ``-100`` means "leave this joint alone"."""
        return self._call("juxie_move_j", _ptr(_doubles(joints, JOINT_COUNT, "joints")), int(v))

    def move_j_p(self, pose, v: int = 10) -> int:
        """Cartesian move, blocking. 14 values in quaternion form."""
        return self._call("juxie_move_j_p",
                          _ptr(_doubles(pose, TCP_QUAT_COUNT, "pose")), int(v))

    def move_l(self, pose, v: int = 10) -> int:
        """Linear Cartesian move, blocking. 14 values in quaternion form."""
        return self._call("juxie_move_l",
                          _ptr(_doubles(pose, TCP_QUAT_COUNT, "pose")), int(v))

    def move_j_canfd(self, joints, freq: int = 50) -> int:
        """Online joint streaming at 10-50 Hz. You are the interpolator."""
        return self._call("juxie_move_j_canfd",
                          _ptr(_doubles(joints, JOINT_COUNT, "joints")), int(freq))

    def move_p_canfd(self, pose, freq: int = 50) -> int:
        """Online Cartesian streaming at 10-50 Hz."""
        return self._call("juxie_move_p_canfd",
                          _ptr(_doubles(pose, TCP_QUAT_COUNT, "pose")), int(freq))

    def move_end(self, v: float, part: int) -> int:
        """End-of-motion command. Encoded at 6000 counts/rev on CAN id 0x108."""
        return self._call("juxie_move_end", ctypes.c_double(v), int(part))

    def break_engage(self, numbers) -> int:
        """Engage the brakes for the given joint numbers."""
        ids = np.ascontiguousarray(numbers, dtype=np.int32).ravel()
        return self._call("juxie_break_engage", _ptr(ids, ctypes.c_int), int(ids.size))

    def break_release(self, numbers) -> int:
        """Release the brakes for the given joint numbers."""
        ids = np.ascontiguousarray(numbers, dtype=np.int32).ravel()
        return self._call("juxie_break_release", _ptr(ids, ctypes.c_int), int(ids.size))

    def set_joint_zero_position(self) -> bool:
        """Declare the current position to be zero. Needs ``on_robot()``. Changes calibration."""
        return self._flag("juxie_set_joint_zero_position")
