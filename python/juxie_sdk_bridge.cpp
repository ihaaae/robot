// A C ABI over Juxie::ControllerJuxie, so Python can call the SDK through ctypes.
//
// The SDK's public signatures use std::array, std::vector and Eigen, none of which ctypes can
// express. This shim exposes the same methods with plain C types: arrays of double in, arrays
// of double out, int return codes. It is the Python counterpart of cmake/juxie-sdk.cmake.
//
// It converts the ways this library can take the interpreter down with it into return codes:
//
//   * C++ exceptions. Nothing may unwind out of an exported function into a C caller, so every
//     one of them runs inside guarded() and reports JUXIE_ERR_NO_MEMORY or JUXIE_ERR_EXCEPTION
//     instead. Vendor abort()/exit() and memory corruption cannot be caught this way, and are
//     not claimed to be.
//   * one controller per process. The SDK keeps its state in a static, and a second
//     ControllerJuxie plus OnRobot() segfaults. juxie_create() refuses, under a mutex, so two
//     threads cannot both win the race. Sequential create -> close -> create was measured
//     under emulation and is fine; the board is unverified.
//   * the four methods that segfault when OnRobot() has not run: getDof, IK, getJointerrcode
//     and setJointZeroPosition. Measured one method per process: SIGSEGV for exactly those
//     four, clean return for the other fourteen. They now return JUXIE_ERR_NO_ON_ROBOT. What
//     the SDK needs is that OnRobot() was *called*, not that it returned true.
//   * getFKpose's heap overflow. It copies (n - 7) doubles into a fixed 7-double buffer, so it
//     is in bounds while n <= 14 (measured: clean at 14, overflow at 15 and 17 under
//     AddressSanitizer). Anything outside 7..14 is refused without calling in.
//
// Threading: the mutex covers creation and destruction only. A call in flight is not
// protected, so use one controller from one thread, and do not close() while another thread is
// inside a call. A single lock across calls would also block a concurrent stop() behind a
// blocking MoveJ, which is worse on a robot.
//
// Build: python/build_bridge.sh. The result is aarch64 Linux, like the SDK itself.
#include <juxie_controller/juxie_controller.h>

#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <vector>

/// Opaque handle. Python only ever sees a pointer to this.
struct juxie_controller;

namespace {

// Bridge-only codes. The SDK's own error codes run from -104 to 0 (state/error_code.h), so
// these cannot be confused with one.
constexpr int JUXIE_ERR_NO_INSTANCE = -1000;
constexpr int JUXIE_ERR_ALREADY_EXISTS = -1001;
constexpr int JUXIE_ERR_NO_ON_ROBOT = -1002;
constexpr int JUXIE_ERR_BAD_SIZE = -1003;
constexpr int JUXIE_ERR_NO_MEMORY = -1004;
constexpr int JUXIE_ERR_EXCEPTION = -1005;

constexpr int kJointCount = 17;   // waist 1 + left 7 + right 7 + head 2
constexpr int kArmCount = 14;     // left 7 + right 7
constexpr int kTcpCount = 12;     // (x, y, z, rx, ry, rz) per arm, what getFKpose returns

struct Bridge {
    Juxie::ControllerJuxie controller;
    bool on_robot = false;
};

std::mutex g_mutex;              // creation and destruction only; see the note above
Bridge *g_bridge = nullptr;

Bridge *bridge(juxie_controller *handle) {
    return reinterpret_cast<Bridge *>(handle);
}

// Run an exported function's body. An exception must not cross the C ABI boundary.
template <typename Body>
int guarded(Body body) noexcept {
    try {
        return body();
    } catch (const std::bad_alloc &) {
        return JUXIE_ERR_NO_MEMORY;
    } catch (...) {
        return JUXIE_ERR_EXCEPTION;
    }
}

Juxie::JointSpaceData to_joint_space(const double *values) {
    Juxie::JointSpaceData out{};
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = values[i];
    return out;
}

Juxie::CartesianSpaceData to_cartesian(const double *values) {
    Juxie::CartesianSpaceData out{};
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = values[i];
    return out;
}

// Copy a std::string out, always NUL-terminated. Returns the full length, so a caller that
// gets length >= buflen knows it was truncated.
int copy_string(const std::string &value, char *buf, int buflen) {
    const int length = static_cast<int>(value.size());
    if (buf != nullptr && buflen > 0) {
        const int take = length < buflen - 1 ? length : buflen - 1;
        std::memcpy(buf, value.data(), static_cast<std::size_t>(take));
        buf[take] = '\0';
    }
    return length;
}

}  // namespace

extern "C" {

/// Create the one controller this process may have, writing it to *out.
///
/// Returns 0, or JUXIE_ERR_ALREADY_EXISTS if a controller already exists, or one of the
/// guarded() codes if it could not be constructed. *out is left null on failure.
int juxie_create(juxie_controller **out) {
    if (out == nullptr) return JUXIE_ERR_BAD_SIZE;
    *out = nullptr;
    return guarded([&] {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_bridge != nullptr) return JUXIE_ERR_ALREADY_EXISTS;
        Bridge *created = new (std::nothrow) Bridge();
        if (created == nullptr) return JUXIE_ERR_NO_MEMORY;
        g_bridge = created;
        *out = reinterpret_cast<juxie_controller *>(created);
        return 0;
    });
}

/// Release a controller. Ignored if the handle is not the live one, so a second close() from
/// another thread is a no-op rather than a double free.
void juxie_destroy(juxie_controller *handle) {
    if (handle == nullptr) return;
    std::lock_guard<std::mutex> lock(g_mutex);
    if (bridge(handle) != g_bridge) return;
    g_bridge = nullptr;
    delete bridge(handle);
}

// ---------------------------------------------------------------------------- state machine

int juxie_on_robot(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        // Note: this powers the low-level board even when there is no CAN bus.
        const bool ready = b->controller.OnRobot();
        // The precondition the SDK actually needs is that this was *called*: measured one
        // method per process, the four methods below segfault when it has not been, and return
        // normally when it has -- whether or not the board came up.
        b->on_robot = true;
        return ready ? 0 : -1;
    });
}

int juxie_off_robot(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        const bool ok = b->controller.OffRobot();
        b->on_robot = false;
        return ok ? 0 : -1;
    });
}

int juxie_enable_robot(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.EnableRobot() ? 0 : -1;
    });
}

int juxie_disable_robot(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.DisableRobot() ? 0 : -1;
    });
}

int juxie_stop(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.Stop() ? 0 : -1;
    });
}

int juxie_clear_fault(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.ClearFault() ? 0 : -1;
    });
}

int juxie_get_robot_state(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.GetRobotState();
    });
}

// ---------------------------------------------------------------------------------- reads

int juxie_get_dof(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        if (!b->on_robot) return JUXIE_ERR_NO_ON_ROBOT;
        return b->controller.getDof();
    });
}

int juxie_get_joint_positions(juxie_controller *handle, double *out17) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || out17 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        const auto joints = b->controller.GetJointPositions();
        for (std::size_t i = 0; i < joints.size(); ++i) out17[i] = joints[i];
        return 0;
    });
}

int juxie_get_single_torques(juxie_controller *handle, double *out14) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || out14 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        const auto torques = b->controller.GetSingleTorques();
        for (std::size_t i = 0; i < torques.size(); ++i) out14[i] = torques[i];
        return 0;
    });
}

int juxie_get_tcp_pose(juxie_controller *handle, double *out14) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || out14 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        // Quaternion form: left (x, y, z, qw, qx, qy, qz) then right.
        const auto pose = b->controller.GetTCPPose();
        for (std::size_t i = 0; i < pose.size(); ++i) out14[i] = pose[i];
        return 0;
    });
}

int juxie_get_joint_err_codes(juxie_controller *handle, unsigned short *out14) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || out14 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        if (!b->on_robot) return JUXIE_ERR_NO_ON_ROBOT;
        const auto codes = b->controller.getJointerrcode();
        for (int i = 0; i < codes.size() && i < kArmCount; ++i) out14[i] = codes[i];
        return 0;
    });
}

int juxie_get_config(juxie_controller *handle, double *upper14, double *lower14) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || upper14 == nullptr || lower14 == nullptr) {
            return JUXIE_ERR_NO_INSTANCE;
        }
        const Juxie::Config config = b->controller.getConfig();
        for (std::size_t i = 0; i < config.upper.size(); ++i) {
            upper14[i] = config.upper[i];
            lower14[i] = config.lower[i];
        }
        return 0;
    });
}

int juxie_get_fault_type(juxie_controller *handle, int code, char *buf, int buflen) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return copy_string(b->controller.GetFaultType(code), buf, buflen);
    });
}

int juxie_get_axis_fault(juxie_controller *handle, unsigned short code, char *buf, int buflen) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return copy_string(b->controller.GetAxisFault(code), buf, buflen);
    });
}

// ---------------------------------------------------------------------------- kinematics

int juxie_get_fk_pose(juxie_controller *handle, const double *joints, int n, int left_num,
                      int right_num, double *out12) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || joints == nullptr || out12 == nullptr) {
            return JUXIE_ERR_NO_INSTANCE;
        }
        // See the header comment: above 14 the SDK overflows a fixed buffer, and below 7 was
        // never measured. Refuse both instead of corrupting the heap.
        if (n < 7 || n > 14) return JUXIE_ERR_BAD_SIZE;
        const std::vector<double> input(joints, joints + n);
        const auto pose = b->controller.getFKpose(input, left_num, right_num);
        for (std::size_t i = 0; i < pose.size() && i < kTcpCount; ++i) out12[i] = pose[i];
        return 0;
    });
}

int juxie_ik(juxie_controller *handle, const double *pose14, double *out_joints, int *out_n) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || pose14 == nullptr || out_joints == nullptr || out_n == nullptr) {
            return JUXIE_ERR_NO_INSTANCE;
        }
        if (!b->on_robot) return JUXIE_ERR_NO_ON_ROBOT;
        Eigen::VectorXd joint;
        const int ret = b->controller.IK(to_cartesian(pose14), joint);
        // The caller's buffer is kArmCount wide. A longer vector would be an SDK change, not
        // something to write past the end of it.
        if (joint.size() > kArmCount) return JUXIE_ERR_BAD_SIZE;
        *out_n = static_cast<int>(joint.size());
        for (int i = 0; i < *out_n; ++i) out_joints[i] = joint[i];
        return ret;
    });
}

// --------------------------------------------------------------------------------- motion
//
// Every one of these needs OnRobot() in practice: the state machine starts at power_off and
// the board is not up. They do not segfault without it, but they cannot succeed either, so
// the failure comes back as an SDK return code rather than as a bridge guard.

int juxie_move_j(juxie_controller *handle, const double *joints17, int v) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || joints17 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.MoveJ(to_joint_space(joints17), v);
    });
}

int juxie_move_j_p(juxie_controller *handle, const double *pose14, int v) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || pose14 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.MoveJ_P(to_cartesian(pose14), v);
    });
}

int juxie_move_l(juxie_controller *handle, const double *pose14, int v) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || pose14 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.MoveL(to_cartesian(pose14), v);
    });
}

int juxie_move_j_canfd(juxie_controller *handle, const double *joints17, int freq) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || joints17 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.MoveJ_Canfd(to_joint_space(joints17), freq);
    });
}

int juxie_move_p_canfd(juxie_controller *handle, const double *pose14, int freq) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || pose14 == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.MoveP_Canfd(to_cartesian(pose14), freq);
    });
}

int juxie_move_end(juxie_controller *handle, double v, int part) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        return b->controller.MoveEnd(v, part);
    });
}

int juxie_break_engage(juxie_controller *handle, const int *ids, int n) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || ids == nullptr) return JUXIE_ERR_NO_INSTANCE;
        if (n < 0) return JUXIE_ERR_BAD_SIZE;   // ids + n would be invalid pointer arithmetic
        const std::vector<int> numbers(ids, ids + n);
        return b->controller.BreakEngage(numbers);
    });
}

int juxie_break_release(juxie_controller *handle, const int *ids, int n) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr || ids == nullptr) return JUXIE_ERR_NO_INSTANCE;
        if (n < 0) return JUXIE_ERR_BAD_SIZE;
        const std::vector<int> numbers(ids, ids + n);
        return b->controller.BreakRelease(numbers);
    });
}

int juxie_set_joint_zero_position(juxie_controller *handle) {
    return guarded([&] {
        Bridge *b = bridge(handle);
        if (b == nullptr) return JUXIE_ERR_NO_INSTANCE;
        if (!b->on_robot) return JUXIE_ERR_NO_ON_ROBOT;
        return b->controller.setJointZeroPosition() ? 0 : -1;
    });
}

}  // extern "C"
