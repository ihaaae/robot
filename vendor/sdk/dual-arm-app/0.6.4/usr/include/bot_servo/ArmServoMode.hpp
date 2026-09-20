#ifndef DUAL_ARM_APP_SERVOMODE_HPP
#define DUAL_ARM_APP_SERVOMODE_HPP

#include "bot_planner/planner_base.hpp"
#include "bot_traj_planner/trajectory_base.hpp"
#include "bot_executor/ExecutorBase.hpp"
#include "state/error_code.h"
#include <shared_mutex>
#include <thread>
#include <atomic>
#include "bot_subscriber/multi_thread_version.hpp"
#include "thread_safe_container/thread_safe_deque.hpp"

template<typename T>
class FilterBase;

template<typename T>
using FilterBasePtr = std::shared_ptr<FilterBase<T>>;

namespace bot_servo {
    enum ServoType{
        Trajectory = 0,
        Direct,
        Interpolation,
        TrajectoryFollow,
    };

    struct StampedJoint{
    public:
        StampedJoint(const Eigen::VectorXd & joint);

    public:
        Eigen::VectorXd joint_data;
        std::chrono::steady_clock::time_point stamp;
    };

    class ArmServoMode {
    public:
        ArmServoMode(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext, ServoType type = Trajectory, int part = 0);

        ~ArmServoMode();

    protected:
        bot_executor::ExecutorPtr ext_;
        bot_planner::PlannerPtr pl_;
        double freq;

        std::atomic_bool startServo{false};

        ThreadSafeDeque<StampedJoint> buffer;

        std::mutex buffer_mutex;
        std::condition_variable buffer_condition;

        bot_common::ErrorInfo execute_ret;

        std::thread servo_worker, maintain_worker, watch_dog;

        ServoType type_;

        int checked_counter = 0 ;

        FilterBasePtr<Eigen::VectorXd> m_filter;

        double m_vel_factor = 1.;
        double m_acc_factor = 1.;
        double m_jek_factor = 1.;

        MultiThreadVersion<Eigen::VectorXd> send_data;

        int part_;
        float vecFactor_;
        float accFactor_;
        float jerkFactor_;

    public:
        bot_common::ErrorInfo startServoMode(double velocity_factor = 1.0, double acc_factor = 1.0, double servo_rate= 30.);

        bot_common::ErrorInfo servoToPoint(const Eigen::VectorXd& q);

        std::pair <bot_common::ErrorInfo, Eigen::VectorXd> servoToPose(const Eigen::Isometry3d &p);

        bot_common::ErrorInfo endServoMode();

        bool isServoHalt();

    protected:
        void servoFrame();

        void trajectoryWorker(bot_common::ErrorInfo& ret);

        void trajectoryFollowWorker(bot_common::ErrorInfo& ret);

        void interpolationWorker();

        void servoSendWorker();

        void directWorker();

        void extractStampedJointPath(std::vector<Eigen::VectorXd> &path);

        bot_common::ErrorInfo executeCurrentTrajectory(const bot_traj_planner::TrajectoryPtr &current_traj, double renew_time, bool consecutive);

        void boostWatchDog();

        void boostServo();
    };


    typedef std::shared_ptr<ArmServoMode> ArmServoModePtr;
}


#endif //DUAL_ARM_APP_SERVOMODE_HPP
