#ifndef DUAL_ARM_APP_EXECUTORJUXIE_HPP
#define DUAL_ARM_APP_EXECUTORJUXIE_HPP
#include <map>
#include <thread>
#include <atomic>
#include "fmt/format.h"
#include "state/error_code.h"
#include "bot_executor/ExecutorBase.hpp"
#include "alg_factory/algorithm_factory.h"
#include "bot_subscriber/multi_thread_version.hpp"
#include "rk3576_can_canfd/rk3576_can_canfd.h"
#include <condition_variable>
#include "filter/joint_velocity_planner.hpp"
#include "bot_subscriber/multi_thread_version.hpp"
#include "thread_safe_container/thread_safe_deque.hpp"
//multithread declaration
template<class T>
class MultiThreadVersion;

namespace bot_executor {
    constexpr int ARM_DIM = 7;
    constexpr int HAND_DIM = 5;
    constexpr int HAND_CURRENT_DIM = 4;
    constexpr int HEAD_DIM = 3;
    constexpr int FULL_DIM = ARM_DIM * 2 + 2 * HAND_DIM + HEAD_DIM; //14 + 3 + 10;

    class JuxieState {
    public:
        uint8_t state_num {0};
        bool isMoving {true};
        bool isInitialized {false};
        bool isEnabled {false};
        bool isInFault {true};
        bool isReceivable {false};
        std::array<unsigned short int,14>motionState;
        std::array<u_int8_t,14>control_Type;
        std::array<unsigned short int,14>currentIntensity;
        std::array<u_int8_t,14>isbreak;
        std::array<int,14>isEnables;
        std::array<double,14>single_toeques;
        // now change to show the error code
        Eigen::Matrix<double, 17, 1> joint_states {Eigen::Matrix<double, 17, 1>::Zero()};
        Eigen::Matrix<unsigned short int, 14, 1> faultData {Eigen::Matrix<unsigned short int, 14, 1>::Zero()};
        bool updated {false};
    };

    constexpr char ExecutorJuxieName[] = "ExecutorJuxieName";

    class MotorCommands {
        public:
            MotorCommands();

            std::atomic_bool isAlreadySent {true};
            // std::atomic_bool isRightAlreadySent {true};

            std::mutex write_mutex;

            Eigen::VectorXd jointsPosition{7};      
            // Eigen::VectorXd jointsRightPosition{7};      

            int armIndex {0};     
    };

    class ExecutorJuxie : public ExecutorBase {
    public:
        explicit ExecutorJuxie(const std::string &config_path, const Eigen::Matrix<double, Eigen::Dynamic, 5> &jointMotionLimits);

        ~ExecutorJuxie() override;

    protected:
        int part_{0};

        double resample_delta {0.005}; //200hz

        MultiThreadVersion<JuxieState> current_state;

        Eigen::Matrix<double, FULL_DIM, 1> home_values {};

        std::atomic_bool keep_main_send0_alive {true};
        std::atomic_bool keep_main_send1_alive {true};
        std::atomic_bool keep_state_alive {true};
        std::atomic_bool keep_watchdog_alive {true};
        
        std::vector<std::string> joint_names;
        std::thread send_thread0, send_thread1;
        std::thread watchdog_thread;
        std::thread state_thread;
        ThreadSafeDeque<Eigen::VectorXd> motor0_commands;
        ThreadSafeDeque<Eigen::VectorXd> motor1_commands;
        rk3576_can_canfd::RK3576CanCanfdPtr RK3576CanCanfdPtr_;
        
        std::atomic_bool isLeftStop_{false};
        std::atomic_bool isRightStop_{false};

        std::atomic_bool isLeftSending_{false};
        std::atomic_bool isRightSending_{false};

        bool left_used_{true};
        bool right_used_{true};
        std::mutex read_mut {};
        std::mutex traj_mutex {};

        std::chrono::steady_clock::time_point m_updateTime;

        JointVelocityPlanner m_jointVelocityPlanner1;
        JointVelocityPlanner m_jointVelocityPlanner2;
        bool m_useLimit{true};
        bool m_isPrintfCan{false};
        int m_dof;
        std::array<double, 17> m_motor_direct;
        std::array<double, 17> m_soft_zero_position;
        std::array<int, 14> m_previousFaultData;
        bool m_use_soft_zero_position;
        // std::condition_variable cv;
        // int cv_part_{1};
    public:
        const std::vector<std::string> &getJointNames() override;

        bot_common::ErrorInfo waitForFinish() override;

        bot_common::ErrorInfo executeCurrentTrajectory(const bot_traj_planner::TrajectoryPtr &current_traj,
                                                       double renew_time, bool consecutive) override;

        /**
         * @brief To send joint position cmd
         * @param joint The given joint position cmds
         * @param follow True for high-follow mode
         * @return True for send right;
         */
        bool sendAllServo(const Eigen::VectorXd &arm_joint,
                          const Eigen::VectorXi &lefthand_joints,
                          const Eigen::VectorXi &lefthand_current,
                          const Eigen::VectorXi &righthand_joints,
                          const Eigen::VectorXi &righthand_current);

        /**
         * @brief To send joint position cmd
         * @param joint The given joint position cmds
         * @param follow True for high-follow mode
         * @return True for send right;
         */
        bool sendServo(const Eigen::VectorXd &arm_joint, int part);

        /**
         * @brief To send joint position cmd
         * @param joint The given joint position cmds
         * @param follow True for high-follow mode
         * @return True for send right;
         */
        bool sendHandServo(const Eigen::VectorXi &lefthand_joints,
                           const Eigen::VectorXi &lefthand_current,
                           const Eigen::VectorXi &righthand_joints,
                           const Eigen::VectorXi &righthand_current);

        // bool sendMaskedServo(const Eigen::VectorXd &joints, const Eigen::VectorXi &mask, const int &armIndex);

        /**
        * @ a convenient function to create
        * @param config_path path to executor parameters
        * @return A unique pointer to the base class
        */
        static ExecutorUniquePtr create(const std::string &config_path , const Eigen::Matrix<double, Eigen::Dynamic, 5> &jointMotionLimits);

        /**
        * @brief To execute trajectory
        * @param traj Trajectory to be executed
        * @return Success for execute successfully, otherwise the enum why it fails.
        */
        bot_common::ErrorInfo move(bot_traj_planner::TrajectoryPtr traj) override;

        /**
         * @brief Fetch the current joint values
         * @return The current joint values
         * @throw invalidArgument when fetch failed
         */
        Eigen::VectorXd getCurrentJointValues() override;

        std::array<double, 14> getCurrentSingleTorques() override;

        /**
         * @brief Fetch the current error code
         * @return The current joint error code
         * @throw invalidArgument when fetch failed
         */
        Eigen::VectorXf getCurrentJointError();

        // /**
        //  * @brief Fetch the current error code
        //  * @return The current joint error code
        //  * @throw invalidArgument when fetch failed
        //  */
        // Eigen::Matrix<int, 5, 1> getJointCurrent();
        
        Eigen::Matrix<unsigned short int, 14, 1> getJointFault()override;



        /**
         * @brief Fetch the home joint values
         * @return The home joint values
         */
        const Eigen::VectorXd &getHomeJointValues() override;

        /**
         *
         * @return True for robot is moving
         */
        bool isMoving() override;

        /**
         *
         * @return True for robot is moving
         */
        bool isInFault() override;

        /**
         *
         * @return True for connected, the simulation will always return true;
         */
        bool isConnected() override;

        /**
         *
         * @return True for robot is Enabled
         */
        bool isEnabled() override;

        /**
         * @brief To clear joint errors for the indexed joint
         * @param index The index ,start from 0 ~6
         * @return True for clear
         */
        bool clearErrorsForJoint() override;

        bool waitForFinishSignal(double expected_time);

        bool disableServo() override;

        /**
        *
        * @param enable The enable state vector of joints, true for enable
        * @return true for all right
        */
        bool setEnableForJoint(bool enable) override;

        int getState();

        const JuxieState &getCurrentJuxieState();

        // Eigen::VectorXi determineMask(const std::vector<std::string> &specific_names = {});

        int getControlFrequency() override;

        int getDof() override;

        bool setJointZeroPosition() override;

        void ReadFault() override;
        
        void SetSending(bool is_send,int part) override;

        int MoveEnd(double v,int part) override;

        int BreakEngage(const std::vector<int>& numbers) override;

        int BreakRelease(const std::vector<int>& numbers) override;

    protected:
        std::vector<int> checkJointsWithLimits(const Eigen::VectorXd& current_joints, const Eigen::Matrix<double, Eigen::Dynamic, 5> &jointMotionLimits);

        /**
         * @brief The thread function to deal with udp receive task
         * In this thread we set current joint positions and two indicators: isEnable, isInFault;
         */
        void sendCommandThread0();

        void sendCommandThread1();

        void *ucas_can1_task_send_thread(void *arg);

        void *ucas_can0_task_send_thread(void *arg);
        
        void listenStateThread();

        void watchdog();
    };

    inline bot_common::REGISTER_ALGORITHM(ExecutorBase, ExecutorJuxieName, ExecutorJuxie, const std::string&, const Eigen::Matrix<double, Eigen::Dynamic, 5>&);
    typedef std::shared_ptr<ExecutorJuxie> ExecutorJuxiePtr;
};


#endif //DUAL_ARM_APP_EXECUTORJUXIE_HPP
