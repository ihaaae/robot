#ifndef DUAL_ARM_APP_EXECUTORBASE_HPP
#define DUAL_ARM_APP_EXECUTORBASE_HPP

#include "bot_traj_planner/trajectory_base.hpp"
#include <array>

namespace bot_executor {
    typedef std::function<bot_traj_planner::TrajectoryUniquePtr(
            const bot_traj_planner::TrajectoryPtr &current_traj, double renew_time)> renew_fuc_t;

    class ExecutorBase {
    public:
        ExecutorBase() = default;

        virtual ~ExecutorBase() = default;

    public:
        /**
         * @brief To send joint position cmd
         * @param joint The given joint position cmds
         * @param lefthand_joints Left hand joints
         * @param lefthand_current Left hand current
         * @param follow True for high-follow mode
         * @return True for send right;
         */
        virtual bool sendAllServo(const Eigen::VectorXd &arm_joint,
                                  const Eigen::VectorXi &lefthand_joints,
                                  const Eigen::VectorXi &lefthand_current,
                                  const Eigen::VectorXi &righthand_joints,
                                  const Eigen::VectorXi &righthand_current) = 0;
        /**
         * @brief To send joint position cmd
         * @param joint The given joint position cmds
         * @param follow True for high-follow mode
         * @return True for send right;
         */
        virtual bool sendServo(const Eigen::VectorXd &arm_joint, int part) = 0;

        /**
         * @brief To send joint position cmd
         * @param joint The given joint position cmds
         * @param follow True for high-follow mode
         * @return True for send right;
         */
        virtual bool sendHandServo(const Eigen::VectorXi &lefthand_joints,
                                   const Eigen::VectorXi &lefthand_current,
                                   const Eigen::VectorXi &righthand_joints,
                                   const Eigen::VectorXi &righthand_current) = 0;

        /**
        * @brief To execute trajectory
        * @param traj Trajectory to be executed
        * @return Success for execute successfully, otherwise the enum why it fails.
        */
        virtual bot_common::ErrorInfo move(bot_traj_planner::TrajectoryPtr traj) = 0;


        virtual bot_common::ErrorInfo
        executeCurrentTrajectory(const bot_traj_planner::TrajectoryPtr &current_traj,
                                 double renew_time, bool consecutive) = 0;

        virtual bot_common::ErrorInfo waitForFinish() = 0;

        /**
         * @brief Fetch the current joint values
         * @return The current joint values
         * @throw invalidArgument when fetch failed
         */
        virtual Eigen::VectorXd getCurrentJointValues() = 0;


        virtual std::array<double, 14> getCurrentSingleTorques() = 0;

        /**
         * @brief Fetch the current error code
         * @return The current joint error code
         * @throw invalidArgument when fetch failed
         */
        virtual Eigen::VectorXf getCurrentJointError() = 0;
        
        /**
         * @brief Fetch the home joint values
         * @return The home joint values
         */
        virtual const Eigen::VectorXd &getHomeJointValues() = 0;
        

        virtual Eigen::Matrix<unsigned short int, 14, 1> getJointFault() = 0;
        /**
         *
         * @return True for robot is moving
         */
        virtual bool isMoving() = 0;

        /**
        *
        * @return True for robot is infault
        */
        virtual bool isInFault() = 0;

        /**
         *
         * @return True for connected, the simulation will always return true;
         */
        virtual bool isConnected() = 0;

        /**
         *
         * @return True for robot is Enabled
         */
        virtual bool isEnabled() = 0;

        virtual const std::vector<std::string>& getJointNames() = 0;

        virtual int getControlFrequency() = 0;

        virtual bool clearErrorsForJoint() = 0;

        virtual bool setEnableForJoint(bool enable) = 0;

        virtual bool disableServo() = 0;

        virtual int getDof() = 0;

        virtual bool setJointZeroPosition() = 0;

        virtual void ReadFault()=0;

        virtual int MoveEnd(double v,int part)=0;

        virtual int BreakEngage(const std::vector<int>& numbers)=0;

        virtual int BreakRelease(const std::vector<int>& numbers)=0;

        virtual void SetSending(bool is_send,int part){}
    };

    typedef std::shared_ptr<ExecutorBase> ExecutorPtr;
    typedef std::unique_ptr<ExecutorBase> ExecutorUniquePtr;
}


#endif //DUAL_ARM_APP_EXECUTORBASE_HPP
