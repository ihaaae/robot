#ifndef DUAL_ARM_APP_JUXIE_CONTROLLER_H
#define DUAL_ARM_APP_JUXIE_CONTROLLER_H

#include <array>
#include <memory>
#include <vector>
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/Core>
namespace Juxie {
    /**
     * When a joint is set as this value, the current actual position will be used.
     */
    constexpr double DEFAULT_INVALID_VALUE = -100.;
    /**
     * Format: waist * 1, left * 7, right * 7, head * 2
     * Unit: rad
     * Special Format: Set as DEFAULT_INVALID_VALUE to maintain the current joint position
     * Example:
     * 1. {1., ..., 1.}
     *
     * 2. {-100.,1., ..., 1., -100. * 9} Only left arm move
     */
    typedef std::array<double, 17> JointSpaceData;

    /**
     * Format: left * 7, right * 7
     * Unit: rad
     * Special Format: Set as DEFAULT_INVALID_VALUE to maintain the current joint position
     * Example:
     * 1. {1., ..., 1.}
     *
     * 2. {1., ..., 1., -100. * 7} Only left arm move
     */
    typedef std::array<double, 14> ArmSpaceData;

    /**
     * Format: left TCP (x, y, z, qw, qx, qy, qz) + right TCP (x, y, z, qw, qx, qy, qz);
     * Unit: m for translation
     * Special Format: Set any value of TCP data as DEFAULT_INVALID_VALUE to disable the corresponding tcp
     * Example:
     * 1. {1., ..., 1., 0., 0., 0.}
     *
     * 2. {-100.,1., ..., 1., 0., 0., 0.} Only right arm move
     */
    typedef std::array<double, 14> CartesianSpaceData;

    /**
     * Format: left TCP (x, y, z, rx, ry, rz) + right TCP (x, y, z, rx, ry, rz);
     * Unit: m for translation
     * Special Format: Set any value of TCP data as DEFAULT_INVALID_VALUE to disable the corresponding tcp
     * Example:
     * 1. {1., ..., 1., 0., 0., 0.}
     *
     * 2. {-100.,1., ..., 1., 0., 0., 0.} Only right arm move
     */
    typedef std::array<double, 12> CartesianSpaceDataRPY;

    struct Config
    {
        std::array<double, 14> upper;
        std::array<double, 14> lower;
    };

    class ControllerJuxieImpl;

    class ControllerJuxie {
    public:
        explicit ControllerJuxie();

        ~ControllerJuxie() = default;
    public:

        /**
         * @brief Switch Robot state to ready.
         * @return True for robot actually ready, otherwise false
         */
        bool OnRobot();

        /**
         * @brief Switch Robot state to Power_off and shutdown the robot.
         * @return True for successfully shutdown the low-level control board, otherwise false (like connection lost)
         */
        bool OffRobot();

        /**
         * @brief Switch Robot state from ready to idle
         * @return True for robot actually idle, otherwise false
         */
        bool EnableRobot();

        /**
         * @brief Switch Robot state from idle to ready
         * @return True for robot actually ready, otherwise false
         */
        bool DisableRobot();

        /**
         * @brief The robot will stop all actions of robot when robot is running, and switch robot state from running to idle.
         * @return True for robot actually stopped, otherwise false
         */
        bool Stop();

        /**
         * @brief Clear existing errors, if there are some, and switch robot state from fault to idle/ready
         * @return True for successfully cleared
         */
        bool ClearFault();

        /**
         * @brief Get current robot state
         * @return enum representing robot state [running,idle,ready,fault,power_off]
         */
        int GetRobotState();

        /**
         * @brief Get the corresponding information for one error code
         * @param error_code enum representing failed reasons
         * @return Specific reason for one error code
         */
        std::string GetFaultType(int error_code);

        /**
         * @brief Get current joint data; Currently only 17 motors are considered, two hands should call @GetHandPositionJoints
         * @return Joint positions of 17 motors
         */
        JointSpaceData GetJointPositions();


        ArmSpaceData GetSingleTorques();

        /**
         * @brief Get current tcp pose
         * @Warning The TCP pose of each arm is defined with respect to the arm base (Not the world frame)
         * @return TCP poses of two arms
         */
        CartesianSpaceData GetTCPPose();
        
        /**
         * @brief Get tcp pose
         * @Warning The TCP pose of each arm is defined with respect to the arm base (Not the world frame)
         * @return TCP poses of two arms
         */
        CartesianSpaceDataRPY getFKpose(const std::vector<double>& joint_angles,
                            const  int left_num, 
                            const int right_num);



        /**
         * @brief get all of joint
         * @return all of joint
         */
        int getDof();
        
        /**
         * @brief Set Joint ZeroPosition
         * @return True for successfully cleared
         */
        bool setJointZeroPosition();

        Config getConfig();        
        /**
         * @brief Online motion generator, the joint positions will be given online in 10~50hz
         * @param joint_position Joint positions data; Currently, only 17 motors are considered, two hands should be controlled through @MoveHand
         * @param freq The idea online discrete commands frequency
         * @return The enum representing why the @MoveJ_Canfd function failed (0 for success)
         */
        int
        MoveJ_Canfd(const JointSpaceData &joint_position, int freq);


        /**
         * @brief Online motion generator, the pose positions will be given online in 10~50hz
         * @param joint_position pose positions data; Currently, only 17 motors are considered, two hands should be controlled through @MoveHand
         * @param freq The idea online discrete commands frequency
         * @return The enum representing why the @MoveJ_Canfd function failed (0 for success)
         */
        int 
        MoveP_Canfd(const Juxie::CartesianSpaceData &pose,int freq);

        
        /**
         * @brief Offline motion generator, this function will block until the robot finished moving
         * @param joint_position Joint positions data; Currently, only 17 motors are considered, two hands should be controlled through @MoveHand
         * @param v velocity factor; Must be positive
         * @return The enum representing why the @MoveJ function failed (0 for success)
         */
        int
        MoveJ(const JointSpaceData &joint_position, int v);

        /**
         * @brief Offline motion generator, this function will block until the robot finished moving
         * @param tcp_pose Cartesian pose data;
        * @param v velocity factor; Must be positive
         * @return The enum representing why the @MoveJ_P function failed (0 for success)
         * @Warning The TCP pose of each arm is defined with respect to the arm base (Not the world frame)
         */
        int
        MoveJ_P(const CartesianSpaceData &tcp_pose, int v);

        /**
         * @brief Offline cartesian motion generator, only the last call finished the next one will be processed.
         * @param tcp_pose Cartesian pose data;
         * @param v velocity factor; Must be positive
         * constraint while reaching this configuration as close as possible.
         * @return The enum representing why the @MoveL function failed (0 for success)
         * @Warning The TCP pose of each arm is defined with respect to the arm base (Not the world frame)
         */
        int
        MoveL(const CartesianSpaceData &tcp_pose, int v);


        int
        MoveEnd(double v,int part);

        int
        BreakEngage(const std::vector<int>&numbers);

        int
        BreakRelease(const std::vector<int>&numbers);
        /**
         * @brief Offline cartesian motion generator, only the last call finished the next one will be processed.
         * @param tcp_pose Cartesian pose data;
         * constraint while reaching this configuration as close as possible.
         * @return The enum representing why the @IK function failed (0 for success)
         * @Warning The TCP pose of each arm is defined with respect to the arm base (Not the world frame)
         */
        int
        IK(const CartesianSpaceData &tcp_pose,Eigen::VectorXd&  joint);

        std::string GetAxisFault(const unsigned short int erro_code);

        Eigen::Matrix<unsigned short int, 14, 1> getJointerrcode();
    private:
        std::shared_ptr<ControllerJuxieImpl> impl_;
    public:
        std::atomic<bool> joint_stream_enable_{false};
        std::atomic<int> joint_stream_freq_{100};
    };


}


#endif //DUAL_ARM_APP_JUXIE_CONTROLLER_H
