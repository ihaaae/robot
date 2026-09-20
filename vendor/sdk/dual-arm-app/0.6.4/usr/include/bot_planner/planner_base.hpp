#ifndef DUAL_ARM_APP_PLANNER_BASE_HPP
#define DUAL_ARM_APP_PLANNER_BASE_HPP

#include "log/log.h"
#include "bot_path_planner/path_planner.hpp"
#include "bot_traj_planner/trajectory_base.hpp"
#include <yaml-cpp/yaml.h>

namespace bot_planner {
    using namespace bot_traj_planner;

    enum class ValidatorType {
        MoveItFCL = 0,
        MoveItBullet = 1,
        RosService = 2
    };

    enum class KinematicsType {
        Screw = 0,
        MoveItNumerical = 1,
        MoveItAnalytical = 2
    };

    enum class TrajectoryType {
        TOTP = 0,
        IterativeSpline = 1,
    };

    class PlannerBase {
    public:
        explicit PlannerBase(const std::string &config_path);

        virtual ~PlannerBase();

    protected:
        bot_path_planner::PathPlannerPtr pp_;
        bot_kinematics::KinematicsPtr kin_;

        bot_validator::ValidatorPtr val_;
        double MaxVelocityFactor {1.0};
        double MaxAccelerationFactor {1.0};
        double MaxJerkFactor {1.0};
        std::string CONFIG_PATH;
        TrajectoryType TrajType;
        TrajectoryParametersPtr TrajParamPtr;
        std::vector<std::string> joint_names;
    public:
        /**
         *
         * @return Joint names get from yml file;
         */
        [[nodiscard]] const std::vector<std::string> &getJointNames() const;

        /**
         *
         * @param config_path The path of planner configuration
         * @return A unique pointer to the base class
         */
        static std::unique_ptr<PlannerBase> create(const std::string &config_path);

        /**
         *
         * @return The stored kinematics solver
         */
        const bot_kinematics::KinematicsPtr &getKinematicsPtr();

        /**
         *
         * @return The stored validator solver
         */
        const bot_validator::ValidatorPtr &getValidatorPtr();

        /**
         *
         * @return The stored path planner solver
         */
        const bot_path_planner::PathPlannerPtr &getPathPlannerPtr();

        /**
         * @brief A convenient function to get the nearest valid ik
         * @param query_pose The query pose
         * @param seeds The seed for optimization-based method
         * @param solution Output container: the nearest valid ik;
         * @return OK for success
         **/
        bot_common::ErrorInfo
        getNearestValidIK(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                          Eigen::VectorXd &solution);

        int
        i_IK(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                          Eigen::VectorXd &solution);

        bot_common::ErrorInfo
        getLeftValidIK(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution);//wyz
        bot_common::ErrorInfo
        getRightValidIK(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution);//wyz
        bot_common::ErrorInfo
        get5LeftValidIK(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution);//wyz
        Eigen::Matrix3d rodriguesRotationMatrix(const Eigen::Vector3d& k, double theta) ;

        /**
         * @brief solve joint-related problem with joint target
         * @param info Output information and error code
         * @param current_joints Current joint values
         * @param goal_joints Goal joint values
         * @return A unique pointer to a newly established trajectory
         */
        TrajectoryPtr solvePTP(bot_common::ErrorInfo &info, const Eigen::VectorXd &current_joints,
                                     const Eigen::VectorXd &goal_joints, bool checkCollision = false);

        /**
         * @brief solve joint-related problem with pose target (this pose is related to the robot base frame not world)
         * @param info Output information and error code
         * @param current_joints Current joint values
         * @param goal_pose Goal pose
         * @return A unique pointer to a newly established trajectory
         */
        TrajectoryPtr solvePTP(bot_common::ErrorInfo &info, const Eigen::VectorXd &current_joints,
                                     const Eigen::Isometry3d &goal_pose, bool checkCollision = false);

        /**
         * @brief To compute the best ik path corresponding to a cartesian path
         * @param info Output information and error code
         * @param pose_path The given cartesian path
         * @param seed The seed (usually the current joints)
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return A unique pointer to a newly established trajectory
         **/
        TrajectoryPtr
        solveCartesianPath(bot_common::ErrorInfo &info, const std::vector<Eigen::Isometry3d> &pose_path,
                           const Eigen::VectorXd &seed, double jump_threshold, double timeout_all,
                           bool checkCollision = false);

        /**
         * @brief solve pose-related problem. The function only provides linear path
         * @param info Output information and error code
         * @param current_joints Current joint values
         * @param goal_pose Goal pose (this pose is related to the robot base frame not world)
         * @param jump_threshold the max interval between two waypoints for the ik path
         * @return A unique pointer to a newly established trajectory
         */
        TrajectoryPtr
        solveLIN(bot_common::ErrorInfo &info, const Eigen::VectorXd &current_joints,
                 const Eigen::Isometry3d &goal_pose, const double &jump_threshold = 0.5, const double &timeout_all = 5,
                 bool checkCollision = true);
        /**
         * @brief solve pose-related problem. The function only provides linear path
         * @param info Output information and error code
         * @param current_joints Current joint values
         * @param goal_pose Goal pose (this pose is related to the robot base frame not world)
         * @param jump_threshold the max interval between two waypoints for the ik path
         * @return A unique pointer to a newly established trajectory
         */
        TrajectoryPtr
        solveLeftLIN(bot_common::ErrorInfo &info, const Eigen::VectorXd &current_joints,
                 const Eigen::Isometry3d &goal_pose, const double &jump_threshold = 0.5, const double &timeout_all = 5,
                 bool checkCollision = true);

                         /**
         * @brief solve pose-related problem. The function only provides linear path
         * @param info Output information and error code
         * @param current_joints Current joint values
         * @param goal_pose Goal pose (this pose is related to the robot base frame not world)
         * @param jump_threshold the max interval between two waypoints for the ik path
         * @return A unique pointer to a newly established trajectory
         */
        TrajectoryPtr
        solveRightLIN(bot_common::ErrorInfo &info, const Eigen::VectorXd &current_joints,
                 const Eigen::Isometry3d &goal_pose, const double &jump_threshold = 0.5, const double &timeout_all = 5,
                 bool checkCollision = true);
        /**
         * @brief solve pose-related problem. The function only provides linear path
         * @param info Output information and error code
         * @param current_joints Current joint values
         * @param goal_joints Goal joints
         * @param jump_threshold the max interval between two waypoints for the ik path
         * @return A unique pointer to a newly established trajectory
         */
        TrajectoryPtr
        solveLIN(bot_common::ErrorInfo &info, const Eigen::VectorXd &current_joints,
                 const Eigen::VectorXd &goal_joints, const double &jump_threshold = 0.5, const double &timeout_all = 5,
                 bool checkCollision = false);

        /**
         * @warning The rotation vector have to be orthogonal w.r.t the (first point - center)
         * @param path_query info Output information and error code
         * @param current_joints current_joints Current joint values
         * @param center the given circ center
         * @param rotation_vector The circ angle and rotation axis(positive in anti-clock direction), this vector is defined in the base frame
         * @return A unique pointer to a newly established trajectory
         */
        TrajectoryPtr
        solveCirc(bot_common::ErrorInfo &info, const Eigen::VectorXd &current_joints,
                  const Eigen::Vector3d &center, const Eigen::AngleAxisd &rotation_vector, double jump_threshold,
                  double timeout_all, bool checkCollision = false);


        /**
         * @brief To set max velocity factor, (0, 1.0]
         * @param value The value of max velocity factor
         */
        void setMaxVelocityFactor(double value);

        /**
         * @brief To get max velocity factor
         * @return The stored max velocity factor, if not set return default value
         */
        [[nodiscard]] double getMaxVelocityFactor() const;

        /**
         * @brief To set max acceleration factor, (0, 2.0]
         * @param value The value of max acceleration factor
         */
        void setMaxAccelerationFactor(double value);

        /**
         * @brief To get max acceleration factor
         * @return The stored max acceleration factor, if not set return default value
         */
        [[nodiscard]] double getMaxAccelerationFactor() const;

        /**
         * @brief To set max jerk factor, (0, 1.0]
         * @param value The value of max jerk factor
         */
        void setMaxJerkFactor(double value);

        /**
         * @brief To get max jerk factor
         * @return The stored max jerk factor, if not set return default value
         */
        [[nodiscard]] double getMaxJerkFactor() const;

        /**
         * @brief To set minimum configuration-space check distance. unit: rad
         * @param value The value of minimum configuration-space check distance
         */
        void setMinConfigCheckDistance(double value);

        /**
         * @brief To get minimum configuration-space check distance
         * @return The stored minimum configuration-space check distance, if not set return default value
         */
        [[nodiscard]] double getMinConfigCheckDistance() const;

        /**
         * @brief To set minimum cartesian-space translation distance. unit: m
         * @param value The value of minimum cartesian-space translation check distance.
         */
        void setMinCartesianTransDistance(double value);

        /**
         * @brief To get minimum cartesian-space translation distance
         * @return The stored minimum cartesian-space translation distance if not set return default value
         */
        [[nodiscard]] double getMinCartesianTransDistance() const;

        /**
         * @brief To set minimum cartesian-space angle distance. unit: rad
         * @param value The value of minimum cartesian angle check distance
         */
        void setMinCartesianAngleDistance(double value);

        /**
         * @brief To get minimum cartesian-space angle distance
         * @return The stored minimum cartesian-space angle distance if not set return default value
         */
        [[nodiscard]] double getMinCartesianAngleDistance() const;

        /**
       *
       * @param traj_query
       * @param path
       * @return
       */
        TrajectoryUniquePtr convertToTrajectory(bot_common::ErrorInfo &info, const std::vector<Eigen::VectorXd> &path,
                                                const std::vector<double> &optional_times = std::vector<double> {});


    protected:

        /**
         *
         * @return The validator creator, all parameters are stored inside a yml file
         */
        bot_validator::ValidatorUniquePtr createValidator(const YAML::Node &node);

        /**
         *
         * @return The kinematics creator, all parameters are stored inside a yml file
         */
        bot_kinematics::KinematicsUniquePtr createKinematics(const YAML::Node &node);

    };

    typedef std::shared_ptr<PlannerBase> PlannerPtr;
    typedef std::unique_ptr<PlannerBase> PlannerUniquePtr;
}

#endif //DUAL_ARM_APP_PLANNER_BASE_HPP
