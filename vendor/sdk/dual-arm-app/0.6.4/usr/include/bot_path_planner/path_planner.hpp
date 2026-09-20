#ifndef DUAL_ARM_APP_PATH_PLANNER_HPP
#define DUAL_ARM_APP_PATH_PLANNER_HPP

#include "bot_kinematics//kinematics_base.hpp"
#include "configuration_planner/config_planner_base.hpp"
#include "cartesian_planner/cartesian_planner_base.hpp"
#include "bot_kinematics/kinematics_screw.hpp"
namespace bot_kinematics {
    class KinematicsBase;
}
namespace bot_validator {
    class ValidatorBase;
}
namespace bot_path_planner {
    using KinematicsPtr = std::shared_ptr<bot_kinematics::KinematicsBase>;
    using ValidatorPtr = std::shared_ptr<bot_validator::ValidatorBase>;
    
    class ConfigPlannerBase;
    class CartesianPlannerBase;

    class PathPlanner {

    private:
   
        // bot_kinematics::KinematicsScrew* left_kinematics_screw_ = nullptr;

        // std::unique_ptr<bot_kinematics::KinematicsScrew> left_kinematics_;
        // std::unique_ptr<bot_kinematics::KinematicsScrew> right_kinematics_;  
        bot_kinematics::KinematicsScrew* getScrewKinematics() const;
    public:
        /**
         * Constructor
         * @param kinematics_ptr Interface of kinematics function
         * @param validator_ptr Interface of validation function
         */
        PathPlanner(const std::string &config_planner_name, const std::string &cartesian_planner_name,
                    bot_kinematics::KinematicsPtr kinematics_ptr, bot_validator::ValidatorPtr validator_ptr);
        // PathPlanner(bot_kinematics::KinematicsScrew* screw);
        virtual ~PathPlanner();

    protected:
        bot_kinematics::KinematicsPtr kin_;

        bot_validator::ValidatorPtr val_;

        bot_path_planner::ConfigPlannerUniquePtr jp_;

        bot_path_planner::CartesianPlannerUniquePtr cp_;

        double MinCartesianTransDistance {0.01};

        double MinCartesianAngleDistance {0.01};
    public:
        // void initializeFromYAML(const std::string& left_arm_yaml,
        //                        const std::string& right_arm_yaml);
        /**
         * @brief To plan a naive joint path, which linearly connects the start and end in joint space.
         * @param current_joints The current joint positions
         * @param goal_joints The goal joint positions
         * @param joint_path Output container
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        planNaiveJointPath(const Eigen::VectorXd &current_joints, const Eigen::VectorXd &goal_joints,
                           std::vector<Eigen::VectorXd> &joint_path, bool checkCollision);

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

        bot_common::ErrorInfo
        getLeftValidIK(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution);//wyz
        bot_common::ErrorInfo
        getRightValidIK(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution);//wyz
        bot_common::ErrorInfo
        get5LeftValidIK(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution);//wyz

        /**
         * @brief solve joint-related problem with joint target
         * @param path_query Output path
         * @param current_joints Current joint values
         * @param goal_joints Goal joint values
         * @param time_out The max allowed time
         * @return Success for solve successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo solvePTP(std::vector<Eigen::VectorXd> &path_query, const Eigen::VectorXd &current_joints,
                                       const Eigen::VectorXd &goal_joints, double time_out = 1., bool checkCollision=true);

        /**
         * @brief solve joint-related problem with pose target (this pose is related to the robot base frame not world)
         * @param path_query Output path
         * @param current_joints Current joint values
         * @param goal_pose Goal pose
         * @param time_out The max allowed time
         * @return Success for solve successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo solvePTP(std::vector<Eigen::VectorXd> &path_query, const Eigen::VectorXd &current_joints,
                                       const Eigen::Isometry3d &goal_pose, double time_out = 1., bool checkCollision=true);

        /**
         * @brief To compute the best ik path corresponding to a cartesian path
         * @param pose_path The given cartesian path
         * @param seed The seed (usually the current joints)
         * @param dst_joint_path Output container: a joint path corresponding to a cartesian path
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return Success for planning successfully, otherwise the enum why it fails
         **/
        virtual bot_common::ErrorInfo
        computeCartesianIKPath(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                               std::vector<Eigen::VectorXd> &dst_joint_path, double jump_threshold, double timeout_all, bool checkCollision);

        /**
         * @brief solve pose-related problem. The function only provides linear path
         * @param path_query Output path
         * @param current_joints Current joint values
         * @param goal_pose Goal pose (this pose is related to the robot base frame not world)
         * @param jump_threshold the max interval between two waypoints for the ik path
         * @return Success for solve successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        solveLIN(std::vector<Eigen::VectorXd> &path_query, const Eigen::VectorXd &current_joints,
                 const Eigen::Isometry3d &goal_pose, const double &jump_threshold = 0.2, const double &timeout_all = 5, bool checkCollision=true);


        /**
         * @brief solve pose-related problem. The function only provides linear path
         * @param path_query Output path
         * @param current_joints Current joint values
         * @param goal_pose Goal pose (this pose is related to the robot base frame not world)
         * @param jump_threshold the max interval between two waypoints for the ik path
         * @return Success for solve successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        solveLeftLIN(std::vector<Eigen::VectorXd> &path_query, const Eigen::VectorXd &current_joints,
                 const Eigen::Isometry3d &goal_pose, const double &jump_threshold = 0.2, const double &timeout_all = 5, bool checkCollision=true);

        /**
         * @brief solve pose-related problem. The function only provides linear path
         * @param path_query Output path
         * @param current_joints Current joint values
         * @param goal_pose Goal pose (this pose is related to the robot base frame not world)
         * @param jump_threshold the max interval between two waypoints for the ik path
         * @return Success for solve successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        solveRightLIN(std::vector<Eigen::VectorXd> &path_query, const Eigen::VectorXd &current_joints,
                 const Eigen::Isometry3d &goal_pose, const double &jump_threshold = 0.2, const double &timeout_all = 5, bool checkCollision=true);

        /**
         * @brief solve pose-related problem. The function only provides linear path
         * @param path_query Output path
         * @param current_joints Current joint values
         * @param goal_joints Goal joints
         * @param jump_threshold the max interval between two waypoints for the ik path
         * @return Success for solve successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        solveLIN(std::vector<Eigen::VectorXd> &path_query, const Eigen::VectorXd &current_joints,
                 const Eigen::VectorXd &goal_joints, const double &jump_threshold = 0.2, const double &timeout_all = 5, bool checkCollision=true);

        /**
         * @warning The rotation vector have to be orthogonal w.r.t the (first point - center)
         * @param path_query path_query Output path
         * @param current_joints current_joints Current joint values
         * @param center the given circ center
         * @param rotation_vector The circ angle and rotation axis(positive in anti-clock direction), this vector is defined in the base frame
         * @return Success for solve successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        solveCirc(std::vector<Eigen::VectorXd> &path_query, const Eigen::VectorXd &current_joints,
                  const Eigen::Vector3d &center, const Eigen::AngleAxisd &rotation_vector, double jump_threshold=0.2, double timeout_all=5, bool checkCollision=true);


        /**
         * @brief To plan a cartesian path connected start and end
         * @param current_joints The current joint values
         * @param goal_pose The goal pose
         * @param pose_path Output container: A path connects start and end
         * @param min_trans The min discrete distance in the translation dimension
         * @param min_angle The min discrete angle in the rotation dimension
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        computeLinePath(const Eigen::VectorXd &current_joints, const Eigen::Isometry3d &goal_pose,
                        std::vector<Eigen::Isometry3d> &pose_path, const double &min_trans, const double &min_angle);

        /**
         * @brief To plan a cartesian path connected start and end
         * @param current_pose The current pose corresponding to the current joint values
         * @param goal_pose The goal pose
         * @param pose_path Output container: A path connects start and end
         * @param min_trans The min discrete distance in the translation dimension
         * @param min_angle The min discrete angle in the rotation dimension
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        computeLinePath(const Eigen::Isometry3d &current_pose, const Eigen::Isometry3d &goal_pose,
                        std::vector<Eigen::Isometry3d> &pose_path, const double &min_trans, const double &min_angle);

        /**
         * @brief This function only consider the translation part to be in a circle, the orientation part is keep constant
         * @param current_joints The current joints
         * @param center the given circ center
         * @param rotation_vector The circ angle and rotation axis(positive in anti-clock direction), this vector is defined in the base frame
         * @param pose_path the output
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        computeCircPath(const Eigen::VectorXd &current_joints, const Eigen::Vector3d &center,
                        const Eigen::AngleAxisd &rotation_vector,
                        std::vector<Eigen::Isometry3d> &pose_path);


        /**
         * @brief This function only consider the translation part to be in a circle, the orientation part is keep constant
         * @param current_pose The current pose corresponding to the current joints
         * @param center the given circ center
         * @param rotation_vector The circ angle and rotation axis(positive in anti-clock direction), this vector is defined in the base frame
         * @param pose_path the output
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        computeCircPath(const Eigen::Isometry3d &current_pose, const Eigen::Vector3d &center,
                        const Eigen::AngleAxisd &rotation_vector,
                        std::vector<Eigen::Isometry3d> &pose_path);

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
        std::shared_ptr<bot_kinematics::KinematicsScrew> getKinematicsScrew() const;

    };

    typedef std::shared_ptr<PathPlanner> PathPlannerPtr;
    typedef std::unique_ptr<PathPlanner> PathPlannerUniquePtr;
}


#endif //DUAL_ARM_APP_PATH_PLANNER_HPP
