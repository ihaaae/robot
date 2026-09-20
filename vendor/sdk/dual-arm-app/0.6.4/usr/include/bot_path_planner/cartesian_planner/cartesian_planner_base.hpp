#ifndef DUAL_ARM_APP_CARTESIAN_PLANNER_BASE_HPP
#define DUAL_ARM_APP_CARTESIAN_PLANNER_BASE_HPP

#include "bot_validator/validator_base.h"
#include "bot_kinematics/kinematics_base.hpp"
#include <memory>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <chrono>

namespace bot_path_planner {


    class CartesianPlannerBase {
    public:
        CartesianPlannerBase(bot_validator::ValidatorPtr validatorPtr, bot_kinematics::KinematicsPtr kinematicsPtr);

        virtual ~CartesianPlannerBase();
    protected:
        double MinConfigCheckDistance {0.01};
        bot_validator::ValidatorPtr val_;
        bot_kinematics::KinematicsPtr kin_;
    public:
        /**
         * @brief To compute all iks corresponding to a cartesian path
         * warning： the collision check is not performed in this function and will be checked later
         * @param pose_path The given cartesian path
         * @param seed The seed usually the current joints
         * @param path_IKS Output container: The valid path iks;
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        computeAllIKPath(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                              std::vector<std::vector<Eigen::VectorXd>> &path_IKS, double max_dist);

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

        /**
         * @brief To compute the best ik path corresponding to a cartesian path
         * @param pose_path The given cartesian path
         * @param seed The seed (usually the current joints)
         * @param dst_joint_path Output container: a joint path corresponding to a cartesian path
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        virtual bot_common::ErrorInfo
        computeCartesianIKPath(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                               std::vector<Eigen::VectorXd> &dst_joint_path, double jump_threshold, double timeout_all, bool checkCollision);


        /**
         * @brief To compute the best ik path corresponding to a cartesian path
         * @param pose_path The given cartesian path
         * @param seed The seed (usually the current joints)
         * @param dst_joint_path Output container: a joint path corresponding to a cartesian path
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        virtual bot_common::ErrorInfo
        computeCartesianLeftIKPath(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                               std::vector<Eigen::VectorXd> &dst_joint_path, double jump_threshold, double timeout_all, bool checkCollision);
        /**
         * @brief To compute the best ik path corresponding to a cartesian path
         * @param pose_path The given cartesian path
         * @param seed The seed (usually the current joints)
         * @param dst_joint_path Output container: a joint path corresponding to a cartesian path
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        virtual bot_common::ErrorInfo
        computeCartesianRightIKPath(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                               std::vector<Eigen::VectorXd> &dst_joint_path, double jump_threshold, double timeout_all, bool checkCollision);

        /**
         * @brief To search a shortest path through all valid iks using greedy method
         * @param path_path All poses.
         * @param seed The seed for optimization method
         * @param dst Output container: The search result
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return Success for computing successfully, otherwise the enum why it fails
         *
         */
        bot_common::ErrorInfo
        greedySearch(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                     std::vector<Eigen::VectorXd> &dst, double jump_threshold, bool checkCollision);

        /**
         * @brief To search a shortest path through all valid iks using greedy method
         * @param path_path All poses.
         * @param seed The seed for optimization method
         * @param dst Output container: The search result
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return Success for computing successfully, otherwise the enum why it fails
         *
         */
        bot_common::ErrorInfo
        greedyLeftSearch(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                     std::vector<Eigen::VectorXd> &dst, double jump_threshold, bool checkCollision);
        /**
         * @brief To search a shortest path through all valid iks using greedy method
         * @param path_path All poses.
         * @param seed The seed for optimization method
         * @param dst Output container: The search result
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return Success for computing successfully, otherwise the enum why it fails
         *
         */
        bot_common::ErrorInfo
        greedyRightSearch(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                     std::vector<Eigen::VectorXd> &dst, double jump_threshold, bool checkCollision);

        /**
         * @brief To search a shortest path through all valid iks using backward-iteration method (This method is more robust but slower)
         * @param pose_path All poses.
         * @param seed The seed for optimization method
         * @param dst Output container: The search result
         * @param jump_threshold The threshold, when it is exceeded the joint path planning is failed
         * @return Success for computing successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo
        searchBestPath(const std::vector<Eigen::Isometry3d> &pose_path, const Eigen::VectorXd &seed,
                       std::vector<Eigen::VectorXd> &dst, double jump_threshold, double &timeout_all, bool checkCollision);

        void setMinConfigCheckDistance(const double &val);

        std::function<bot_common::ErrorInfo(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution)> call_back_leftIk;
        std::function<bot_common::ErrorInfo(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution)> call_back_rightIk;
        std::function<bot_common::ErrorInfo(const Eigen::Isometry3d &query_pose, const Eigen::VectorXd &current_joints,
                                   Eigen::VectorXd &solution)> call_back_5leftIk;

        [[nodiscard]] double getMinConfigCheckDistance() const;
    protected:
        bool wrap(Eigen::VectorXd& raw);
    };
    typedef std::shared_ptr<CartesianPlannerBase> CartesianPlannerPtr;
    typedef std::unique_ptr<CartesianPlannerBase> CartesianPlannerUniquePtr ;
}


#endif //DUAL_ARM_APP_CARTESIAN_PLANNER_BASE_HPP
