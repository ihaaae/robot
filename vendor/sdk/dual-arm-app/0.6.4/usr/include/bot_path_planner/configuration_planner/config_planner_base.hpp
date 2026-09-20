//
// Created by zyx on 23-8-27.
//

#ifndef DUAL_ARM_APP_CONFIG_PLANNER_BASE_HPP
#define DUAL_ARM_APP_CONFIG_PLANNER_BASE_HPP

#include "bot_validator/validator_base.h"
#include <memory>
#include <Eigen/Dense>
#include <vector>

namespace bot_path_planner {

    class ConfigPlannerBase {
    public:

        ConfigPlannerBase(bot_validator::ValidatorPtr validatorPtr);

        virtual ~ConfigPlannerBase();

    protected:
        double MinConfigCheckDistance {0.01};
        bot_validator::ValidatorPtr val_;
    public:
        /**
         * @brief To plan joint path for joint-space target
         * @param current_joints The current joint values
         * @param goal_joints The goal joint values
         * @param joint_path Output container, A vector of joint values
         * @param time_out The max allowed planning time
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        bot_common::ErrorInfo planJointPath(const Eigen::VectorXd &current_joints, const Eigen::VectorXd &goal_joints,
                                            std::vector<Eigen::VectorXd> &joint_path, double time_out, bool checkCollision);

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
         * @brief To plan a non-trivial considering collision avoiding (RRT-Connect)
         * @param current_joints THe current joint positions
         * @param goal_joints The goal joint positions
         * @param joint_path Output container
         * @param time_out The time allowed to plan
         * @return Success for planning successfully, otherwise the enum why it fails
         */
        virtual bot_common::ErrorInfo
        planNonTrivialJointPath(const Eigen::VectorXd &current_joints, const Eigen::VectorXd &goal_joints,
                                std::vector<Eigen::VectorXd> &joint_path, double time_out, bool checkCollision) ;

        void setMinConfigCheckDistance(const double &val);

        [[nodiscard]] double getMinConfigCheckDistance() const;
    };

    typedef std::shared_ptr<ConfigPlannerBase> ConfigPlannerPtr;
    typedef std::unique_ptr<ConfigPlannerBase> ConfigPlannerUniquePtr;
}


#endif //DUAL_ARM_APP_CONFIG_PLANNER_BASE_HPP
