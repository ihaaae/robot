#ifndef DUAL_ARM_APP_VALIDATOR_VANILLA_HPP
#define DUAL_ARM_APP_VALIDATOR_VANILLA_HPP

#include "bot_validator/validator_base.h"
#include "alg_factory/algorithm_factory.h"

namespace bot_validator {
    constexpr char ValidatorVanillaName[] = "ValidatorVanilla";


    class ValidatorVanilla : public ValidatorBase {
    public:
        explicit ValidatorVanilla(std::string config_path);

        ~ValidatorVanilla() override;

    public:
    protected:
        Eigen::Matrix<double, -1, 5> JointMotionLimits;
    public:

        /**
         * @brief To check is one single joint waypoint is in collision or not
         * @param joint_values The joint values to be checked
         * @return ErrorCode::InCollision for in collision, otherwise ErrorCode::OK
         */
        bot_common::ErrorInfo checkCollision(const Eigen::VectorXd &joint_values) override;

        /**
         * @brief To check if joint status is inside limitation range or not
         * @param joint_values The joint values to be checked
         * @param velocity_values The joint velocities to be checked
         * @param acceleration_values The acceleration values to be checked
         * @return ErrorCode::OutLimitation for out of limitation, otherwise ErrorCode::OK
         */
        bot_common::ErrorInfo
        checkLimitation(const Eigen::VectorXd &joint_values, const Eigen::VectorXd &velocity_values,
                        const Eigen::VectorXd &acceleration_values) override;


        /**
         * @brief To modify ACM matrix (add now)
         * @param allowed_pairs The new collision-allowed pairs to be added
         */
        void
        modifyCollisionAllowedMatrix(const std::vector<std::pair<std::string, std::string>> &allowed_pairs,
                                     bool enableCollision) override;

        /**
         * @ a convenient function to create
         * @param planning_group the planning group that used to check collision
         * @param type The collision checker type, in default we use fcl
         * @return A unique pointer to the base class
         */
        static ValidatorUniquePtr
        create(std::string config_path);


        /**
         *
         * @return planning group name specified
         */
        const std::string &getPlanningGroupName() override;

    };

    inline bot_common::REGISTER_ALGORITHM(ValidatorBase, ValidatorVanillaName, ValidatorVanilla, std::string);
}


#endif //DUAL_ARM_APP_VALIDATOR_VANILLA_HPP
