#ifndef DUAL_ARM_APP_TRAJECTORY_TIME_OPTIMAL_HPP
#define DUAL_ARM_APP_TRAJECTORY_TIME_OPTIMAL_HPP

#include "bot_traj_planner/trajectory_base.hpp"
#include "alg_factory/algorithm_factory.h"

namespace bot_traj_planner {
    class TOTPImpl;

    typedef std::shared_ptr<TOTPImpl> TOTPImplPtr;

    constexpr char TrajectoryTimeOptimalName[] = "TrajectoryTimeOptimal";

    struct TOTPParameters : public TrajectoryParameters {
    public:
        explicit TOTPParameters(const double &path_tolerance_ = 0.1,
                                const double min_angle_change_ = 0.001) : TrajectoryParameters(),
                                                                          path_tolerance(path_tolerance_),
                                                                          min_angle_change(min_angle_change_) {

        };

        double path_tolerance;
        double min_angle_change;
    };

    class TrajectoryTOTP : public TrajectoryBase {
    public:
        /**
         * Constructor
         * @param path_tolerance The tolerance that may the fitted path exceed the original path
         * @param min_angle_change The minimum joint angle change to identify two set of joint values as the same one
         */
        explicit TrajectoryTOTP(double path_tolerance = 0.1,
                                double min_angle_change = 0.001);

        ~TrajectoryTOTP() override = default;

    public:
        /**
         * @brief a convenient function to create empty trajectory
         * @param path_tolerance A tolerance to approximate the original discrete path, the bigger the smoother with a risk of unexpected collision
         * @param min_angle_change A tolerance to delete duplicate waypoint
         * @return A unique pointer to the base class
         */
        static TrajectoryUniquePtr create(double path_tolerance = 0.1, double min_angle_change = 0.001);

        /**
         * @brief To pass constraints and convert a discrete path to a continue trajectory
         * @param path Path waypoints
         * @param max_velocities Max velocity constraints
         * @param max_accelerations Max acceleration constraints
         * @param max_jerks Max jerk constraints
         * @return OK for success, or information why failed;
         */
        bot_common::ErrorInfo init(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &max_velocities,
                                   const Eigen::VectorXd &max_accelerations, const Eigen::VectorXd &max_jerks) override;

        /**
         * @brief To get trajectory duration
         * @return Trajectory duration in seconds format
         * @throw CustomException when the trajectory is empty
        */
        double getDuration() const override;

        /**
         * @brief To compute the position at a specific time point
         * @param sample_time The time point w.r.t. the start of the trajectory in seconds format
         * @return Position at time. Any time bigger than trajectory would return the last position.
         */
        Eigen::VectorXd computePositionAt(double sample_time) const override;


        /**
         * @brief To compute the velocity at a specific time point
         * @param sample_time The time point w.r.t. the start of the trajectory in seconds format
         * @return Velocity at time. Any time bigger than trajectory would return the last velocity.
         */
        Eigen::VectorXd computeVelocityAt(double sample_time) const override;

        /**
         * @brief To compute the acceleration at a specific time point
         * @param sample_time The time point w.r.t. the start of the trajectory in seconds format
         * @return Acceleration at time. Any time bigger than trajectory would return the last acceleration.
         */
        Eigen::VectorXd computeAccelerationAt(double sample_time) const override;

        /**
        * @brief To compute the jerk at a specific time point
        * @param sample_time The time point w.r.t. the start of the trajectory in seconds format
        * @return Jerk at time. Any time bigger than trajectory would return the last jerk.
        */
        [[nodiscard]] Eigen::VectorXd computeJerkAt(double sample_time) const override;


        /**
         * @brief To test if the trajectory contains information
         * @return True for empty trajectory
         */
        bool empty() const override;

        /**
         * @brief set scale factor
         * @param val The given scale factor
         * @return True for set success, some type of trajectories cannot be accelerated
         */
        void setScaleFactor(double val) override;

    protected:
        mutable TOTPImplPtr m_trajectory;
        mutable bool is_initialized {false};
        const double path_tolerance_;
        const double min_angle_change_;
        mutable bool is_only_one_point {false};
        mutable Eigen::VectorXd start_point;
        mutable int point_size {};
    };

    inline bot_common::REGISTER_ALGORITHM(TrajectoryBase, TrajectoryTimeOptimalName, TrajectoryTOTP, double, double);
}


#endif //DUAL_ARM_APP_TRAJECTORY_TIME_OPTIMAL_HPP
