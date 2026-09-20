#ifndef DUAL_ARM_APP_TRAJECTORY_ITERATIVE_HPP
#define DUAL_ARM_APP_TRAJECTORY_ITERATIVE_HPP

#include "bot_traj_planner/trajectory_base.hpp"
#include "alg_factory/algorithm_factory.h"
#include "bot_traj_planner/spline_base.hpp"

namespace bot_traj_planner {
    constexpr char TrajectoryIterativeName[] = "TrajectoryIterative";

    enum class SplineType {
        Polynomial = 0,
        BSpline = 1,
        Nurbs = 2,
        Bezier = 3,
    };

    struct IterativeSplineParameters : public TrajectoryParameters {
    public:
        explicit IterativeSplineParameters(bool constrained_acc_ = true, bool enable_jerk_ = false,
                                           bool min_stretch = true,
                                           const double &min_angle_change_ = 0.001,
                                           const SplineType &type_ = SplineType::Polynomial)
                : TrajectoryParameters(), constrained_acc(constrained_acc_), enable_jerk(enable_jerk_),
                  m_stretch(min_stretch),
                  min_angle_change(min_angle_change_), type(type_) {

        };
        bool constrained_acc;
        bool enable_jerk;
        bool m_stretch;
        double min_angle_change;
        SplineType type;
    };

    class TrajectoryIterativeSpline : public TrajectoryBase {
    public:
        explicit TrajectoryIterativeSpline(bool constrained_acc = true, bool enable_jerk = false, bool m_stretch = true,
                                           double min_angle_change = 0.001,
                                           const SplineType &type = SplineType::Polynomial);

        ~TrajectoryIterativeSpline() override = default;

    public:
        /**
         * @brief a convenient function to create empty trajectory
         * @param constrained_acc True for adopt zero initial and end accelerations
         * @param enable_jerk True for enable jerk computation( @warning not implemented yet)
         * @param min_angle_change A tolerance to delete duplicate waypoint
         * @param type The enum to specify spline used
         * @return A unique pointer to the base class
         */
        static TrajectoryUniquePtr
        create(bool constrained_acc = true, bool enable_jerk = false, bool m_stretch = true,
               double min_angle_change = 0.001,
               const SplineType &type = SplineType::Polynomial);

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
         * @brief To pass constraints and convert a discrete path to a continue trajectory
         * @param path Path waypoints
         * @param initial_status initial status may be velocity or acceleration or more
         * @param end_status end status may be velocity or acceleration or more
         * @param max_velocities Max velocity constraints
         * @param max_accelerations Max acceleration constraints
         * @param max_jerks Max jerk constraints
         * @return OK for success, or information why failed;
         */
        bot_common::ErrorInfo
        init(const std::vector<Eigen::VectorXd> &path, const std::vector<Eigen::VectorXd> &initial_status,
             const std::vector<Eigen::VectorXd>& end_status,
             const Eigen::VectorXd &max_velocities,
             const Eigen::VectorXd &max_accelerations, const Eigen::VectorXd &max_jerks, bool enable_limit_constraint = true);


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
        Eigen::VectorXd computeJerkAt(double sample_time) const override;

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

        const std::unique_ptr<SplineBase>& getImpl();

    protected:

        /**
         * To get the factor between the obtained raw data and the limits
         * @param raw The obtained raw data
         * @param upper_limit The given upper limit
         * @param lower_limit The given lower limit
         * @param order should be 1 for velocity, 2 for acceleration, or 3 for jerk
         * @return The factor
         */
        static double
        getFactor(const Eigen::VectorXd &raw, const Eigen::VectorXd &upper_limit, const Eigen::VectorXd &lower_limit,
                  int order);

    protected:
        std::unique_ptr<SplineBase> impl_;

        const bool jerk_enabled_;/// @brief If true, enable jerk and initial/final acceleration matching
        const bool constrained_acc_;
        const bool m_stretch_;

        const double min_angle_change_;

        mutable bool is_initialized {false};

        mutable bool is_only_one_point {false};

        mutable Eigen::VectorXd start_point;

        mutable int point_size {0};

        SplineRequestPtr requestPtr;
    };

    inline bot_common::REGISTER_ALGORITHM(TrajectoryBase, TrajectoryIterativeName, TrajectoryIterativeSpline, bool,
                                          bool, bool,
                                          double, const SplineType&);

} // bot_traj_planner

#endif //DUAL_ARM_APP_TRAJECTORY_ITERATIVE_HPP
