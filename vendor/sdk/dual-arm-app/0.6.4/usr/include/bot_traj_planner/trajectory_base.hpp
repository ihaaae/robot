#ifndef DUAL_ARM_APP_TRAJECTORY_BASE_HPP
#define DUAL_ARM_APP_TRAJECTORY_BASE_HPP

#include <Eigen/Core>
#include <memory>
#include "state/error_code.h"
#include <vector>

namespace bot_traj_planner {

    struct TrajectoryParameters {
    public:
        TrajectoryParameters() = default;

        virtual ~TrajectoryParameters() = default;
    };

    typedef std::shared_ptr<TrajectoryParameters> TrajectoryParametersPtr;
    typedef std::unique_ptr<TrajectoryParameters> TrajectoryParametersUniquePtr;

    class TrajectoryBase {
    public:
        TrajectoryBase() = default;

        virtual ~TrajectoryBase() = default;

    public:
        void setNames(const std::vector<std::string>& names);

        const std::vector<std::string>& getNames();
        /**
         * @brief To pass constraints and convert a discrete path to a continue trajectory
         * @param origin_points Path waypoints
         * @param max_velocities Max velocity constraints
         * @param max_accelerations Max acceleration constraints
         * @param max_jerks Max jerk constraints
         * @return OK for success, or information why failed;
         */
        virtual bot_common::ErrorInfo
        init(const std::vector<Eigen::VectorXd> &path, const Eigen::VectorXd &max_velocities,
             const Eigen::VectorXd &max_accelerations,
             const Eigen::VectorXd &max_jerks) = 0;

        /**
         * @brief To get trajectory duration
         * @return Trajectory duration in seconds format
         * @throw CustomException when the trajectory is empty
        */
        [[nodiscard]]  virtual double getDuration() const = 0;

        /**
         * @brief To compute the position at a specific time point
         * @param sample_time The time point w.r.t. the start of the trajectory in seconds format
         * @return Position at time. Any time bigger than trajectory would return the last position.
         */
        [[nodiscard]] virtual Eigen::VectorXd computePositionAt(double sample_time) const = 0;

        /**
         * @brief To compute the velocity at a specific time point
         * @param sample_time The time point w.r.t. the start of the trajectory in seconds format
         * @return Velocity at time. Any time bigger than trajectory would return the last velocity.
         */
        [[nodiscard]] virtual Eigen::VectorXd computeVelocityAt(double sample_time) const = 0;

        /**
         * @brief To compute the acceleration at a specific time point
         * @param sample_time The time point w.r.t. the start of the trajectory in seconds format
         * @return Acceleration at time. Any time bigger than trajectory would return the last acceleration.
         */
        [[nodiscard]] virtual Eigen::VectorXd computeAccelerationAt(double sample_time) const = 0;

        /**
         * @brief To compute the jerk at a specific time point
         * @param sample_time The time point w.r.t. the start of the trajectory in seconds format
         * @return Jerk at time. Any time bigger than trajectory would return the last jerk.
         */
        [[nodiscard]] virtual Eigen::VectorXd computeJerkAt(double sample_time) const = 0;

        /**
         * @brief To test if the trajectory contains information
         * @return True for empty trajectory
         */
        [[nodiscard]] virtual bool empty() const = 0;

        /**
         * @brief set scale factor
         * @param val The given scale factor
         * @return True for set success, some type of trajectories cannot be accelerated
         */
        virtual void setScaleFactor(double val);

        /**
         * @brief get scale factor
         * @return The store scale factor;
         */
        [[nodiscard]] double getScaleFactor() const;

        /**
         * @brief synchronize two trajectories: The faster will be slow down to keep synchronized
         * @return
         */
        void synchronizeWithTrajectory(std::shared_ptr<TrajectoryBase> &traj2);

        void setPart(const int &index);
        const int &getPart();
    protected:
        double scale_factor {1.0};
        std::vector<std::string> names_{}; //in the order of the given trajectory;
        int part_ { 0 };
    };

    typedef std::shared_ptr<TrajectoryBase> TrajectoryPtr;
    typedef std::unique_ptr<TrajectoryBase> TrajectoryUniquePtr;
}
#endif //DUAL_ARM_APP_TRAJECTORY_BASE_HPP
