#ifndef DUAL_ARM_APP_SPLINE_BASE_HPP
#define DUAL_ARM_APP_SPLINE_BASE_HPP
#include <Eigen/Core>
#include <memory>
#include <map>
namespace bot_traj_planner {

    /**
     * base class for trajectory smoother request
     */
    class SplineRequest{
    public:
        SplineRequest() = default;

        virtual ~SplineRequest() = default;

    public:
        std::vector<double> overline_u_diff{}; // raw time stamps
        std::string param_option{"Centripetal"}; // self generated time stamps option
        std::vector<Eigen::VectorXd> initial_status{}; // initial velocity, acceleration, jerk, if needed
        std::vector<Eigen::VectorXd> end_status{}; // end velocity, acceleration, jerk, if needed
        std::string name{"Basic"};
        int free_flag{0};
        bool minimize_stretch{false};
    };
    typedef std::shared_ptr<SplineRequest> SplineRequestPtr;

    class SplineBase {
    public:
        SplineBase();

        virtual ~SplineBase() = default;
    protected:
        std::vector<Eigen::VectorXd> data_points{};

        std::vector<double> time_stamps{};

        std::map<std::string, std::function<void()>> _time_par_method_names;

        bool isInitialized{false};

    public:
        /**
         * @brief To init the b-spline smoother
         * @param path_data The discrete path waypoints
         * @param request Method parameters
         */
        virtual void init(const std::vector<Eigen::VectorXd> &path_data, SplineRequestPtr request) = 0;

        /**
         * @brief To compute derivatives
         * @param t_query The time point on which the derivatives are computed
         * @param degree The order of derivatives to be computed
         * @param u_handle The handle to convert t into u.
         * @return The degree-order derivatives
         */
        virtual Eigen::VectorXd evaluateDerivative(double t_query, int degree,
                                                    std::function<double(double)> u_handle) = 0;
        /**
         * @brief To get modifiable waypoints
         * @return A vector of the modifiable waypoints
         */
        std::vector<Eigen::VectorXd> &getWaypoints();

        /**
         * @brief To get unmodifiable waypoints
         * @return A vector of the unmodifiable waypoints
         */
        [[nodiscard]] const std::vector<Eigen::VectorXd> &getWaypoints() const;

        /**
         * @brief To get modifiable time stamps
         * @return A vector of the modifiable time stamps
         */
        std::vector<double> &getTimeStamps();

        /**
         * @brief To get unmodifiable timestamps
         * @return A vector of the unmodifiable time stamps
         */
        [[nodiscard]] const std::vector<double> &getTimeStamps() const;

        /**
         * @brief To get the total duration of the trajectory
         * @return The total duration of the trajectory in seconds format.
         * @throw CustomException if empty trajectory
         */
        [[nodiscard]] virtual double getDuration() const = 0;

        /**
         * @brief To re-init the whole trajectory with different method parameters but the same waypoints
         * @param request The method parameters
         */
        virtual void reInit(const SplineRequestPtr& request) = 0;
    protected:
        /**
         * @brief To compute a basic time stamp distribution using centripetal method. Generally this is the best method,
         *        especially for containing some sharp turns.
         */
        void centripetal();

        /**
         * @brief To compute a basic time stamp distribution using cord method
         */
        void cord();

        /**
         * @brief To compute a basic time stamp distribution using equally method.
         *        Generally this is the (velocity)fastest method
         */
        void equally();
    };

} // bot_traj_planner

#endif //DUAL_ARM_APP_SPLINE_BASE_HPP
