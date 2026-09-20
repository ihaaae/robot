#ifndef DUAL_ARM_APP_SPLINE_POLYNOMINAL_HPP
#define DUAL_ARM_APP_SPLINE_POLYNOMINAL_HPP
/**
 * only cubic spline is considered now
 */
#include "spline_base.hpp"

#include "alg_factory/algorithm_factory.h"

class CubicCurve;
namespace cubic_spline {
    class CubicSplineBase;

    typedef std::shared_ptr<CubicSplineBase> CubicSplinePtr;
}

namespace bot_traj_planner {

    constexpr char SplinePolynomialName[] = "TrajectoryPolynomial";

    class SplinePolynomial : public SplineBase {
    public:
        SplinePolynomial() = default;

        ~SplinePolynomial() override = default;

    public:

        /**
        * @brief To init the b-spline smoother
        * @param path_data The discrete path waypoints
        * @param request Method parameters
        */
        void init(const std::vector<Eigen::VectorXd> &path_data, SplineRequestPtr request) override;

        /**
         * @brief To compute derivatives
         * @param t_query The time point on which the derivatives are computed
         * @param degree The order of derivatives to be computed
         * @param u_handle The handle to convert t into u.
         * @return The degree-order derivatives
         */
        Eigen::VectorXd evaluateDerivative(double t_query, int degree,
                                           std::function<double(double)> u_handle) override;


        /**
         * @brief To get the total duration of the trajectory
         * @return The total duration of the trajectory in seconds format.
         * @throw CustomException if empty trajectory
         */
        [[nodiscard]] double getDuration() const override;

        /**
         * @brief To re-init the whole trajectory with different method parameters but the same waypoints
         * @param request The method parameters
         */
        void reInit(const SplineRequestPtr &request) override;

    protected:
        cubic_spline::CubicSplinePtr spline;
        std::shared_ptr<CubicCurve> curve;
    };

}


#endif //DUAL_ARM_APP_SPLINE_POLYNOMINAL_HPP
