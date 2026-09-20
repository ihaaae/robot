#ifndef GCOPTER_ROBOT_HPP
#define GCOPTER_ROBOT_HPP

#include "SE3.hpp"

namespace robot {
    using SE3::TMat;
    typedef Eigen::Matrix<double, 6, -1> ScrewList;
    typedef Eigen::Matrix<double, 6, -1> Jacobian;
    typedef Eigen::Matrix<double, -1, 1> ThetaList;
    typedef Eigen::Matrix<double, 1, -1> RowThetaList;
    typedef std::vector<Eigen::Matrix<double, 6, -1>> Hessian; ///< Every element denotes that \f(dJ / dq_i\f)
    typedef std::vector<Eigen::Matrix<double, 3, 3>> RJacobian; ///< Every element denotes that \f(dR / dq_i\f)
    typedef std::vector<Eigen::Matrix<double, 4, 4>> TJacobian; ///< Every element denotes that \f(dT / dq_i\f)

    typedef TMat (*lfk_t)(const ThetaList &thetaList);

    typedef std::function<TMat(const ThetaList &thetaList)> lfk_func;

    typedef std::function<RowThetaList(double &H, const ThetaList &thetaList, const Jacobian &J)> gradient_func;

    template<typename T>
    struct matrix_hash : std::unary_function<T, size_t> {
        std::size_t operator()(T const &matrix) const {
            // Note that it is oblivious to the storage order of Eigen matrix (column- or
            // row-major). It will give you the same hash value for two different matrices if they
            // are the transpose of each other in different storage order.
            size_t seed = 0;
            for (long i = 0; i < matrix.size(); ++i) {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };


    /**
     * Computes forward kinematics in the space frame for an open chain robot
     * @param M The home configuration (position and orientation) of the end-effector
     * @param SList The joint screw axes in the space frame when the manipulator is at the home position,
     * in the format of a matrix with axes as the columns
     * @param thetaList A list of joint coordinates
     * @return A homogeneous transformation matrix representing the end-
     *        effector frame when the joints are at the specified coordinates
     *        (i.t.o Space Frame)
     */
    inline static TMat fkInSpace(const TMat &M, const ScrewList &SList, const ThetaList &thetaList);

    /**
     * Computes forward kinematics in the body frame for an open chain robot
     * @param M The home configuration (position and orientation) of the end-
              effector
     * @param BList The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
     * @param thetaList  A list of joint coordinates
     * @return  A homogeneous transformation matrix representing the end-
             effector frame when the joints are at the specified coordinates
             (i.t.o Body Frame)
     */
    inline static TMat fkInBody(const TMat &M, const ScrewList &BList, const ThetaList &thetaList);

    /**
     * Computes the body Jacobian for an open chain robot
     * @param BList The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
     * @param thetaList A list of joint coordinates
     * @return The body Jacobian corresponding to the inputs (6xn real
             numbers)
     */
    inline static Jacobian jacobianBody(const ScrewList &BList, const ThetaList &thetaList);

    /**
     *
     * Computes the space Jacobian for an open chain robot
     * @param BList The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
     * @param thetaList A list of joint coordinates
     * @param ret The output jacobian
     * @return The space Jacobian corresponding to the inputs (6xn real
             numbers)
     */
    inline static Jacobian jacobianBodyInPlace(const ScrewList &BList, const ThetaList &thetaList, Jacobian &ret);

    /**
     * Computes the space Jacobian for an open chain robot
     * @param SList The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
     * @param thetaList A list of joint coordinates
     * @return The space Jacobian corresponding to the inputs (6xn real
             numbers)
     */
    inline static Jacobian jacobianSpace(const ScrewList &SList, const ThetaList &thetaList);

    /**
     *
     * Computes the space Jacobian for an open chain robot
     * @param SList The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
     * @param thetaList A list of joint coordinates
     * @param ret The output jacobian
     * @return The space Jacobian corresponding to the inputs (6xn real
             numbers)
     */
    inline static void jacobianSpaceInPlace(const ScrewList &SList, const ThetaList &thetaList, Jacobian &ret);

    /**
     * Convert a MR-form space jacobian to a norm form jacobian
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param space_jacobian jacobian in MR form
     * @param in_ee True for change into the end frame
     * @return Norm form jacobian
     */
    inline static Jacobian fromMRSpaceJacobian(const TMat &T, const Jacobian &space_jacobian, bool in_ee = false);

    /**
     * Inplace version of \fromMRSpaceJacobian
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param space_jacobian jacobian in MR form
     * @param in_ee True for change into the end frame
     */
    inline static void fromMRSpaceJacobianInPlace(const TMat &T, Jacobian &space_jacobian, bool in_ee = false);

    /**
     * Convert a MR-form space jacobian to a norm form jacobian
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param body_jacobian jacobian in MR form
     * @param in_ee True for change into the end frame
     * @return Norm form jacobian
     */
    inline static Jacobian fromMRBodyJacobian(const TMat &T, const Jacobian &body_jacobian, bool in_ee = true);

    /**
     * Inplace version of \fromMRBodyJacobian
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param space_jacobian jacobian in MR form
     * @param in_ee True for change into the end frame
     */
    inline static void fromMRBodyJacobianInPlace(const TMat &T, Jacobian &body_jacobian, bool in_ee = true);

    /**
     * Convert a Norm-form space jacobian to a MR form jacobian
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param space_jacobian jacobian in MR form
     * @param in_ee True for change into the end frame
     * @return MR form jacobian
     */
    inline static Jacobian fromNormSpaceJacobian(const TMat &T, const Jacobian &space_jacobian, bool in_ee = false);

    /**
     * Inplace version of \fromNormSpaceJacobian
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param space_jacobian jacobian in MR form
     * @param in_ee True for change into the end frame
     * @return MR form jacobian
     */
    inline static void fromNormSpaceJacobianInPlace(const TMat &T, Jacobian &space_jacobian, bool in_ee = false) ;

    /**
     * Convert a Norm-form space jacobian to a MR form jacobian
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param body_jacobian jacobian in MR form
     * @param in_ee True for change into the end frame
     * @return MR form jacobian
     */
    inline static Jacobian fromNormBodyJacobian(const TMat &T, const Jacobian &body_jacobian, bool in_ee = false) ;

    /**
     * Inplace version of \fromNormBodyJacobian
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param body_jacobian jacobian in MR form
     * @param in_ee True for change into the end frame
    */
    inline static void fromNormBodyJacobianInPlace(const TMat &T, Jacobian &body_jacobian, bool in_ee = false) ;

    /**
     * Compute Screw list through fk function
     * @param fk_function function handle of forward kinematics
     * @param SList Output Screw list
     * @param body True for computing screw w.r.t. body
     * @param joint_numbers the total joint numbers
     * @return Initial condition
     */
    inline static TMat
    computeScrewList(lfk_t fk_function, ScrewList &SList, bool body = false, const int joint_numbers = 7);

    /**
     * Compute Screw list through fk function
     * @param fk_function function handle of forward kinematics
     * @param SList Output Screw list
     * @param body True for computing screw w.r.t. body
     * @param joint_numbers the total joint numbers
     * @return Initial condition
     */
    inline static TMat
    computeScrewList(lfk_func fk_function, ScrewList &SList, bool body = false, const int joint_numbers = 7);

    /**
     * Compute analytical jacobian d([r, x])/d(q)
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param BList The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
     * @param thetaList  A list of joint coordinates
     * @return analytical jacobian d([r, x])/d(q)
     */
    inline static Jacobian analyticalJacobianBody(const TMat &T, const ScrewList &BList, const ThetaList &thetaList);

    /**
     * Compute analytical jacobian d([r, x])/d(q)
     * @param T The pose of the jacobian defined link w.r.t. world
     * @param BList The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
     * @param thetaList  A list of joint coordinates
     * @return analytical jacobian d([r, x])/d(q)
     */
    inline static Jacobian analyticalJacobianSpace(const TMat &T, const ScrewList &SList, const ThetaList &thetaList);


    /**
     * Compute numerical solution of inverse kinematics, Newton-Raphson method
     * @param T The query end-effector pose in SE3;
     * @param angles The joint angles
     * @param M The home configuration
     * @param SList The screw list in the body frame
     * @param eomg error thresh for omega
     * @param ev error thresh for v
     * @return The theta list whose fk result equals to T
     */
    inline static bool
    numericalIKInBody(const TMat &T, ThetaList &angles, const TMat &M, const ScrewList &BList, double eomg, double ev);

    /**
     * Compute numerical solution of inverse kinematics, Levenberg-Marquardt method
     * @param T The query end-effector pose in SE3;
     * @param angles The joint angles
     * @param M The home configuration
     * @param SList The screw list in the body frame
     * @param eomg error thresh for omega
     * @param ev error thresh for v
     * @return The theta list whose fk result equals to T
     */
    inline static bool
    numericalIKLMInBody(const TMat &T, ThetaList &angles, const TMat &M, const ScrewList &BList, double eomg,
                        double ev, double *output_error = nullptr);

    /**
     * Compute numerical solution of inverse kinematics Newton-Raphson method
     * @param T The query end-effector pose in SE3;
     * @param angles The joint angles
     * @param M The home configuration
     * @param SList The screw list in the space space
     * @param eomg error thresh for omega
     * @param ev error thresh for v
     * @return The theta list whose fk result equals to T
     */
    inline static bool
    numericalIKInSpace(const TMat &T, ThetaList &angles, const TMat &M, const ScrewList &SList, double eomg,
                       double ev);

    inline static void pesudoInverse(const Eigen::MatrixXd &J, Eigen::MatrixXd &J_pinv);

    /**
     * Compute human-liked numerical solution of inverse kinematics Levenberg-Marquardt method
     * @param getHumanoidGradient The function to compute gradient of humanoid index, this function must return the same size of theta list
     * @param T The query end-effector pose in SE3;
     * @param angles The joint angles
     * @param M The home configuration
     * @param SList The screw list in the space space
     * @param emog error thresh for omega
     * @param ev error thresh for v
     * @return The theta list whose fk result equals to T
     */
    inline static bool
    humanoidIKLMInSpace(const gradient_func &getHumanoidGradient, const TMat &T, ThetaList &angles, const TMat &M,
                        const ScrewList &SList, double emog, double ev, double *output_error = nullptr);

    /**
     * Compute numerical solution of inverse kinematics Levenberg-Marquardt method
     * @param T The query end-effector pose in SE3;
     * @param angles The joint angles
     * @param M The home configuration
     * @param SList The screw list in the space space
     * @param emog error thresh for omega
     * @param ev error thresh for v
     * @return The theta list whose fk result equals to T
     */
    inline static double
    numericalIKLMInSpace(const TMat &T, ThetaList &angles, const TMat &M, const ScrewList &SList, double emog,
                         double ev, double *output_error = nullptr);

    inline static double singleGradient(const double &u, const double &l, const double &x, const double &d);

    inline static void enforceLimits(double &val, double min, double max);

    inline static Eigen::VectorXd
    computeQNull(const Eigen::VectorXd &angles, const double &jd, const Eigen::Matrix<double, -1, 2> &limits);

    /**
     * Compute numerical solution of inverse kinematics Levenberg-Marquardt method
     * @param T The query end-effector pose in SE3;
     * @param angles The joint angles
     * @param M The home configuration
     * @param SList The screw list in the space space
     * @param limits The given joint limits
     * @param eomg error thresh for omega
     * @param ev error thresh for v
     * @return The theta list whose fk result equals to T
     */
    inline static bool
    numericalIKLMWithLimitsInSpace(const TMat &T, ThetaList &angles, const TMat &M, const ScrewList &SList,
                                   const Eigen::Matrix<double, -1, 2> &limits, double eomg,
                                   double ev);

    inline static double
    solveQP(Eigen::VectorXd &g, const double error, const Eigen::VectorXd &q, const robot::Jacobian &J,
            const Eigen::VectorXd &v,
            const Eigen::Matrix<double, -1, 3> &limits, const Eigen::VectorXd *optional_term = nullptr);

    /**
     * Compute numerical solution of inverse kinematics Levenberg-Marquardt method
     * @param T The query end-effector pose in SE3;
     * @param angles The joint angles
     * @param M The home configuration
     * @param SList The screw list in the space space
     * @param limits The given joint limits
     * @param eomg error thresh for omega
     * @param ev error thresh for v
     * @return The theta list whose fk result equals to T
     */
    inline static bool
    numericalIKQPInSpace(const TMat &T, ThetaList &angles, const TMat &M, const ScrewList &SList,
                         const Eigen::Matrix<double, -1, 3> &limits, double eomg,
                         double ev, double *output_error = nullptr);

    inline static bool
    humanoidIKQPInSpace(const gradient_func &getHumanoidGradient, const TMat &T, ThetaList &angles, const TMat &M,
                        const ScrewList &SList,
                        const Eigen::Matrix<double, -1, 3> &limits, double eomg,
                        double ev, double *output_error = nullptr);

}
#include "../../../src/robot.cpp"
#endif //GCOPTER_ROBOT_HPP
