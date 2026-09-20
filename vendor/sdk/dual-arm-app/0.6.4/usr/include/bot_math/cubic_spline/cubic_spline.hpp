#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include "cubic_curve.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <utility>
#include <vector>

namespace cubic_spline {

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q) {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
        }

        inline void destroy() {
            if (ptrData != nullptr) {
                delete[] ptrData;
                ptrData = nullptr;
            }
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void) {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j) {
            return ptrData[(i - j + upperBw) * N + j];
        }

        bool isNotEmptyAt(const int &i, const int &j) {
            int index = (i - j + upperBw) * N + j;
            return index >= 0 && index < N * (lowerBw + upperBw + 1);
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU() {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; ++k) {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; ++i) {
                    if (operator()(i, k) != 0.0) {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; ++j) {
                    cVl = operator()(k, j);
                    if (cVl != 0.0) {
                        for (int i = k + 1; i <= iM; ++i) {
                            if (operator()(i, k) != 0.0) {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template<typename EIGENMAT>
        inline void solve(EIGENMAT &b) const {
            int iM;
            for (int j = 0; j <= N - 1; ++j) {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; ++i) {
                    if (operator()(i, j) != 0.0) {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; --j) {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; ++i) {
                    if (operator()(i, j) != 0.0) {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template<typename EIGENMAT>
        inline void solveAdj(EIGENMAT &b) const {
            int iM;
            for (int j = 0; j <= N - 1; ++j) {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; ++i) {
                    if (operator()(j, i) != 0.0) {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; --j) {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; ++i) {
                    if (operator()(j, i) != 0.0) {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
        }
    };

    enum class Type {
        CSFE = 0, // constrained start and free end
        CSCE = 1, // constrained start and constrained end
        CACV = 2, // constrained initial and end velocities and accelerations
    };

    class CubicSplineBase {
    public:
        explicit CubicSplineBase() = default;

        virtual ~CubicSplineBase() {
            A.destroy();
        };

    protected:
        int m {};
        int N {};
        Eigen::VectorXd headP;
        Eigen::VectorXd tailP;
        mutable BandedSystem A;

        Eigen::MatrixXd C;
        Eigen::VectorXd CH;
        Eigen::VectorXd CP;

        Eigen::MatrixXd D;

        Eigen::MatrixXd M;

        Eigen::MatrixXd b;

        Eigen::RowVectorXd durs;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd QT;

        std::vector<Eigen::Matrix<double, -1, 4>> cMats;

        Eigen::Matrix2d K;

        double tolerance = 0.1;
    public:
        inline virtual void setConditions(const Eigen::VectorXd &headPos,
                                          const Eigen::VectorXd &tailPos,
                                          const int &pieceNum) {
            m = (int) headPos.size();
            N = pieceNum;
            if(N < 4){
                throw std::invalid_argument("The piece number must bigger than 3, try interpolate intermediate points");
            }
            headP = headPos;
            tailP = tailPos;
            A.create(N + 1, 1, 1);

            C.resize(N + 1, N - 1);
            CH.resize(N + 1, 1);
            CP.resize(N + 1, 1);

            D.resize(N + 1, m);

            M.resize(N + 1, m);
            M.setZero();

            b.resize(N - 1, m);

            durs.resize(N);
            Q.resize(m, N - 1);

            cMats.resize(N);
            for (auto &cMat: cMats) {
                cMat.resize(m, 4);
            }

            K << 1, 1, 0, 1;

            C.setZero();
            CH.setZero();
            CP.setZero();
            D.setZero();
        }

        inline virtual void setTime(const Eigen::Matrix<double, 1, -1> &time) = 0;

        inline void setInnerPoints(const Eigen::Matrix<double, -1, -1> &inPs) {
            Q = inPs;
            QT = Q.transpose();
        }

        inline void setVariables(const Eigen::Matrix<double, -1, -1> &vars) {
            const auto R = vars.rows();
            const auto &points = R == m + 1 ? vars.block(0, 0, m, N - 1) :
                                 R == 1 ? Q : vars.topRows(m);
            setInnerPoints(points);
            if (R == m + 1) {
                setTime(vars.row(m));
            } else if (R == 1) {
                setTime(vars);
            }
        }

        inline virtual void init() = 0;

        inline virtual void getCurve(CubicCurve &curve) const = 0;

        inline void getStretchEnergy(double &energy) const {
            energy = 0.;
            for (int i = 0; i < N; ++i) {
                const auto &mj = M.block(i, 0, 2, m);
                energy += (mj.transpose() * K * mj).trace() * durs[i] / 3.;
            }
        }

        inline const Eigen::MatrixXd &getCoeffs() const {
            return b;
        }

    };

    typedef std::shared_ptr<CubicSplineBase> CubicSplinePtr;

    /**
     * this class solve constrained INITIAL and constrained END velocities (typical zero)
     */
    class CubicSplineClamped : public CubicSplineBase {
    public:
        explicit CubicSplineClamped(Eigen::VectorXd initial_vel = Eigen::VectorXd {},
                                    Eigen::VectorXd end_vel = Eigen::VectorXd {}, bool stretch = false,
                                    double mu = 0.999)
                : CubicSplineBase(), headV(std::move(initial_vel)), tailV(std::move(end_vel)), m_stretch(stretch) {
            if (m_stretch) {
                mu = (mu <= 1 && mu > 0) ? mu : 0.9;
                lambda = (1. - mu) / mu;
            }
            if (m_stretch) {
                A.destroy();
                A.create(N + 1, 2, 2);
            }
        };

        ~CubicSplineClamped() override = default;

    protected:
        Eigen::VectorXd headV;
        Eigen::VectorXd tailV;
        bool m_stretch;
        double lambda;
    public:
        inline void setTime(const Eigen::Matrix<double, 1, -1> &time) override {
            durs = time;
            const auto &T = durs;
            A.reset();
            A(0, 0) = T(0) / 3.;
            A(0, 1) = T(0) / 6.;

            A(N, N - 1) = T(N - 1) / 6.;
            A(N, N) = T(N - 1) / 3.;
            for (int row = 1; row < N; row++) {
                A(row, row - 1) = T(row - 1) / 6.;
                A(row, row) = (T(row - 1) + T(row)) / 3.;
                A(row, row + 1) = T(row) / 6.;
            }

            C(0, 0) = 1. / T(0);
            C.block(1, 0, 1, 2) << -1. / T(0) - 1. / T(1), 1. / T(1);
            for (int row = 2; row < N - 1; ++row) {
                C.block(row, row - 2, 1, 3) << 1. / T(row - 1), -1. / T(row - 1) - 1. / T(row), 1. / T(row);
            }
            C(N, N - 2) = 1. / T(N - 1);
            C.block(N - 1, N - 3, 1, 2) << 1. / T(N - 2), -1. / T(N - 2) - 1. / T(N - 1);

            if (m_stretch) {
                for (int i = 0; i < N + 1; ++i) {
                    for (int j = 0; j < N + 1; ++j) {
                        if (A.isNotEmptyAt(i, j))
                            A(i, j) = A(i, j) + lambda * C.row(i) * C.row(j).transpose();;
                    }
                }
            }

            CH(0) = -C(0, 0);
            CH(1) = C(0, 0);

            CP(N - 1) = C(N, N - 2);
            CP(N) = -CP(N - 1);

            if (headV.size() != 0 && tailV.size() != 0) {
                D.row(0) = headV;
                D.row(N) = -tailV;
                m_stretch = m_stretch && (headV.norm() < std::numeric_limits<double>::epsilon());
            }

            A.factorizeLU();
        }

        inline void init() override {
            while (true) {
                M.noalias() = CH * headP.transpose() + C * QT + CP * tailP.transpose() - D;
                A.template solve(M);
                Eigen::MatrixXd MT = M.transpose();
                const Eigen::MatrixXd &S = m_stretch ? Q - lambda * MT * C : Q;
                double delta = (S - Q).cwiseAbs().maxCoeff();
                if (delta > tolerance) {
                    lambda = lambda / (2. + lambda);
                    setTime(durs);
                } else {
                    for (int i = 0; i < N; ++i) {
                        auto &cMat = cMats[i];
                        const auto &qi = i == 0 ? headP : S.col(i - 1);
                        const auto &qi_p1 = i == N - 1 ? tailP : S.col(i);
                        const auto &mi = MT.col(i);
                        const auto &mi_p1 = MT.col(i + 1);
                        const auto &Ti = durs(i);

                        cMat.col(3) = qi;
                        cMat.col(2) = (qi_p1 - qi) / Ti - Ti / 3. * (mi + mi_p1 / 2.);
                        cMat.col(1) = mi / 2.;
                        cMat.col(0) = (mi_p1 - mi) / 6. / Ti;
                    }
                    break;
                }
            }
        }

        inline void getCurve(CubicCurve &curve) const override {
            std::vector<double> durs_(N);
            memcpy(durs_.data(), durs.data(), N * sizeof(double));
            curve = CubicCurve(durs_, cMats);
        };
    };


    /**
     * this class solve constrained INITIAL and constrained END velocities & accelerations (typical zero)
     */
    class CubicSplineConstrained : public CubicSplineBase {
    public:
        explicit CubicSplineConstrained(Eigen::VectorXd initial_vel = Eigen::VectorXd {},
                                        Eigen::VectorXd end_vel = Eigen::VectorXd {},
                                        Eigen::VectorXd initial_acc = Eigen::VectorXd {},
                                        Eigen::VectorXd end_acc = Eigen::VectorXd {}, bool stretch = false,
                                        double mu = 0.999)
                : CubicSplineBase(), headV(std::move(initial_vel)), tailV(std::move(end_vel)),
                  headA(initial_acc.transpose()),
                  tailA(end_acc.transpose()), m_stretch(stretch) {
            if (m_stretch) {
                mu = (mu <= 1. && mu >= 0.) ? mu : 0.9;
                lambda = (1. - mu) / mu;
                if (1. - mu < std::numeric_limits<double>::epsilon()) {
                    m_stretch = false;
                }
            }
        };

        ~CubicSplineConstrained() override = default;

    protected:
        Eigen::VectorXd headV;
        Eigen::VectorXd tailV;
        Eigen::RowVectorXd headA;
        Eigen::RowVectorXd tailA;
        bool m_stretch;
        double lambda;
        double mu = 0.5;
    public:
        inline void setConditions(const Eigen::VectorXd &headPos,
                                  const Eigen::VectorXd &tailPos,
                                  const int &pieceNum) override {
            CubicSplineBase::setConditions(headPos, tailPos, pieceNum);
            cMats.resize(N + 2);
            for (auto &cMat: cMats) {
                cMat.resize(m, 4);
            }
            if (m_stretch) {
                A.destroy();
                A.create(N + 1, 2, 2);
            }
        }

        inline void setTime(const Eigen::Matrix<double, 1, -1> &time) override {
            durs = time;
            const auto &T = durs;
            double one_minus_mu = 1. - mu;
            double one_minus_mu_inverse = 1. / one_minus_mu;
            double frac = one_minus_mu * one_minus_mu_inverse;

            A.reset();
            A(0, 0) = T(0) * (2. - mu) * one_minus_mu_inverse / 6.;
            A(0, 1) = T(0) * (1. - mu) / 6.;

            A(1, 0) = (one_minus_mu - mu) * one_minus_mu_inverse * T(0) / 6.;
            A(1, 1) = (T(0) * one_minus_mu + T(1)) / 3.;
            A(1, 2) = T(1) / 6.;

            A(N, N) = T(N - 1) * (2. - mu) * one_minus_mu_inverse / 6.;
            A(N, N - 1) = T(N - 1) * (1. - mu) / 6.;
            A(N - 1, N) = (one_minus_mu - mu) * one_minus_mu_inverse * T(N - 1) / 6.;
            A(N - 1, N - 1) = (one_minus_mu * T(N - 1) + T(N - 2)) / 3.;
            A(N - 1, N - 2) = T(N - 2) / 6.;

            for (int row = 2; row < N - 1; row++) {
                A(row, row - 1) = T(row - 1) / 6.;
                A(row, row) = (T(row - 1) + T(row)) / 3.;
                A(row, row + 1) = T(row) / 6.;
            }


            C(0, 0) = one_minus_mu_inverse / T(0);
            C.block(1, 0, 1, 2) << -one_minus_mu_inverse / T(0) - 1. / T(1), 1. / T(1);
            for (int row = 2; row < N - 1; ++row) {
                C.block(row, row - 2, 1, 3) << 1. / T(row - 1), -1. / T(row - 1) - 1. / T(row), 1. / T(row);
            }
            C(N, N - 2) = one_minus_mu_inverse / T(N - 1);
            C.block(N - 1, N - 3, 1, 2) << 1. / T(N - 2), -1. / T(N - 2) - one_minus_mu_inverse / T(N - 1);

            if (m_stretch) {
                auto getColIndex = [](int N, int i) { return i < 2 ? 0 : i > N - 2 ? N - 4 : i - 2; };
                for (int i = 0; i < N + 1; ++i) {
                    for (int j = 0; j < N + 1; ++j) {
                        if (A.isNotEmptyAt(i, j)) {
                            A(i, j) = A(i, j) + lambda * C.row(i).block<1, 3>(0, getColIndex(N, i)) *
                                                C.row(j).block<1, 3>(0, getColIndex(N, i)).transpose();
                        }
                    }
                }
            }

            CH(0) = -C(0, 0);
            CH(1) = C(0, 0);

            CP(N - 1) = C(N, N - 2);
            CP(N) = -CP(N - 1);

            if (headV.size() != 0 && tailV.size() != 0 && tailA.size() != 0 && headA.size() != 0) {
                D.row(0) = (1. + frac) * headV.transpose() + (mu / 2. + mu * frac / 3.) * T(0) * headA;
                D.row(1) = -frac * headV.transpose() - mu * frac / 3. * T(0) * headA;
                D.row(N - 1) = frac * tailV.transpose() - mu * frac / 3. * T(N - 1) * tailA;
                D.row(N) = -(frac + 1.) * tailV.transpose() + (mu / 2. + mu * frac / 3.) * T(N - 1) * tailA;
                m_stretch = m_stretch && headV.norm() < std::numeric_limits<double>::epsilon();
            }

            A.factorizeLU();
        }

        inline void init() override {
            while (true) {
                M.noalias() = CH * headP.transpose() + C * QT + CP * tailP.transpose() - D;
                A.template solve(M);
                Eigen::MatrixXd MT = M.transpose();
                const Eigen::MatrixXd &S = m_stretch ? Q - lambda * MT * C : Q;
                double delta = (S - Q).cwiseAbs().maxCoeff();
                if (delta > tolerance) {
                    lambda = lambda / (2. + lambda);
                    setTime(durs);
                } else {
                    Eigen::VectorXd mi(m), mi_p1(m);
                    Eigen::VectorXd qi(m), qi_p1(m);
                    double Ti;
                    for (int i = 0; i < N + 2; ++i) {
                        auto &cMat = cMats[i];
                        if (i <= 1) {
                            Ti = durs[0] * mu;
                            Eigen::Matrix<double, -1, 1> overline_q1 = headP + Ti * headV +
                                                                       Ti * Ti *
                                                                       (2. * headA.transpose() + MT.col(0)) / 6.;
                            if (i == 0) {
                                mi = headA.transpose();
                                mi_p1 = MT.col(0);

                                qi = headP;
                                qi_p1 = overline_q1;
                            } else {
                                mi = MT.col(0);
                                mi_p1 = MT.col(1);
                                qi = overline_q1;
                                qi_p1 = S.col(0);
                                Ti = durs[0] * (1. - mu);
                            }

                        } else if (i < N) {
                            Ti = durs(i - 1);
                            qi = S.col(i - 2);
                            qi_p1 = S.col(i - 1);
                            mi = MT.col(i - 1);
                            mi_p1 = MT.col(i);
                        } else {
                            Ti = durs[N - 1] * mu;
                            Eigen::Matrix<double, -1, 1> overline_qN = tailP - Ti * tailV +
                                                                       Ti * Ti *
                                                                       (2. * tailA.transpose() + MT.col(N)) / 6.;;
                            if (i == N + 1) {
                                mi = MT.col(N);
                                mi_p1 = tailA.transpose();
                                qi = overline_qN;
                                qi_p1 = tailP;
                            } else {
                                mi = MT.col(N - 1);
                                mi_p1 = MT.col(N);
                                qi = S.col(N - 2);
                                qi_p1 = overline_qN;
                                Ti = durs[N - 1] * (1. - mu);
                            }
                        }
                        cMat.col(3) = qi;
                        cMat.col(2) = (qi_p1 - qi) / Ti - Ti / 3. * (mi + mi_p1 / 2.);
                        cMat.col(1) = mi / 2.;
                        cMat.col(0) = (mi_p1 - mi) / 6. / Ti;
                    }
                    break;
                }
            }
        }

        inline void getCurve(CubicCurve &curve) const override {
            std::vector<double> durs_(N + 2);
            for (int i = 2; i < N; ++i) {
                durs_[i] = durs[i - 1];
            }
            durs_[0] = durs[0] * mu;
            durs_[1] = durs[0] * (1 - mu);
            durs_[N + 1] = durs[N - 1] * mu;
            durs_[N] = durs[N - 1] * (1 - mu);
            curve = CubicCurve(durs_, cMats);
        };
    };

    template<int m>
    class CubicSpline {
    public:
        explicit CubicSpline(bool minimize_stretch = false) : m_stretch(minimize_stretch) {

        };

        ~CubicSpline() { A.destroy(); }

    private:
        int N {};
        Eigen::Matrix<double, m, 1> headP;
        Eigen::Matrix<double, m, 1> tailP;
        Eigen::Matrix<double, 1, m> headV;
        Eigen::Matrix<double, 1, m> tailV;
        Eigen::Matrix<double, 1, m> headA;
        Eigen::Matrix<double, 1, m> tailA;
        mutable BandedSystem A;
        Eigen::MatrixXd overline_A;
        Eigen::Matrix<double, -1, -1> C;
        Eigen::Matrix<double, -1, 1> CH;
        Eigen::Matrix<double, -1, 1> CP;

        Eigen::Matrix<double, -1, m> D;

        Eigen::Matrix<double, -1, m> M;

        Eigen::Matrix<double, -1, m> b;

        std::vector<Eigen::Matrix<double, m, 4>> cMats;
        Eigen::Matrix<double, 1, -1> durs;
        Eigen::Matrix<double, m, -1> Q;
        mutable std::vector<Eigen::Matrix<double, -1, m>> dMT;

        Eigen::Matrix<double, 2, 2> K;
        Eigen::Matrix<double, 2, 2> KH;

        Type type {};

        double mu {0.5};

        bool m_stretch {false};
    public:
        inline void setConditions(const Eigen::Matrix<double, m, 1> &headPos,
                                  const Eigen::Matrix<double, m, 1> &tailPos,
                                  const int &pieceNum,
                                  const Type &tp = Type::CSCE,
                                  const Eigen::Matrix<double, m, 1> &initialS = Eigen::Matrix<double, m, 1>::Zero(),
                                  const Eigen::Matrix<double, m, 1> &initialE = Eigen::Matrix<double, m, 1>::Zero(),
                                  const Eigen::Matrix<double, m, 1> &initialSA = Eigen::Matrix<double, m, 1>::Zero(),
                                  const Eigen::Matrix<double, m, 1> &initialEA = Eigen::Matrix<double, m, 1>::Zero()
        ) {
            type = tp;
            N = pieceNum;
            headP = headPos;
            tailP = tailPos;
            headV = initialS.transpose();
            tailV = initialE.transpose();
            headA = initialSA.transpose();
            tailA = initialEA.transpose();
            cMats.resize(N);
            if (type == Type::CACV) {
                cMats.resize(N + 2);
            }


            durs.resize(N);

            if (type == Type::CSFE)
                A.create(N, 1, 1);
            else
                A.create(N + 1, 1, 1);
            if (m_stretch) {
                overline_A.resize(N + 1, N + 1);
            }
            C.resize(N + 1, N - 1);

            D.resize(N + 1, m);

            CH.resize(N + 1, 1);
            CP.resize(N + 1, 1);

            M.resize(N + 1, m);
            M.setZero();

            b.resize(N - 1, m);

            Q.resize(m, N - 1);

            dMT.resize(N);

            K << 1, 1, 0, 1;

            KH << 2, 1, 1, 2;
        }

        inline void setTime(const Eigen::Matrix<double, 1, -1> &time) {
            durs = time;
            const auto &T = durs;
            double one_minus_mu = 1. - mu;
            double one_minus_mu_inverse = 1. / one_minus_mu;
            double frac = one_minus_mu * one_minus_mu_inverse;

            A(0, 0) = T(0) / 3.;
            A(0, 1) = T(0) / 6.;
            if (type == Type::CSFE) {
                A(N - 1, N - 1) = (T(N - 2) + T(N - 1)) / 3.;
                A(N - 1, N - 2) = T(N - 2) / 6.;
                for (int row = 1; row < N - 1; row++) {
                    A(row, row - 1) = T(row - 1) / 6.;
                    A(row, row) = (T(row - 1) + T(row)) / 3.;
                    A(row, row + 1) = T(row) / 6.;
                }
            } else if (type == Type::CSCE) {
                A(N, N - 1) = T(N - 1) / 6.;
                A(N, N) = T(N - 1) / 3.;
                for (int row = 1; row < N; row++) {
                    A(row, row - 1) = T(row - 1) / 6.;
                    A(row, row) = (T(row - 1) + T(row)) / 3.;
                    A(row, row + 1) = T(row) / 6.;
                }
            } else if (type == Type::CACV) {

                A(0, 0) = T(0) * (2. - mu) * one_minus_mu_inverse / 6.;
                A(0, 1) = T(0) * (1. - mu) / 6.;

                A(1, 0) = (one_minus_mu - mu) * one_minus_mu_inverse * T(0) / 6.;
                A(1, 1) = (T(0) * one_minus_mu + T(1)) / 3.;
                A(1, 2) = T(1) / 6.;

                A(N, N) = T(N - 1) * (2. - mu) * one_minus_mu_inverse / 6.;
                A(N, N - 1) = T(N - 1) * (1. - mu) / 6.;
                A(N - 1, N) = (one_minus_mu - mu) * one_minus_mu_inverse * T(N - 1) / 6.;
                A(N - 1, N - 1) = (one_minus_mu * T(N - 1) + T(N - 2)) / 3.;
                A(N - 1, N - 2) = T(N - 2) / 6.;

                for (int row = 2; row < N - 1; row++) {
                    A(row, row - 1) = T(row - 1) / 6.;
                    A(row, row) = (T(row - 1) + T(row)) / 3.;
                    A(row, row + 1) = T(row) / 6.;
                }
            }
            if (m_stretch) {
                overline_A.setZero();
                for (int i = 0; i < N + 1; ++i) {
                    for (int j = 0; j < N + 1; ++j) {
                        if (A.isNotEmptyAt(i, j))
                            overline_A(i, j) = A(i, j);
                    }
                }
                Eigen::MatrixXd A_temp = overline_A;
                if (type == Type::CACV) {
                    overline_A.block<2, 2>(0, 0) << T(0) / 3., T(0) / 6., T(0) / 6., (T(0) + T(1)) / 3.;
                    overline_A.block<2, 2>(N - 1, N - 1) << (T(N - 2) + T(N - 1)) / 3., T(N - 1) / 6., T(N - 1) / 6.,
                            T(N - 1) / 3.;
                }
                overline_A = A_temp * overline_A.inverse() * A_temp;

            }

            A.factorizeLU();

            if (C.size() != 0) {
                C.setZero();
                C(0, 0) = 1. / T(0);
                C.block(1, 0, 1, 2) << -1. / T(0) - 1. / T(1), 1. / T(1);
                C.block(N - 1, N - 3, 1, 2) << 1. / T(N - 2), -1. / T(N - 2) - 1. / T(N - 1);
                if (type == Type::CACV) {
                    C(0, 0) = one_minus_mu_inverse / T(0);
                    C.block(1, 0, 1, 2) << -one_minus_mu_inverse / T(0) - 1. / T(1), 1. / T(1);
                    C.block(N - 1, N - 3, 1, 2) << 1. / T(N - 2), -1. / T(N - 2) - one_minus_mu_inverse / T(N - 1);
                    C(N, N - 2) = one_minus_mu_inverse / T(N - 1);
                }
                for (int row = 2; row < N - 1; ++row) {
                    C.block(row, row - 2, 1, 3) << 1. / T(row - 1), -1. / T(row - 1) - 1. / T(row), 1. / T(row);
                }
                if (type == Type::CSCE) {
                    C(N, N - 2) = 1. / T(N - 1);
                }
            }


            CH.setZero();
            if (C.size() != 0) {
                CH(0) = -C(0, 0);
                CH(1) = C(0, 0);
            }

            CP.setZero();
            CP(N - 1) = C(N, N - 2);

            if (type != Type::CSFE) {
                CP(N) = -CP(N - 1);
            }

            D.setZero();
            D.row(0) = headV;
            if (type == Type::CSCE) {
                D.row(N) = -tailV;
            } else if (type == Type::CACV) {
                D.row(0) = (1. + frac) * headV + (mu / 2. + mu * frac / 3.) * T(0) * headA;
                D.row(1) = -frac * headV - mu * frac / 3. * T(0) * headA;
                D.row(N - 1) = frac * tailV - mu * frac / 3. * T(N - 1) * tailA;
                D.row(N) = -(frac + 1.) * tailV + (mu / 2. + mu * frac / 3.) * T(N - 1) * tailA;
            }
            A.template solve(CH);
            A.template solve(C);
            A.template solve(CP);
            A.template solve(D);
        }

        inline void setInnerPoints(const Eigen::Matrix<double, -1, -1> &inPs) {
            Q = inPs;
        }

        inline void setVariables(const Eigen::Matrix<double, -1, -1> &vars) {
            const auto R = vars.rows();
            const auto &points = R == m + 1 ? vars.block(0, 0, m, N - 1) :
                                 R == 1 ? Q : vars.topRows(m);
            setInnerPoints(points);
            if (R == m + 1) {
                setTime(vars.row(m));
            } else if (R == 1) {
                setTime(vars);
            }
        }

        inline void init() {
            M.noalias() = CH * headP.transpose() + C * Q.transpose() + CP * tailP.transpose() - D;

            Eigen::Matrix<double, m, 1> mi, mi_p1;
            if (type != Type::CACV) {
                for (int i = 0; i < N; ++i) {
                    auto &cMat = cMats[i];
                    const auto &qi = i == 0 ? headP : Q.col(i - 1);
                    const auto &qi_p1 = i == N - 1 ? tailP : Q.col(i);
                    mi = M.row(i).transpose();
                    mi_p1 = M.row(i + 1).transpose();
                    const auto &Ti = durs(i);

                    cMat.col(3) = qi;
                    cMat.col(2) = (qi_p1 - qi) / Ti - Ti / 3. * (mi + mi_p1 / 2.);
                    cMat.col(1) = mi / 2.;
                    cMat.col(0) = (mi_p1 - mi) / 6. / Ti;
                }
            } else {
                Eigen::Matrix<double, m, 1> qi, qi_p1;
                double Ti;
                for (int i = 0; i < N + 2; ++i) {
                    auto &cMat = cMats[i];
                    if (i <= 1) {
                        Ti = durs[0] * mu;
                        Eigen::Matrix<double, m, 1> overline_q1 = headP + Ti * headV.transpose() +
                                                                  Ti * Ti *
                                                                  (2. * headA.transpose() + M.row(0).transpose()) / 6.;
                        if (i == 0) {
                            mi = headA.transpose();
                            mi_p1 = M.row(0).transpose();

                            qi = headP;
                            qi_p1 = overline_q1;
                        } else {
                            mi = M.row(0).transpose();
                            mi_p1 = M.row(1).transpose();
                            qi = overline_q1;
                            qi_p1 = Q.col(0);
                            Ti = durs[0] * (1. - mu);
                        }

                    } else if (i < N) {
                        Ti = durs(i - 1);
                        qi = Q.col(i - 2);
                        qi_p1 = Q.col(i - 1);
                        mi = M.row(i - 1).transpose();
                        mi_p1 = M.row(i).transpose();
                    } else {
                        Ti = durs[N - 1] * mu;
                        Eigen::Matrix<double, m, 1> overline_qN = tailP - Ti * tailV.transpose() +
                                                                  Ti * Ti *
                                                                  (2. * tailA.transpose() + M.row(N).transpose()) / 6.;;
                        if (i == N + 1) {
                            mi = M.row(N).transpose();
                            mi_p1 = tailA.transpose();
                            qi = overline_qN;
                            qi_p1 = tailP;
                        } else {
                            mi = M.row(N - 1).transpose();
                            mi_p1 = M.row(N).transpose();
                            qi = Q.col(N - 2);
                            qi_p1 = overline_qN;
                            Ti = durs[N - 1] * (1. - mu);
                        }
                    }

                    cMat.col(3) = qi;
                    cMat.col(2) = (qi_p1 - qi) / Ti - Ti / 3. * (mi + mi_p1 / 2.);
                    cMat.col(1) = mi / 2.;
                    cMat.col(0) = (mi_p1 - mi) / 6. / Ti;
                }
            }
        }

        inline void getCurve(CubicCurve &curve) const {
            if (type != Type::CACV) {
                std::vector<double> durs_(N);
                memcpy(durs_.data(), durs.data(), N * sizeof(double));
                curve = CubicCurve(durs_, cMats);
            } else {
                std::vector<double> durs_(N + 2);
                for (int i = 2; i < N; ++i) {
                    durs_[i] = durs[i - 1];
                }
                durs_[0] = durs[0] * mu;
                durs_[1] = durs[0] * (1 - mu);
                durs_[N + 1] = durs[N - 1] * mu;
                durs_[N] = durs[N - 1] * (1 - mu);
                curve = CubicCurve(durs_, cMats);
            }

        }

        inline void getStretchEnergy(double &energy) const {
            energy = 0.;
            for (int i = 0; i < N; ++i) {
                const auto &mj = M.block(i, 0, 2, m);
                energy += (mj.transpose() * K * mj).trace() * durs[i] / 3.;
            }
        }

        inline const Eigen::Matrix<double, -1, m> &getCoeffs(void) const {
            return b;
        }

        inline void getGrads(Eigen::Ref<Eigen::Matrix<double, -1, -1>> grads) const {
            grads.setZero();
            const auto R = grads.rows();
            if (R == 1) {
                getGradT(grads);
            } else if (R == m) {
                getGradQ(grads);
            } else {
                Eigen::Matrix<double, -1, -1> gradT(1, N);
                getGradT(gradT);
                Eigen::Matrix<double, -1, -1> gradQ(m, N - 1);
                getGradQ(gradQ);

                grads.block(0, 0, m, N - 1) = gradQ;
                grads.row(m) = gradT;
            }
        }

        inline void getGradQ(Eigen::Ref<Eigen::Matrix<double, -1, -1>> gradByQ) const {
            gradByQ.setZero();
            Eigen::Matrix<double, m, 2> mjT;
            for (int i = 0; i < N; ++i) {
                mjT = durs[i] / 3. * M.block(i, 0, 2, m).transpose();
                const auto &mjq = C.block(i, 0, 2, N - 1);
                gradByQ.noalias() += mjT * KH * mjq;
            }
        }

        inline void getGradT(Eigen::Ref<Eigen::Matrix<double, -1, -1>> gradByT) const {
            gradByT.setZero();
            Eigen::Matrix<double, 1, m> biT, diT;
            for (int i = 0; i < N; ++i) {
                auto &dMTi = dMT[i];
                dMTi.resize(N + 1, m);
                const auto &Ti = durs[i];
                biT = cMats[i].col(2).transpose() / Ti;
                diT = cMats[i].col(0).transpose() * Ti;
                const auto &mi = M.row(i);

                //dm w.r.t. Ti
                dMTi.setZero();
                dMTi.row(i + 1) = biT - diT;
                dMTi.row(i) = -dMTi.row(i + 1) - 3. * diT - mi;
                if (i == N - 1 && type == Type::CSFE)
                    dMTi.row(i + 1).setZero();
                A.template solve(dMTi);

                for (int j = 0; j < N; ++j) {
                    const auto &Tj = durs[j];
                    const auto &mj = M.block(j, 0, 2, m);
                    const auto mjT = mj.transpose();
                    const auto &mjTi = dMTi.block(j, 0, 2, m);

                    gradByT(0, i) += Tj / 3. * (mjT * KH * mjTi).trace();
                    if (i == j) {
                        gradByT(0, i) += (mjT * K * mj).trace() / 3.;
                    }
                }
            }
        }

        inline Eigen::Matrix<double, m, 1> getDerivatives(int order, int k) const {
            Eigen::Matrix<double, m, 1> result;
            if (order == 0) {
                if (k < N)
                    result = cMats[k].col(3);
                else {
                    const auto &cMat = cMats[N - 1];
                    const auto &T = durs[N - 1];
                    result = cMat.col(3) + (cMat.col(2) + (cMat.col(1) + cMat.col(0) * T) * T) * T;
                }
            } else if (order == 1) {
                if (k < N)
                    result = cMats[k].col(2);
                else {
                    const auto &cMat = cMats[N - 1];
                    const auto &T = durs[N - 1];
                    result = cMat.col(2) + (2. * cMat.col(1) + 3. * cMat.col(0) * T) * T;
                }
            } else {
                Eigen::Matrix<double, m, 1> Zero;
                Zero.setZero();
                result = M.row(k).transpose();
            }
            return result;
        }

        inline void getKthPositionAndGrads(Eigen::Matrix<double, m, 1> &position,
                                           Eigen::Matrix<double, -1, -1> &grads,
                                           const int k, int gradType, bool recursive = false) const {
            if (!recursive)
                position = getDerivatives(0, k);
            if (gradType == 0) {
                grads.resize(N, m);
                grads.setZero();
            } else if (gradType == 1) {
                grads.resize(1, N - 1);
                grads.setZero();
                if (k != N and k != 0) {
                    grads(k - 1) = 1.;
                }
            } else {
                grads.resize(N + 1, std::max(N - 1, m));
                grads.setZero();
                Eigen::Matrix<double, -1, -1> gradsT, gradsQ;
                getKthPositionAndGrads(position, gradsT, k, 0, true);
                getKthPositionAndGrads(position, gradsQ, k, 1, true);
                grads.block(0, 0, N, m) = gradsT;
                grads.row(N).head(N - 1) = gradsQ;
            }
        }

        inline void getKthVelocityAndGrads(Eigen::Matrix<double, m, 1> &vel,
                                           Eigen::Matrix<double, -1, -1> &grads,
                                           const int k, int gradType, bool recursive = false) const {
            if (!recursive)
                vel = getDerivatives(1, k);
            if (gradType == 0) {
                grads.resize(N, m);
                grads.setZero();
                if (!(k == 0 or (k == N && type == Type::CSCE))) {
                    const auto &Tj = k == N ? durs[N - 1] : durs[k];
                    const auto vj = vel.transpose() / Tj;
                    const auto mj = k == N ? M.row(N - 1) / 3. : M.row(k) / 3.;
                    const auto mj_p1 = k == N ? M.row(N - 1) / 3. : M.row(k + 1) / 3.;
                    Eigen::Matrix<double, 1, m> dmji, dmjp1;
                    for (int i = 0; i < N; ++i) {
                        const auto &dMTi = dMT[i];
                        dmji = k == N ? dMTi.row(N - 1) * Tj / 3. : dMTi.row(k) * Tj / 3.;
                        dmjp1 = k == N ? dMTi.row(N - 1) * Tj / 6. : dMTi.row(k + 1) * Tj / 6.;
                        if (k == N && type == Type::CSFE) {
                            grads.row(i) = dmji / 2.;
                            if (i == N - 1) {
                                grads.row(i).noalias() += mj - vj;
                            }
                        } else {
                            grads.row(i) = -dmji - dmjp1;
                            if (i == k) {
                                grads.row(i).noalias() += -vj - 2. * mj - mj_p1;
                            }
                        }
                    }
                }
            } else if (gradType == 1) {
                grads.resize(1, N - 1);
                grads.setZero();
                Eigen::RowVectorXd Lk(N - 1);
                Lk.setZero();
                if (k == N && type == Type::CSFE) {
                    Lk(N - 2) = -1;
                    grads = Lk / durs[N - 1] + C.row(N - 1) / 6. * durs[N - 1];
                } else if (k > 0 && k < N) {
                    Eigen::RowVectorXd ZERO(N - 1);
                    ZERO.setZero();
                    Lk(k - 1) = -1;
                    if (k != N - 1)
                        Lk(k) = 1;
                    const auto &Ti = durs[k];
                    const auto mi = C.row(k) / 3. * Ti;
                    const auto mi_p1 = k == N - 1 && type == Type::CSFE ? ZERO : C.row(k + 1) / 6. * Ti;
                    grads = Lk / Ti - mi - mi_p1;
                }
            } else {
                grads.resize(N + 1, std::max(N - 1, m));
                grads.setZero();
                Eigen::Matrix<double, -1, -1> gradsT, gradsQ;
                getKthVelocityAndGrads(vel, gradsT, k, 0, true);
                getKthVelocityAndGrads(vel, gradsQ, k, 1, true);
                grads.block(0, 0, N, m) = gradsT;
                grads.row(N).head(N - 1) = gradsQ;
            }

        }

        inline void getKthAccelerationAndGrads(Eigen::Matrix<double, m, 1> &acc,
                                               Eigen::Matrix<double, -1, -1> &grads,
                                               const int k, int gradType, bool recursive = false) const {
            if (!recursive)
                acc = getDerivatives(2, k);
            if (gradType == 0) {
                grads.resize(N, m);
                grads.setZero();
                if (k != N or type != Type::CSFE) {
                    for (int i = 0; i < N; ++i) {
                        grads.row(i) = dMT[i].row(k);
                    }
                }
            } else if (gradType == 1) {
                grads.resize(1, N - 1);
                grads.setZero();
                if (k != N or type != Type::CSFE) {
                    grads = C.row(k);
                }
            } else {
                grads.resize(N + 1, std::max(N - 1, m));
                grads.setZero();
                Eigen::Matrix<double, -1, -1> gradsT, gradsQ;
                getKthAccelerationAndGrads(acc, gradsT, k, 0, true);
                getKthAccelerationAndGrads(acc, gradsQ, k, 1, true);
                grads.block(0, 0, N, m) = gradsT;
                grads.row(N).head(N - 1) = gradsQ;
            }

        }

    };
}

#endif
