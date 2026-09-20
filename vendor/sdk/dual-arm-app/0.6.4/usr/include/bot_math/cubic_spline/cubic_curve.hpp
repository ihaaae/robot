#ifndef CUBIC_CURVE_HPP
#define CUBIC_CURVE_HPP

#include <Eigen/Eigen>

#include <iostream>
#include <cmath>
#include <cfloat>
#include <plog/Log.h>
#include <vector>

class CubicPolynomial
{
private:
    double duration{};
    Eigen::Matrix<double, -1, 4> coeffMat;
protected:

    [[nodiscard]] inline Eigen::VectorXd computeVec(const double& t) const{
        Eigen::VectorXd dqt = coeffMat.col(2) +  t * (2. * coeffMat.col(1) + t * 3. * coeffMat.col(0));
        return dqt.normalized();
    }

    [[nodiscard]] inline Eigen::VectorXd computeAcc(const double& t) const{
        Eigen::VectorXd dqt = coeffMat.col(2) +  t * (2. * coeffMat.col(1) + t * 3. * coeffMat.col(0));
        Eigen::VectorXd ddqt = 2. * coeffMat.col(1) + t * 6. * coeffMat.col(0);
        double snorm = dqt.squaredNorm();
        if(snorm == 0.)
            return Eigen::VectorXd::Zero(coeffMat.rows());
        else{
            return (ddqt - dqt * ddqt.dot(dqt) / snorm) / snorm;
        }
    }

public:
    CubicPolynomial() = default;

    CubicPolynomial(double dur, const Eigen::Matrix<double, -1, 4> &cMat)
        : duration(dur), coeffMat(cMat) {}

    [[nodiscard]] inline int getDim() const
    {
        return coeffMat.rows();
    }

    inline int getDegree() const
    {
        return 3;
    }

    inline double getDuration() const
    {
        return duration;
    }

    inline const Eigen::Matrix<double, -1, 4> &getCoeffMat() const
    {
        return coeffMat;
    }

    inline double getS() const{
        return (getVec(0).norm() + getVec(duration).norm() + 4 * getVec(duration / 2.).norm() ) * duration / 6.;;
    }

    inline Eigen::VectorXd getPos(const double &t) const
    {
        // std::cout << "getPos_t" <<t<< std::endl;
        // std::cout << "coeffMat.col(3)" <<coeffMat.col(3)<< std::endl;
        // std::cout << "coeffMat.col(2)" <<coeffMat.col(2)<< std::endl;
        // std::cout << "coeffMat.col(1)" <<coeffMat.col(1)<< std::endl;
        // std::cout << "coeffMat.col(0)" <<coeffMat.col(0)<< std::endl;
        return coeffMat.col(3) + t * (coeffMat.col(2) + t * (coeffMat.col(1) + t * coeffMat.col(0)));
    }
    inline Eigen::VectorXd getVec(const double& t)const{
        return coeffMat.col(2) +  t * (2. * coeffMat.col(1) + t * 3. * coeffMat.col(0));
    }
    inline Eigen::VectorXd getAcc(const double& t)const{
        return 2. * coeffMat.col(1) + t * 6. * coeffMat.col(0);
    }

    inline Eigen::VectorXd getJerk(const double& t)const{
        return 6. * coeffMat.col(0);
    }

    inline Eigen::VectorXd getVecS() const{
        Eigen::VectorXd q = computeVec(0) + 4 * computeVec(duration / 2.) + computeVec(duration);
        return q / 6.;
    }

    inline Eigen::VectorXd getAccS() const{
        Eigen::VectorXd q = computeAcc(0) + 4 * computeAcc(duration / 2.) + computeAcc(duration);
        return q / 6.;
    }

};


class CubicCurve
{
private:
    typedef std::vector<CubicPolynomial> Pieces;
    Pieces pieces;
public:
    CubicCurve() = default;

    CubicCurve(const std::vector<double> &durs,
               const std::vector<Eigen::Matrix<double, -1, 4>> &cMats)
    {
        const int N = std::min(durs.size(), cMats.size());
        pieces.reserve(N);
        for (int i = 0; i < N; ++i)
        {
            pieces.emplace_back(durs[i], cMats[i]);
        }
    }

    inline int getPieceNum() const
    {
        return pieces.size();
    }

    inline Eigen::VectorXd getDurations() const
    {
        const int N = getPieceNum();
        Eigen::VectorXd durations(N + 1); durations.setZero();
        for (int i = 1; i < N + 1; ++i)
        {
            durations(i) = durations(i - 1) + pieces[i - 1].getDuration();
        }
        return durations;
    }

    inline double getTotalDuration() const
    {
        const int N = getPieceNum();
        double totalDuration = 0.0;
        for (int i = 0; i < N; ++i)
        {
            totalDuration += pieces[i].getDuration();
        }
        return totalDuration;
    }

    inline Eigen::Matrix<double, -1, -1> getPositions() const
    {
        const int N = getPieceNum();
        Eigen::Matrix<double, -1, -1> positions(pieces[0].getDim(), N + 1);
        for (int i = 0; i < N; ++i)
        {
            positions.col(i) = pieces[i].getCoeffMat().col(3);
        }
        positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
        return positions;
    }

    inline Eigen::Matrix<double, -1, -1> getVelocities() const
    {
        const int N = getPieceNum();
        Eigen::Matrix<double, -1, -1> velocities(pieces[0].getDim(), N + 1);
        for (int i = 0; i < N; ++i)
        {
            velocities.col(i) = pieces[i].getCoeffMat().col(2);
        }
        velocities.col(N) = pieces[N - 1].getVec(pieces[N - 1].getDuration());
        return velocities;
    }
    inline Eigen::VectorXd getAccelerations() const
    {
        const int N = getPieceNum();
        Eigen::Matrix<double, -1, -1> accelerations(pieces[0].getDim(), N + 1);
        for (int i = 0; i < N; ++i)
        {
            accelerations.col(i) = pieces[i].getCoeffMat().col(1);
        }
        accelerations.col(N) = pieces[N - 1].getAcc(pieces[N - 1].getDuration());
        return accelerations;
    }
    inline const CubicPolynomial &operator[](int i) const
    {
        return pieces[i];
    }

    inline CubicPolynomial &operator[](int i)
    {
        return pieces[i];
    }

    inline void clear(void)
    {
        pieces.clear();
        return;
    }

    inline typename Pieces::const_iterator begin() const
    {
        return pieces.begin();
    }

    inline typename Pieces::const_iterator end() const
    {
        return pieces.end();
    }

    inline typename Pieces::iterator begin()
    {
        return pieces.begin();
    }

    inline typename Pieces::iterator end()
    {
        return pieces.end();
    }

    inline void reserve(const int &n)
    {
        pieces.reserve(n);
        return;
    }

    inline void emplace_back(const CubicPolynomial &piece)
    {
        pieces.emplace_back(piece);
        return;
    }

    inline void emplace_back(const double &dur,
                             const Eigen::Matrix<double, -1, 4> &cMat)
    {
        pieces.emplace_back(dur, cMat);
        return;
    }

    inline void append(const CubicCurve &traj)
    {
        pieces.insert(pieces.end(), traj.begin(), traj.end());
        return;
    }

    inline int locatePieceIdx(double &t) const
    {
        const int N = getPieceNum();
        int idx;
        double dur;
        for (idx = 0;
             idx < N &&
             t > (dur = pieces[idx].getDuration());
             idx++)
        {
            t -= dur;
        }
        if (idx == N)
        {
            idx--;
            t += pieces[idx].getDuration();
        }
        return idx;
    }

    inline Eigen::VectorXd getPos(double t) const
    {
        const int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getPos(t);
    }

    inline Eigen::VectorXd getVel(double t) const
    {
        const int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getVec(t);
    }

    inline Eigen::VectorXd getAcc(double t) const
    {
        const int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getAcc(t);
    }

    inline Eigen::VectorXd getJerk(double t) const
    {
        const int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getJerk(t);
    }

    inline Eigen::VectorXd getJuncPos(const int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getCoeffMat().col(3);
        }
        else
        {
            return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
        }
    }
};

#endif
