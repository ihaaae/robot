#ifndef DUAL_ARM_APP_JUXIE_STATE_HPP
#define DUAL_ARM_APP_JUXIE_STATE_HPP

#include "juxie_controller.h"
#include "bot_executor/ExecutorJuxie.hpp"
#include <vector>
#include "bot_path_planner/path_planner.hpp"
#include <Eigen/Dense>
namespace Juxie {
    enum RobotState {
        power_off = 0,
        ready,
        idle,
        running,
        fault
    };
    
    using CartesianSpaceData = std::array<double, 14>;
    namespace bot_path_planner  {
        class PathPlanner;
    }
    class State {
    private:
        class PathPlanner ;
    protected:
        static ControllerJuxieImpl *m_impl;
        RobotState m_state {power_off};
    public:
        explicit State() = default;

        ~State() = default;

    public:
        void updateImpl(ControllerJuxieImpl * impl) {
            m_impl = impl;
        }

        virtual bool SelfCheck() = 0;

        RobotState GetStateEnum() {
            return m_state;
        };

        virtual bool OnRobot() = 0;

        virtual bool OffRobot() = 0;

        virtual bool EnableRobot() = 0;

        virtual bool DisableRobot() = 0;

        virtual bool Stop() = 0;

        virtual bool ClearFault();

        virtual JointSpaceData GetJointPositions();

        virtual ArmSpaceData GetSingleTorques();

        virtual CartesianSpaceData GetTCPPose();
        virtual CartesianSpaceDataRPY getFKpose(const std::vector<double>& joint_angles,
                           const int left_num, 
                           const int right_num);

        virtual int getDof();

        virtual bool setJointZeroPosition();

        virtual int
        MoveJ_Canfd(const JointSpaceData &joint_position, int freq) = 0;
        
        virtual int
        MoveP_Canfd(const Juxie::CartesianSpaceData &pose, int freq) = 0;

        virtual int
        MoveJ(const JointSpaceData &joint_position, int v) = 0;

        virtual int
        MoveJ_P(const CartesianSpaceData &tcp_pose, int v) = 0;

        virtual int
        MoveL(const CartesianSpaceData &tcp_pose, int v) = 0;

        virtual int
        MoveEnd(double v,int part) = 0;
        
        virtual int
        BreakEngage(const std::vector<int>&numbers)=0;

        virtual int
        BreakRelease(const std::vector<int>&numbers)=0;
        
        virtual int
        IK(const CartesianSpaceData &tcp_pose,Eigen::VectorXd& joint) = 0;

    protected:
        bool m_on_robot();

        bool m_off_robot();

        bool m_restartStation();

        bot_executor::ExecutorPtr getExecutorPtr();

        int
        m_moveJ_Canfd(const JointSpaceData &joint_position, int freq);

        int
        m_moveP_Canfd(const Juxie::CartesianSpaceData &pose, int freq);

        int
        m_moveL(const CartesianSpaceData &tcp_pose, int v);

        int
        m_MoveEnd(double v,int part);

        int
        m_BreakRelease(const std::vector<int>&numbers);

        int
        m_BreakEngage(const std::vector<int>&numbers);
        
        int
        i_IK(const CartesianSpaceData &tcp_pose,Eigen::VectorXd &joint);
        
        int m_moveJ(const JointSpaceData &joint_positions, int v);

        int m_MoveJ_P(const CartesianSpaceData &tcp_pose, int v);
    private:
        bot_path_planner ::PathPlanner* planner_ptr_ = nullptr;
        std::shared_ptr<bot_path_planner ::PathPlanner> planner_shared_ptr_;

    };
    ControllerJuxieImpl *State::m_impl = nullptr;
    typedef std::shared_ptr<State> StatePtr;

#define DECLARE_STATE_CLASS(StateName, StateEnum) \
class StateName : public State { \
public: \
    explicit StateName() : State() { \
        m_state = RobotState::StateEnum; \
    }                                             \
    bool SelfCheck() override;                    \
    bool OnRobot() override; \
    bool OffRobot() override; \
    bool EnableRobot() override; \
    bool DisableRobot() override; \
    bool Stop() override; \
    int MoveJ_Canfd(const JointSpaceData &joint_position, int freq) override; \
    int MoveP_Canfd(const Juxie::CartesianSpaceData &pose, int freq) override; \
    int MoveJ(const JointSpaceData &joint_position, int v) override; \
    int MoveJ_P(const CartesianSpaceData &tcp_pose, int v) override; \
    int MoveL(const CartesianSpaceData &tcp_pose, int v) override; \
    int MoveEnd(double v,int part) override; \
    int BreakEngage(const std::vector<int>&numbers) override; \
    int BreakRelease(const std::vector<int>&numbers) override; \
    int IK(const CartesianSpaceData &tcp_pose ,Eigen::VectorXd &joint) override;\
};

    DECLARE_STATE_CLASS(StatePowerOff, power_off);

    DECLARE_STATE_CLASS(StateReady, ready);

    DECLARE_STATE_CLASS(StateIdle, idle);

    DECLARE_STATE_CLASS(StateRunning, running);

    DECLARE_STATE_CLASS(StateFault, fault);

}


#endif //DUAL_ARM_APP_JUXIE_STATE_HPP
