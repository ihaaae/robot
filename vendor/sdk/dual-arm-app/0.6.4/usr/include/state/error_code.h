

#ifndef BOT_COMMON_ERROR_CODE_H
#define BOT_COMMON_ERROR_CODE_H

#include <string>

namespace bot_common {

    enum ErrorCode {
        OK = 0,
        Error = -1,
/************************HARDWARE -2 ~ -20********************/
        HeadConnectFailed = -2,
        ArmJointCommunicationFailed = -3,
        ArmJointTargetExceedLimits = -4,
        ArmJointTargetSingular = -5,
        ArmRealTimeKernelWrong = -6,
        ArmJointBusWrong = -7,
        ArmPlanningFailed = -8,
        ArmJointVelExceedLimits = -10,
        ArmEndBoardConnectionWrong = -11,
        ArmVelExceedLimits = -12,
        ArmAccExceedLimits = -13,
        ArmBrakeHold = -14,
        ArmTeachTooFast = -15,
        ArmCollisionHappend = -16,
        ArmNoSuchWorkFrame = -17,
        ArmNoSuchToolFrame = -18,
        ArmNotEnabled = -19,
        ArmControllerTemperatureHigh = -20,
        ArmControllerCurrentHigh = -21,
        ArmControllerCurrentLow = -22,
        ArmControllerVoltageHigh = -23,
        ArmControllerVolatgeLow = -24,
        ArmRealTimeKernelCommunicationWrong = -25,
        /**************Joint Flag *************/
        ArmJointFOCWrong = -26,
        ArmJointVoltageHigh = -27,
        ArmJointVoltageLow = -28,
        ArmJointTemperatureHigh = -29,
        ArmJointSetupFailed = -30,
        ArmJointEncoderWorng = -31,
        ArmJointCurrentHigh = -32,
        ArmJointSoftwareWrong = -33,
        ArmJointTemperatureSensorWrong = -34,
        ArmJointExceedLimits = -35,
        ArmJointIndexWrong = -36,
        ArmJointServoWrong = -37,
        ArmJointCurrentWrong = -38,
        ArmJointBrakeHold = -39,
        ArmJointCmdsStep  = -40,
        ArmJointLosLoop = -41,
        ArmJointConnectionDropFrames = -42,
/***********************SOFTWARRE********************/
        ROSError = -43,
        IkExceedMaxDis = -44,
        IKFailed = -45,
        CartesianPlanningFailed = -46,
        TrajectoryPlanningFailed = -47,
        /***************VALIDATOR -20 ~ ******************/
        EmptyPath = -48,
        InCollision = -49,
        OutLimitation = -50,
        NotValid = -51,
        CartesianPlanningTimeout = -52,


        /**************CONTROL******************/
        ExecuteFailed = -100,
        RobotConnectFailed = -101,
        ServerConnectFailed = -102,
        ArmMoving = -103,
        EmergencyStop = -104,
    };

    class ErrorInfo {

    public:
        ErrorInfo() : error_code_(ErrorCode::OK), error_msg_("all ok") {};

        ErrorInfo(ErrorCode error_code, const std::string &error_msg) : error_code_(error_code),
                                                                        error_msg_(error_msg) {};

        ErrorInfo &operator=(const ErrorInfo &error_info) {
            if (&error_info != this) {
                error_code_ = error_info.error_code_;
                error_msg_ = error_info.error_msg_;
            }
            return *this;
        }

        static ErrorInfo OK() {
            return ErrorInfo();
        }

        ErrorCode error_code() const { return error_code_; };

        const std::string &error_msg() const { return error_msg_; }

        bool operator==(const ErrorInfo &rhs) {
            return error_code_ == rhs.error_code();
        }

        bool IsOK() const {
            return (error_code_ == ErrorCode::OK);
        }

        ~ErrorInfo() = default;

    private:
        ErrorCode error_code_;
        std::string error_msg_;


    };

} //namespace bot_common

#endif //BOT_COMMON_ERROR_CODE_H
