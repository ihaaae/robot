#ifndef RK3576_CAN_CANFD_H 
#define RK3576_CAN_CANFD_H
// #include <fmt/format.h>
#include <cstdint>
#include <utility>
#include <functional>
#include <chrono>
#include <memory>
#include <deque>
#include <shared_mutex>
#include <vector>


#define BIT(nr)				(1UL << (nr))

#define CAN0_PHYADDR			0x2AC00000
#define CAN1_PHYADDR			0x2AC10000
#define CAN_ADDR_LEN			0x10000

#define ISM_WATERMASK_CANFD		0x6c
#define BUSOFF_RCY_CNT_FAST     	4
#define BUSOFF_RCY_TIME_FAST		0x3d0900 /* 40ms : cnt * (1 / can_clk) */

#define INT_ENABLE			BIT(0)
#define STORAGE_TIMEOUT_MODE		BIT(8)
#define RX_DMA_ENABLE			BIT(9)
#define BUSOFF_RCY_MODE_EN      	BIT(8)
#define WORK_MODE			BIT(0)

#define CANFD_TX0_REQ			BIT(0)
#define CANFD_TX1_REQ			BIT(1)
#define CANFD_TX_REQ_FULL		((CANFD_TX0_REQ) | (CANFD_TX1_REQ))

#define INT_ENABLE				BIT(0)

/* registers definition */
#define	CANFD_MODE  			0x00
#define	CANFD_CMD 				0x04
#define CANFD_STATE 			0x08
#define CANFD_INT 				0x0c
#define CANFD_RXFRD 			0x400
#define CANFD_STR_STATE 		0x604
#define	CANFD_RXERRORCNT  		0x910
#define	CANFD_TXERRORCNT  		0x914
#define	CANFD_INT_MASK 			0x10
#define	CANFD_ATF0 			0x700
#define	CANFD_ATF1 			0x704
#define	CANFD_ATF2 			0x708
#define	CANFD_ATF3 			0x70c
#define	CANFD_ATF4 			0x710
#define	CANFD_ATFM0 			0x714
#define	CANFD_ATFM1 			0x718
#define	CANFD_ATFM2 			0x71c
#define	CANFD_ATFM3 			0x720
#define	CANFD_ATFM4 			0x724
#define	CANFD_STR_CTL 			0x600
#define	CANFD_STR_WTM 			0x60c
#define	CANFD_DMA_CTRL 			0x11c
#define CANFD_BRS_CFG 			0x10c
#define	CANFD_BUSOFFRCY_CFG 		0x830
#define	CANFD_BUSOFF_RCY_THR 		0x834
#define	CANFD_NBTP 			0x100
#define	CANFD_DBTP 			0x104
#define	CANFD_TDCR 			0x108
#define	CANFD_TXFIC 			0x200
#define	CANFD_TXID 			0x204
#define	CANFD_TXDAT0 			0x208
#define	CANFD_TXDAT1 			0x20c
#define	CANFD_RXFIC 			0x300
#define	CANFD_RXID 			0x304
#define	CANFD_RXTS 			0x308
#define	CANFD_RXDAT0 			0x30c
#define	CANFD_RXDAT1 			0x310

#define CAN_EFF_FLAG 		0x80000000U 
#define CAN_SFF_MASK 		0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 		0x1FFFFFFFU /* extended frame format (EFF) */
#define TX_DLC_SHIFT		0
#define TX_DLC_MASK			(0xF << TX_DLC_SHIFT)

#define TX_BRS				BIT(4)
#define TX_FDF				BIT(5)
#define TX_RTR				BIT(6)
#define TX_EXR				BIT(7)
#define TX_CANFD_MAX		0xF

/*the real time priority */
#define SCHED_PRIORITY 	90

/*the sleep time,ns*/
// #define SLEEP_TIME 		5000000L   /*发送周期*/
#define SLEEP_TIME 		200000L   /*发送周期*/
#define USEC_PER_SEC    1000000L
#define CAN0_BUFF_LEN			200 /*byte*/
#define CAN1_BUFF_LEN			200 /*byte*/
#define CAN0_SEND_LEN			8 /*byte*/
#define CAN1_SEND_LEN			8 /*byte*/
#define SEND_MAX				64 	

#define CANFD_MAX_DLEN			64
#define CAN_MAX_DLEN 8

#define CANFD_BRS 				0x01 /* bit rate switch (second bitrate for payload data) */
#define RX_FIFO_MASK			0x1ff00
#define RX_FIFO_SHIFT			0x8
#define RX_MAX_DATA				0x12
#define RX_FIFO_DEPTH			0xe
#define INTM_CNT_SHIFT			17
#define INTM_CNT_MASK			(0x1ff << INTM_CNT_SHIFT)
#define RX_DLC_SHIFT			24
#define RX_DLC_MASK				(0xF << RX_DLC_SHIFT)
#define RX_FORMAT_SHIFT			23
#define RX_FORMAT_MASK			(0x1 << RX_FORMAT_SHIFT)
#define CAN_EFF_FLAG 			0x80000000U /* EFF/SFF is set in the MSB */
#define RX_RTR_SHIFT			22
#define RX_RTR_MASK				(0x1 << RX_RTR_SHIFT)
#define CAN_RTR_FLAG 			0x40000000U /* remote transmission request */
#define RX_BRS_SHIFT			20
#define RX_BRS_MASK				(0x1 << RX_BRS_SHIFT)
#define RX_FDF_SHIFT		21
#define RX_FDF_MASK		(0x1 << RX_FDF_SHIFT)

#define min_t(type, x, y) ({                    \
        type __min1 = (x);                      \
        type __min2 = (y);                      \
        __min1 < __min2 ? __min1 : __min2; })
#define can_cc_dlc2len(dlc)	(min_t(u_int8_t, (dlc), CAN_MAX_DLEN))

#define DEVICE_PATH_CAN0 		"/dev/misc_shm_can0"
#define DEVICE_PATH_CAN1 		"/dev/misc_shm_can1"
#define SHM_SIZE 				(4096)

#define ENABLE_FPRINTF 			0

typedef enum{
	CAN0 = 0,
	CAN1
}Can_If;

typedef enum{
	CAN,
	CANFD
}Can_T;

/**
 * struct canfd_frame - CAN flexible data rate frame structure
 * @can_id: CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @len:    frame payload length in byte (0 .. CANFD_MAX_DLEN)
 * @flags:  additional flags for CAN FD
 * @__res0: reserved / padding
 * @__res1: reserved / padding
 * @data:   CAN FD frame payload (up to CANFD_MAX_DLEN byte)
 */
struct canfd_frame {
	u_int32_t 	can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	u_int8_t    len;     /* frame payload length in byte */
	u_int8_t    flags;   /* additional flags for CAN FD */
	u_int8_t    __res0;  /* reserved / padding */
	u_int8_t    __res1;  /* reserved / padding */
	u_int8_t    data[CANFD_MAX_DLEN];
};

/** CAN2.0-frame */
typedef struct {
	Can_If can;
	u_int32_t 	can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	u_int8_t    len;     /* frame payload length in byte */
	u_int8_t    data[CAN0_SEND_LEN];
} CAN_MSG;

typedef int16_t err_t;

struct JointState
{
    u_int8_t MotionState[7];
	u_int8_t ControlType[7];
    // u_int16_t MotionEnable[7];
    u_int16_t Current[7];
	// u_int16_t Vel[7];
	int16_t Vel[7];
	int single_torque[7];
	float six_axis_torque[7];
	u_int16_t FaultData[7];
    int origPosAct[7];
	bool isUpdated;
};

enum class JointControlType
{
	EnabledJoint = 0,
	DisEnabledJioint,
	ClearFault,
	StopMotion,
};

enum class BreakType
{
	BreakEngage = 0,
	BreakRelease,
};

namespace rk3576_can_canfd
{
	class RK3576CanCanfd
	{
		public:
			virtual ~RK3576CanCanfd() throw();
			virtual err_t hal_start_can() = 0;		
			virtual err_t can_send_frame(Can_If can,u_int32_t can_id,u_int32_t length, u_int8_t *data) = 0; //非阻塞
			virtual void setReadFunction(std::function<void(JointState&, JointState&)> f) = 0;
			virtual bool sendJointControl(JointControlType type) = 0;
			virtual bool sendJointBreak(const std::vector<int>&numbers,BreakType type) = 0;
			virtual void setJointZeroPosition() = 0;
			virtual void ReadFaultData()=0;
			virtual void initLog(bool isPintf) = 0;
			virtual void getMotorCount(u_int8_t &motionCAN0, u_int8_t &motionCAN1) = 0;
			static std::shared_ptr<RK3576CanCanfd> create(int arm_type, int dof);
	};
	typedef std::shared_ptr<RK3576CanCanfd> RK3576CanCanfdPtr;
}

#endif
