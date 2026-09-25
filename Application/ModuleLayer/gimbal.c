#include "gimbal.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "rp_math.h"

static void Gimbal_Init(Gimbal_t* gimbal);
static uint8_t Board_Is_Online(void);
static void Gimbal_Info_Update(Gimbal_t* gimbal);
static void Gimbal_Target_Update(Gimbal_t* gimbal);
static void Gimbal_Car_State_Update(Gimbal_t* gimbal);
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal);
static void Gimbal_Work(Gimbal_t* gimbal);

Gimbal_t gimbal = {
	.mode = G_SLEEP,
	.init = Gimbal_Init,
};

static void Gimbal_Init(Gimbal_t* gimbal)
{
	gimbal->work = Gimbal_Work;
}

static uint8_t Board_Is_Online(void)
{
	if(board.status->last_rx_tick == 0)
	{
		return 0;
	}

	if(HAL_GetTick() - board.status->last_rx_tick > BOARD_LOST_TICK)
	{
		return 0;
	}

	return 1;
}

static void Gimbal_Info_Update(Gimbal_t* gimbal)
{
	gimbal->info.yaw_imu = board.rx_meg->gimbal_meg.yaw_imu;
}

static void Gimbal_Target_Update(Gimbal_t* gimbal)
{
	if(Board_Is_Online() == 0)
	{
		gimbal->init_flag = 0;
		gimbal->mode = G_SLEEP;
		return;
	}

	if(rc_sensor_info.s1.value != RC_SW_DOWN)
	{
		gimbal->init_flag = 0;
		gimbal->mode = G_INIT;
		return;
	}

	gimbal->mode = G_GYRO;

	if(gimbal->init_flag == 0)
	{
		gimbal->init_flag = 1;
		gimbal->target.yaw_imu_tar = gimbal->info.yaw_imu;
		return;
	}

	if(HAL_GetTick() - rc_sensor_info.last_rx_tick > GIMBAL_RC_LOST_TICK)
	{
		return;
	}

	gimbal->target.yaw_imu_tar -= RC_RIGH_CH_LR_VALUE / 660.f * GIMBAL_YAW_STEP;
	gimbal->target.yaw_imu_tar = motor_half_cycle(gimbal->target.yaw_imu_tar, 360.f);
}

static void Gimbal_Car_State_Update(Gimbal_t* gimbal)
{
	uint8_t car_state = CAR_SLEEP;

	if(gimbal->init_flag != 0 && rc_sensor_info.s1.value == RC_SW_DOWN)
	{
		car_state = CAR_RC;
	}

	board.tx_pkt->car_pkt.car_state = car_state;
	board.tx_pkt->car_pkt.gimbal_mode = (gimbal->mode == G_GYRO) ? 1 : 0;
}

static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal)
{
	board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = gimbal->target.yaw_imu_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = 0.f;
	board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = 0.f;
	board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = 0.f;
}

static void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Info_Update(gimbal);
	Gimbal_Target_Update(gimbal);
	Gimbal_Car_State_Update(gimbal);
	Gimbal_Cmd_Transmit(gimbal);
}
