#include "gimbal.h"
#include "board.h"
#include "rc_sensor.h"
#include "rp_math.h"

static void Gimbal_Data_Update(Gimbal_t* gimbal);
static void Gimbal_Target_Update(Gimbal_t* gimbal);
static void Gimbal_Car_State_Update(Gimbal_t* gimbal);
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal);
static void Chassis_Data_Update(Chassis_yaw_t* chassis_yaw);

Gimbal_t gimbal = {	.mode = G_SLEEP,};
Chassis_yaw_t chassis_yaw;
extern imu_sensor_t imu_sensor;
extern Board_Rx_Data_t board_rx_data;

static void Gimbal_Data_Update(Gimbal_t* gimbal)
{
	gimbal->data.yaw_imu = board.rx_data->gimbal_data.yaw_imu;
}

static void Chassis_Data_Update(Chassis_yaw_t* chassis_yaw)
{
	chassis_yaw->yaw_last = chassis_yaw->yaw_current;
	chassis_yaw->yaw_current = imu_sensor.info->base_info.yaw;
}

static float Gimbal_Get_Front(Gimbal_t* gimbal)
{
	float front = gimbal->data.yaw_imu + motor_half_cycle(GIMBAL_YAW_FRONT - board.rx_data->gimbal_data.yaw_mec, 25) * (180.f / PI);
	return motor_half_cycle(front,360.f);
}

static void Gimbal_Yaw_Update(Gimbal_t* gimbal)
{
	if(rc_sensor_info.s2.value == RC_SW_MID)
	{
		gimbal->target.yaw_imu_tar = Gimbal_Get_Front(gimbal);
		return;
	}

	if(rc_sensor_info.s2.value == RC_SW_UP)
	{
		if(rc_sensor_info.s2.value_last != RC_SW_UP)
		{
			gimbal->target.yaw_imu_tar = Gimbal_Get_Front(gimbal);
		}
	}
	gimbal->target.yaw_imu_tar -= RC_RIGH_CH_LR_VALUE / 660.f * GIMBAL_YAW_STEP;
	gimbal->target.yaw_imu_tar = motor_half_cycle(gimbal->target.yaw_imu_tar, 360.f);
}

static void Gimbal_Pitch_Update(Gimbal_t* gimbal)
{
	gimbal->target.pitch_mec_tar += RC_RIGH_CH_UD_VALUE / 660.f * GIMBAL_PITCH_STEP_RAD;
	gimbal->target.pitch_mec_tar = constrain(gimbal->target.pitch_mec_tar,GIMBAL_PITCH_MIN,GIMBAL_PITCH_MAX);
}

static void Gimbal_Target_Update(Gimbal_t* gimbal)
{
	if(Board_Is_Online(&board) == 0)
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
		gimbal->target.yaw_imu_tar = Gimbal_Get_Front(gimbal); 
		gimbal->target.pitch_mec_tar = GIMBAL_PITCH_MIDDLE;   
		return;
	}

	if(HAL_GetTick() - rc_sensor_info.last_rx_tick > GIMBAL_RC_LOST_TICK)
	{
		return;
	}

	Gimbal_Yaw_Update(gimbal);
	Gimbal_Pitch_Update(gimbal);
}

static void Gimbal_Car_State_Update(Gimbal_t* gimbal)
{
	uint8_t car_state = CAR_SLEEP;

	if(gimbal->init_flag != 0 && rc_sensor_info.s1.value == RC_SW_DOWN)
	{
		car_state = CAR_RC;
	}

	board.tx_data->car_data.car_state = car_state;
}

static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal)
{
	board.tx_data->gimbal_target_data.yaw_imu_tar = gimbal->target.yaw_imu_tar;
	board.tx_data->gimbal_target_data.pitch_mec_tar = gimbal->target.pitch_mec_tar;
}

void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Data_Update(gimbal);
	Chassis_Data_Update(&chassis_yaw);
	Gimbal_Target_Update(gimbal);
	Gimbal_Car_State_Update(gimbal);
	Gimbal_Cmd_Transmit(gimbal);
}
