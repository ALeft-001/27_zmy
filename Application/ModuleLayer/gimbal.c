#include "gimbal.h"
#include "board.h"
#include "motor.h"
#include "imu_sensor.h"
#include "PID.h"
#include "rp_math.h"

static void Gimbal_Target_Update(Gimbal_t* gimbal);
static void Gimbal_Tx_Data_Update(Gimbal_t* gimbal);
static uint8_t Gimbal_Should_Coast(Gimbal_t* gimbal);
static void Gimbal_Torque_Zero(Gimbal_t* gimbal);
static void Gimbal_Pid_Calculate(Gimbal_t* gimbal);
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal);

Gimbal_t gimbal;

static void Gimbal_Target_Update(Gimbal_t* gimbal)
{
	if(Board_Is_Online(&board) == 0)
	{
		return;
	}

	if(board.rx_data->car_data.car_state == CAR_SLEEP)
	{
		return;
	}

	gimbal->target.yaw_target = board.rx_data->gimbal_target_data.yaw_imu_tar;
}

static void Gimbal_Tx_Data_Update(Gimbal_t* gimbal)
{
	board.tx_data->gimbal_data.yaw_imu   = imu_sensor.info->base_info.yaw;
	board.tx_data->gimbal_data.pitch_imu = imu_sensor.info->base_info.pitch;
	board.tx_data->gimbal_data.yaw_mec   = Yaw_Motor.rx_data->motor_angle;
	board.tx_data->gimbal_data.pitch_mec = 0.f;

	board.tx_data->state_data.yaw_motor_state = (uint8_t)Yaw_Motor.state_data->status;
	board.tx_data->state_data.r_fric_state    = (uint8_t)R_Fric.state->status;
	board.tx_data->state_data.is_down         = 0;
}

static uint8_t Gimbal_Should_Coast(Gimbal_t* gimbal)
{
	if(Board_Is_Online(&board) == 0)
	{
		return 1;
	}

	if(board.rx_data->car_data.car_state == CAR_SLEEP)
	{
		return 1;
	}

	return 0;
}

static void Gimbal_Torque_Zero(Gimbal_t* gimbal)
{
	gimbal->out.yaw_out = 0.f;

	Yaw_Motor.tx_data->torque = 0.f;

	if(Yaw_Motor.state_data->motor_state == Motor_Enable)
	{
		DM_Set_Torque(&Yaw_Motor);
	}
	else
	{
		DM_Motor_Sleep(&Yaw_Motor);
	}
}

static void Gimbal_Pid_Calculate(Gimbal_t* gimbal)
{
	float err = motor_half_cycle(gimbal->target.yaw_target - imu_sensor.info->base_info.yaw, 360.f);

	yaw_gyro_outer.err = err;
	single_pid_ctrl(&yaw_gyro_outer);

	yaw_gyro_inner.target = yaw_gyro_outer.out;
	yaw_gyro_inner.err = yaw_gyro_inner.target - imu_sensor.info->base_info.rate_yaw;
	single_pid_ctrl(&yaw_gyro_inner);

	gimbal->out.yaw_out = yaw_gyro_inner.out;
}

static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal)
{
	Yaw_Motor.tx_data->torque = gimbal->out.yaw_out;

	DM_Set_Torque(&Yaw_Motor);
}

void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Target_Update(gimbal);
	Gimbal_Tx_Data_Update(gimbal);

	if(Gimbal_Should_Coast(gimbal) != 0)
	{
		gimbal->work_flag = 0;
		Gimbal_Torque_Zero(gimbal);
		return;
	}

	if(gimbal->work_flag == 0)
	{
		gimbal->work_flag = 1;

		yaw_gyro_outer.integral = 0.f;
		yaw_gyro_inner.integral = 0.f;
	}

	Gimbal_Pid_Calculate(gimbal);
	Gimbal_Cmd_Transmit(gimbal);
}
