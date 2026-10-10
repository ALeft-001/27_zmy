#include "gimbal.h"
#include "Lift.h"
#include "Chassis.h"
#include "board.h"
#include "rc_sensor.h"
#include "rp_math.h"

static void Gimbal_Data_Update(Gimbal_t* gimbal);
static void Gimbal_Turn_Set(Gimbal_t* gimbal);
static void Gimbal_Turn_Update(Gimbal_t* gimbal);
static void Gimbal_Key_Update(Gimbal_t* gimbal);
static void Gimbal_Target_Update(Gimbal_t* gimbal);
static void Gimbal_Car_State_Update(Gimbal_t* gimbal);
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal);

Gimbal_t gimbal = {	.mode = G_SLEEP,};
extern Board_Rx_Data_t board_rx_data;

uint8_t rc_lost_lock = 0;
static uint8_t rc_mid_seen = 0;

static void Gimbal_Data_Update(Gimbal_t* gimbal)
{
	gimbal->data.yaw_imu = board.rx_data->gimbal_data.yaw_imu;
}

static int16_t gimbal_tw_last = 0;
static uint8_t gimbal_mode_flip = 0;
static uint8_t gimbal_g_last = 0;
static uint8_t gimbal_z_last = 0;

static void Gimbal_Turn_Set(Gimbal_t* gimbal)
{
	gimbal->turn_back = (gimbal->turn_back == 0) ? 1 : 0;
	gimbal->target.yaw_mec_tar = (gimbal->turn_back != 0) ? GIMBAL_YAW_BACK : GIMBAL_YAW_FRONT;
}

static void Gimbal_Turn_Update(Gimbal_t* gimbal)
{
	int16_t tw = rc_sensor_info.thumbwheel.value;

	if(rc_sensor_info.s1.value != RC_SW_DOWN
		|| rc_sensor_info.s2.value != RC_SW_MID
		|| gimbal->target.yaw_mode != 0
		|| HAL_GetTick() - rc_sensor_info.last_rx_tick > GIMBAL_RC_LOST_TICK)
	{
		if(gimbal->turn_back != 0)
		{
			gimbal->turn_back = 0;
			gimbal->target.yaw_mec_tar = GIMBAL_YAW_FRONT;
		}

		gimbal_tw_last = tw;
		return;
	}

	if(tw <= GIMBAL_TW_UP && gimbal_tw_last > GIMBAL_TW_UP)
	{
		Gimbal_Turn_Set(gimbal);
	}

	gimbal_tw_last = tw;
}

static void Gimbal_Key_Update(Gimbal_t* gimbal)
{
	uint8_t g = (rc_sensor_info.G.value != 0) ? 1 : 0;
	uint8_t z = (rc_sensor_info.Z.value != 0) ? 1 : 0;

	if(g != 0 && gimbal_g_last == 0 && lift_lower_lock == 0)
	{
		gimbal_mode_flip = (gimbal_mode_flip == 0) ? 1 : 0;
	}

	if(z != 0 && gimbal_z_last == 0
		&& rc_sensor_info.s1.value == RC_SW_DOWN
		&& rc_sensor_info.s2.value == RC_SW_MID
		&& gimbal->target.yaw_mode == 0
		&& lift.yaw_lock == 0
		&& lift_lower_lock == 0)
	{
		Gimbal_Turn_Set(gimbal);
	}

	gimbal_g_last = g;
	gimbal_z_last = z;
}

static void Gimbal_Yaw_Update(Gimbal_t* gimbal)
{
	uint8_t yaw_mode = (rc_sensor_info.s2.value == RC_SW_MID) ? 0 : 1;
	uint8_t yaw_free = 0;

	if(gimbal_mode_flip != 0)
	{
		yaw_mode = (yaw_mode == 0) ? 1 : 0;
	}

	if(lift.yaw_lock != 0
		|| gimbal->center_done == 0
		|| lift_lower_lock != 0)
	{
		yaw_mode = 0;
	}

	if(yaw_mode == 0
		&& lift.yaw_lock == 0
		&& lift_lower_lock == 0
		&& gimbal->center_done != 0
		&& gimbal_mode_flip != 0)
	{
		yaw_free = 1;
	}

	if(yaw_mode != gimbal->target.yaw_mode)
	{
		gimbal->target.yaw_imu_tar = gimbal->data.yaw_imu
			+ motor_half_cycle(GIMBAL_YAW_FRONT - board.rx_data->gimbal_data.yaw_mec, 2.f * PI) / PI * 180.f;
		gimbal->target.yaw_mec_tar = (gimbal->turn_back != 0) ? GIMBAL_YAW_BACK : GIMBAL_YAW_FRONT;
	}

	gimbal->target.yaw_mode = yaw_mode;

	if(yaw_mode == 0)
	{
		if(lift.yaw_lock != 0 || lift_lower_lock != 0)
		{
			gimbal->turn_back = 0;
			gimbal->target.yaw_mec_tar = GIMBAL_YAW_FRONT;
		}
		else if(yaw_free != 0)
		{
			gimbal->target.yaw_mec_tar -= (float)rc_sensor_info.mouse_vx / GIMBAL_MOUSE_FULL_X * (GIMBAL_YAW_STEP * PI / 180.f);
			gimbal->target.yaw_mec_tar = motor_half_cycle(gimbal->target.yaw_mec_tar, 2.f * PI);
		}

		rc_sensor_info.mouse_vx = 0;
		return;
	}

	gimbal->target.yaw_imu_tar -= RC_RIGH_CH_LR_VALUE / 660.f * GIMBAL_YAW_STEP;
	gimbal->target.yaw_imu_tar -= (float)rc_sensor_info.mouse_vx / GIMBAL_MOUSE_FULL_X * GIMBAL_YAW_STEP;
	rc_sensor_info.mouse_vx = 0;
	gimbal->target.yaw_imu_tar = motor_half_cycle(gimbal->target.yaw_imu_tar, 360.f);

	if(rc_sensor_info.s2.value == RC_SW_DOWN
		&& spin_key_active == 0
		&& Board_Is_Online(&board) != 0
		&& fabsf(board.rx_data->gimbal_data.yaw_mec) > 1e-5f)
	{
		float front_imu = gimbal->data.yaw_imu
			+ motor_half_cycle(GIMBAL_YAW_FRONT - board.rx_data->gimbal_data.yaw_mec, 2.f * PI) / PI * 180.f;
		float lag = motor_half_cycle(gimbal->target.yaw_imu_tar - front_imu, 360.f);

		if(lag > GIMBAL_FOLLOW_LAG_MAX)
		{
			gimbal->target.yaw_imu_tar = motor_half_cycle(front_imu + GIMBAL_FOLLOW_LAG_MAX, 360.f);
		}
		else if(lag < -GIMBAL_FOLLOW_LAG_MAX)
		{
			gimbal->target.yaw_imu_tar = motor_half_cycle(front_imu - GIMBAL_FOLLOW_LAG_MAX, 360.f);
		}
	}
}

static void Gimbal_Pitch_Update(Gimbal_t* gimbal)
{
	gimbal->target.pitch_mec_tar += RC_RIGH_CH_UD_VALUE / 660.f * GIMBAL_PITCH_STEP_RAD;
	gimbal->target.pitch_mec_tar -= (float)rc_sensor_info.mouse_vy / GIMBAL_MOUSE_FULL_Y * GIMBAL_PITCH_STEP_RAD;
	rc_sensor_info.mouse_vy = 0;
	gimbal->target.pitch_mec_tar = constrain(gimbal->target.pitch_mec_tar,GIMBAL_PITCH_MIN,GIMBAL_PITCH_MAX);
}

static void Gimbal_Target_Update(Gimbal_t* gimbal)
{
	Gimbal_Turn_Update(gimbal);
	Gimbal_Key_Update(gimbal);

	if(Board_Is_Online(&board) == 0)
	{
		gimbal->init_flag = 0;
		gimbal->mode = G_SLEEP;
		return;
	}

	if(rc_sensor_info.s1.value != RC_SW_DOWN && rc_sensor_info.s1.value != RC_SW_UP)
	{
		gimbal->init_flag = 0;
		gimbal->mode = G_INIT;
		return;
	}

	gimbal->mode = G_GYRO;

	if(gimbal->init_flag == 0)
	{
		gimbal->init_flag = 1;
		gimbal->center_done = 0;
		gimbal->center_tick = 0;
		gimbal->turn_back = 0;
		gimbal->target.yaw_mode = 0;
		gimbal->target.yaw_mec_tar = GIMBAL_YAW_FRONT;
		gimbal->target.pitch_mec_tar = GIMBAL_PITCH_MIDDLE;
		return;
	}

	if(HAL_GetTick() - rc_sensor_info.last_rx_tick > GIMBAL_RC_LOST_TICK)
	{
		return;
	}

	if(gimbal->center_done == 0)
	{
		gimbal->center_tick ++;

		if(gimbal->center_tick > GIMBAL_CENTER_TIMEOUT
			|| (fabsf(board.rx_data->gimbal_data.pitch_mec - GIMBAL_PITCH_MIDDLE) <= LIFT_PITCH_TOL
				&& fabsf(motor_half_cycle(GIMBAL_YAW_FRONT - board.rx_data->gimbal_data.yaw_mec, 2.f * PI)) <= LIFT_YAW_TOL))
		{
			gimbal->center_done = 1;
		}
	}

	Gimbal_Yaw_Update(gimbal);

	if(lift.pitch_lock != 0 || gimbal->center_done == 0)
	{
		rc_sensor_info.mouse_vy = 0;
		gimbal->target.pitch_mec_tar = GIMBAL_PITCH_MIDDLE;
		return;
	}

	Gimbal_Pitch_Update(gimbal);
}

static void Gimbal_Car_State_Update(Gimbal_t* gimbal)
{
	uint8_t car_state = CAR_SLEEP;

	if(rc_sensor_info.last_rx_tick != 0
		&& HAL_GetTick() - rc_sensor_info.last_rx_tick > GIMBAL_RC_LOST_TICK)
	{
		rc_lost_lock = 1;
		rc_mid_seen = 0;
	}
	else if(rc_sensor_info.s1.value == RC_SW_MID)
	{
		rc_mid_seen = 1;
	}
	else if(rc_mid_seen != 0
		&& (rc_sensor_info.s1.value == RC_SW_DOWN || rc_sensor_info.s1.value == RC_SW_UP))
	{
		rc_lost_lock = 0;
	}

	if(rc_lost_lock == 0
		&& gimbal->init_flag != 0
		&& (rc_sensor_info.s1.value == RC_SW_DOWN || rc_sensor_info.s1.value == RC_SW_UP))
	{
		car_state = CAR_RC;
	}

	board.tx_data->car_data.car_state = car_state;
}

static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal)
{
	board.tx_data->gimbal_target_data.yaw_mode = gimbal->target.yaw_mode;
	board.tx_data->gimbal_target_data.yaw_mec_tar = gimbal->target.yaw_mec_tar;
	board.tx_data->gimbal_target_data.yaw_imu_tar = gimbal->target.yaw_imu_tar;
	board.tx_data->gimbal_target_data.pitch_mec_tar = gimbal->target.pitch_mec_tar;
}

void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Data_Update(gimbal);

	Gimbal_Target_Update(gimbal);
	Gimbal_Car_State_Update(gimbal);
	Gimbal_Cmd_Transmit(gimbal);
}
