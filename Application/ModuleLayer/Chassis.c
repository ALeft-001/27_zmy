#include "chassis.h"
#include "rc_sensor.h"
#include "config_chassis.h"
#include "rp_math.h"
#include "pid.h"

static void Chassis_Target_Clear(Chassis_t* chassis);
static uint8_t Chassis_Should_Coast(Chassis_t* chassis);
static void Chassis_Torque_Zero(Chassis_t* chassis);
static void Chassis_Target_Update(Chassis_t* chassis);
static void Chassis_Inverse_Calculate(Chassis_t* chassis);
static void Chassis_Pid_Calculate(Chassis_t* chassis);
static void Chassis_Cmd_Transmit(Chassis_t* chassis);

int8_t chassis_wheel_sign[WHEEL_CNT] = {1, 1, 1, 1};

Chassis_t chassis = {
	.wheel = &wheel_set,
};

static void Chassis_Target_Clear(Chassis_t* chassis)
{
	chassis->target.front_speed = 0;
	chassis->target.left_speed = 0;
	chassis->target.cycle_speed = 0;
}

static uint8_t Chassis_Should_Coast(Chassis_t* chassis)
{
	if(HAL_GetTick() - rc_sensor_info.last_rx_tick <= RC_LOST_TICK &&
		rc_sensor_info.s1.value == RC_SW_UP)
	{
		return 1;
	}

	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		if(HAL_GetTick() - chassis->wheel->motor[i]->rx_data->last_rx_tick > WHEEL_LOST_TICK)
		{
			return 1;
		}
	}

	return 0;
}

static void Chassis_Torque_Zero(Chassis_t* chassis)
{
	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->wheel->motor[i]->tx_data->torque = 0;
		chassis->wheel->motor[i]->tx_data->torque_current = 0;
		chassis->wheel->motor[i]->tx_data->torque_current_raw = 0;
		chassis->out.wheel_initial_out[i] = 0;
		chassis->out.wheel_end_out[i] = 0;
	}

	RM_Set_Torque(chassis->wheel);
}

static void Chassis_Target_Update(Chassis_t* chassis)
{
	if(HAL_GetTick() - rc_sensor_info.last_rx_tick > RC_LOST_TICK)
	{
		Chassis_Target_Clear(chassis);
		return;
	}

	switch(rc_sensor_info.s1.value)
	{
		case RC_SW_DOWN:
			break;

		case RC_SW_MID:
		case RC_SW_UP:
		default:
			Chassis_Target_Clear(chassis);
			return;
	}

	chassis->target.front_speed = -RC_LEFT_CH_UD_VALUE / 660.f * FRONT_MAX_SPEED;
	chassis->target.left_speed = RC_LEFT_CH_LR_VALUE / 660.f * LEFT_MAX_SPEED;
	chassis->target.cycle_speed = (rc_sensor_info.s2.value == RC_SW_UP) ? TURN_CYCLE_SPEED : 0;
}

static void Chassis_Inverse_Calculate(Chassis_t* chassis)
{
	float front = chassis->target.front_speed;
	float left = chassis->target.left_speed;
	float cycle = chassis->target.cycle_speed;

	float trans = abs(front) + abs(left);
	float rotate = abs(cycle);

	if(trans + rotate > CHASSIS_MAX_SPEED)
	{
		float rotate_limit = CHASSIS_MAX_SPEED * 0.6f;

		if(rotate > rotate_limit)
		{
			cycle *= rotate_limit / rotate;
		}

		if(trans > 0.f)
		{
			float k = (CHASSIS_MAX_SPEED - abs(cycle)) / trans;

			if(k < 1.f)
			{
				front *= k;
				left *= k;
			}
		}
	}

	chassis->target.front_speed = front;
	chassis->target.left_speed = left;
	chassis->target.cycle_speed = cycle;

	chassis->target.motor_speed[WHEEL_LF] = (- front + left + cycle) * chassis_wheel_sign[WHEEL_LF];
	chassis->target.motor_speed[WHEEL_LB] = (- front - left + cycle) * chassis_wheel_sign[WHEEL_LB];
	chassis->target.motor_speed[WHEEL_RF] = (  front + left + cycle) * chassis_wheel_sign[WHEEL_RF];
	chassis->target.motor_speed[WHEEL_RB] = (  front - left + cycle) * chassis_wheel_sign[WHEEL_RB];

	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->target.motor_speed[i] = constrain(chassis->target.motor_speed[i], -WHEEL_MAX_SPEED, WHEEL_MAX_SPEED);
	}
}

static void Chassis_Pid_Calculate(Chassis_t* chassis)
{
	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->out.wheel_initial_out[i] = all_pid_calc(NULL,
			chassis->wheel->motor[i]->ctrl_data->speed_ctrl,
			chassis->target.motor_speed[i],
			0,
			chassis->wheel->motor[i]->rx_data->speed,
			0,
			ERR_MODE_NONE);
	}
}

static void Chassis_Cmd_Transmit(Chassis_t* chassis)
{
	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->out.wheel_end_out[i] = chassis->out.wheel_initial_out[i];
		chassis->wheel->motor[i]->tx_data->torque = chassis->out.wheel_end_out[i];
	}

	RM_Set_Torque(chassis->wheel);
}

void Chassis_Work(Chassis_t* chassis)
{
	Chassis_Target_Update(chassis);

	if(Chassis_Should_Coast(chassis) != 0)
	{
		Chassis_Torque_Zero(chassis);
		return;
	}

	Chassis_Inverse_Calculate(chassis);
	Chassis_Pid_Calculate(chassis);
	Chassis_Cmd_Transmit(chassis);
}
