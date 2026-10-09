#include "Lift.h"
#include "motor.h"
#include "gimbal.h"
#include "board.h"
#include "PID.h"
#include "rp_math.h"
#include "math.h"

Lift_t lift =
{
	.state = LIFT_TO_UP,
	.mode  = LIFT_STOP,
};

static void Lift_Target_Update(Lift_t* lift);
static void Lift_Find_Record(Lift_t* lift, float angle);
static void Lift_Block_Check(Lift_t* lift);
static void Lift_State_Machine(Lift_t* lift);
static void Lift_Pid_Calculate(Lift_t* lift);
static void Lift_Stuck_Check(Lift_t* lift);
static void Lift_Tx_Update(Lift_t* lift);
static uint8_t Lift_Get_State(void);

static uint8_t Lift_Get_State(void)
{
	if(HAL_GetTick() - Lift_Motor.rx_data->last_rx_tick > LIFT_LOST_TICK)
	{
		return LIFT_STATE_UNKNOWN;
	}

	switch(lift.state)
	{
	case LIFT_UP:
		return LIFT_STATE_UP;

	case LIFT_DOWN:
		return LIFT_STATE_DOWN;

	default:
		return LIFT_STATE_MOVING;
	}
}

static void Lift_Target_Update(Lift_t* lift)
{
	if(Board_Is_Online(&board) == 0)
	{
		return;
	}

	if(board.rx_data->shoot_target.lift != 0)
	{
		if(lift->state != LIFT_TO_DOWN && lift->state != LIFT_DOWN)
		{
			lift->state = LIFT_TO_DOWN;
			lift->move_tick = 0;
			lift->move_fault = 0;
		}
	}
	else
	{
		if(lift->state != LIFT_TO_UP && lift->state != LIFT_UP)
		{
			lift->state = LIFT_TO_UP;
			lift->move_tick = 0;
			lift->move_fault = 0;
		}
	}
}

static void Lift_Find_Record(Lift_t* lift, float angle)
{
	lift->angle_min = angle + LIFT_MIN_OFFSET_DOWN;
	lift->angle_max = lift->angle_min + LIFT_DEFAULT_TRAVEL;

	lift->is_find_limit = 1;
	lift->mode = LIFT_STOP;
	lift->state = LIFT_DOWN;
}

static void Lift_Block_Check(Lift_t* lift)
{
	if(lift->is_find_limit != 0)
	{
		return;
	}

	if(fabsf((float)Lift_Motor.rx_data->torque_current_raw) >= LIFT_FIND_CURRENT)
	{
		lift->block_tick ++;

		if(lift->block_tick >= LIFT_BLOCK_TICK)
		{
			lift->block_tick = 0;
			Lift_Find_Record(lift, Lift_Motor.rx_data->motor_angle_sum);
		}
	}
	else
	{
		lift->block_tick = 0;
	}
}

static void Lift_State_Machine(Lift_t* lift)
{
	if(lift->is_find_limit == 0)
	{
		lift->mode = LIFT_SPEED;
		lift->target_speed = -LIFT_FIND_SPEED;
		lift->find_tick ++;

		if(lift->find_tick > LIFT_FIND_TIMEOUT)
		{
			Lift_Find_Record(lift, Lift_Motor.rx_data->motor_angle_sum);
		}

		return;
	}

	if(lift->move_fault != 0)
	{
		lift->mode = LIFT_STOP;
		return;
	}

	switch(lift->state)
	{
	case LIFT_TO_DOWN:
		lift->mode = LIFT_ANGLE;
		lift->target_angle = lift->angle_min;
		lift->move_tick ++;

		if(fabsf(Lift_Motor.rx_data->motor_angle_sum - lift->angle_min) <= LIFT_ANGLE_TOL)
		{
			lift->state = LIFT_DOWN;
		}
		else if(lift->move_tick > LIFT_MOVE_TIMEOUT)
		{
			lift->move_fault = 1;
			lift->mode = LIFT_STOP;
		}
		break;

	case LIFT_TO_UP:
		lift->mode = LIFT_ANGLE;
		lift->target_angle = lift->angle_max;
		lift->move_tick ++;

		if(fabsf(Lift_Motor.rx_data->motor_angle_sum - lift->angle_max) <= LIFT_ANGLE_TOL)
		{
			lift->state = LIFT_UP;
		}
		else if(lift->move_tick > LIFT_MOVE_TIMEOUT)
		{
			lift->move_fault = 1;
			lift->mode = LIFT_STOP;
		}
		break;

	default:
		lift->mode = LIFT_STOP;
		break;
	}
}

static void Lift_Pid_Calculate(Lift_t* lift)
{
	switch(lift->mode)
	{
	case LIFT_SPEED:
		lift->out = all_pid_calc(NULL, &lift_speed_pid, lift->target_speed, 0.f,
		                         Lift_Motor.rx_data->speed, 1.f, 2);
		break;

	case LIFT_ANGLE:
		lift->out = all_pid_calc(&lift_angle_outer, &lift_angle_inner, lift->target_angle,
		                         Lift_Motor.rx_data->motor_angle_sum, Lift_Motor.rx_data->speed, -1.f, 0);
		break;

	default:
		lift->out = 0.f;
		break;
	}
}

static void Lift_Stuck_Check(Lift_t* lift)
{
	float angle = Lift_Motor.rx_data->motor_angle_sum;

	if(lift->mode != LIFT_ANGLE)
	{
		lift->stuck_tick = 0;
		lift->angle_last = angle;
		return;
	}

	if(fabsf(angle - lift->angle_last) >= LIFT_STUCK_ANGLE_TOL)
	{
		lift->angle_last = angle;
		lift->stuck_tick = 0;
		return;
	}

	lift->stuck_tick ++;

	if(lift->stuck_tick > LIFT_STUCK_TICK)
	{
		lift->stuck_tick = 0;
		lift->move_fault = 1;
		lift->mode = LIFT_STOP;
		lift->out = 0.f;
	}
}

static void Lift_Tx_Update(Lift_t* lift)
{
	board.tx_data->lift_state = Lift_Get_State();
}

static float Yaw_Front_Err(void)
{
	float yaw_mec;

	if(HAL_GetTick() - Yaw_Motor.rx_data->last_rx_tick > DM_MOTOR_LOST_TICK)
	{
		return 999.f;
	}

	yaw_mec = Yaw_Mec_Reduce(Yaw_Motor.rx_data->motor_angle);

	return fabsf(motor_half_cycle(GIMBAL_YAW_FRONT - yaw_mec, 2.f * PI));
}

static uint8_t lift_cmd_last = 0;
static uint8_t lift_cmd_seen = 0;

static void Lift_Cmd_Check(void)
{
	if(board.rx_data->shoot_target.lift != lift_cmd_last)
	{
		lift_cmd_last = board.rx_data->shoot_target.lift;
		lift_cmd_seen = 1;
	}
}

void Lift_Work(Lift_t* lift)
{
	if(Board_Is_Online(&board) == 0
		|| board.rx_data->car_data.car_state == CAR_SLEEP
		|| HAL_GetTick() - Lift_Motor.rx_data->last_rx_tick > LIFT_LOST_TICK)
	{
		lift->mode = LIFT_STOP;
		lift->out = 0.f;
		Lift_Motor.tx_data->torque = 0.f;
		return;
	}

	Lift_Cmd_Check();

	if(lift->is_find_limit == 0 && lift_cmd_seen == 0)
	{
		lift->mode = LIFT_STOP;
		lift->out = 0.f;
		Lift_Motor.tx_data->torque = 0.f;
		Lift_Tx_Update(lift);
		return;
	}

	Lift_Target_Update(lift);

	if(lift->is_find_limit == 0
		&& (fabsf(gimbal.pitch_mec_angle) > LIFT_FIND_PITCH_TOL
			|| Yaw_Front_Err() > LIFT_FIND_YAW_TOL))
	{
		lift->find_tick = 0;
		lift->mode = LIFT_STOP;
		lift->out = 0.f;
		Lift_Motor.tx_data->torque = 0.f;
		Lift_Tx_Update(lift);
		return;
	}

	Lift_Block_Check(lift);
	Lift_State_Machine(lift);
	Lift_Pid_Calculate(lift);
	Lift_Stuck_Check(lift);
	Lift_Tx_Update(lift);

	Lift_Motor.tx_data->torque = lift->out;
}
