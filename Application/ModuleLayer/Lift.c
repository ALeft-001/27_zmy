#include "Lift.h"
#include "board.h"
#include "gimbal.h"
#include "rc_sensor.h"
#include "rp_math.h"
#include "math.h"

Lift_t lift =
{
	.want = LIFT_EXTEND,
	.request = LIFT_EXTEND,
};

static uint32_t lift_up_tick = 0;

static void Lift_Switch_Update(Lift_t* lift);
static void Lift_Feedback_Update(Lift_t* lift);
static void Lift_Request_Update(Lift_t* lift);
static void Lift_Lock_Update(Lift_t* lift);
static void Lift_Tx_Update(Lift_t* lift);

static void Lift_Switch_Update(Lift_t* lift)
{
	if(rc_sensor_info.s1.status != up_R)
	{
		return;
	}

	if(HAL_GetTick() - lift_up_tick < LIFT_SWITCH_DEBOUNCE)
	{
		return;
	}

	lift_up_tick = HAL_GetTick();

	if(HAL_GetTick() - rc_sensor_info.last_rx_tick > LIFT_RC_LOST_TICK)
	{
		return;
	}

	lift->want = (lift->want == LIFT_RETRACT) ? LIFT_EXTEND : LIFT_RETRACT;
}

static void Lift_Feedback_Update(Lift_t* lift)
{
	if(board.rx_data->lift_state == LIFT_STATE_DOWN)
	{
		lift->retracted = 1;
	}
	else if(board.rx_data->lift_state == LIFT_STATE_UP)
	{
		lift->retracted = 0;
	}
}

static void Lift_Request_Update(Lift_t* lift)
{
	float yaw_err;

	if(rc_sensor_info.s1.value != RC_SW_UP)
	{
		if(board.rx_data->lift_state == LIFT_STATE_UP || board.rx_data->lift_state == LIFT_STATE_DOWN)
		{
			lift->request = (board.rx_data->lift_state == LIFT_STATE_DOWN) ? LIFT_RETRACT : LIFT_EXTEND;
		}

		return;
	}

	if(lift->want == LIFT_EXTEND)
	{
		lift->pitch_ready = 0;
		lift->yaw_ready = 0;
		lift->request = LIFT_EXTEND;
		return;
	}

	if(lift->pitch_ready == 0
		&& fabsf(board.rx_data->gimbal_data.pitch_mec - GIMBAL_PITCH_MIDDLE) <= LIFT_PITCH_TOL)
	{
		lift->pitch_ready = 1;
	}

	yaw_err = fabsf(motor_half_cycle(GIMBAL_YAW_FRONT - board.rx_data->gimbal_data.yaw_mec, 2.f * PI));

	if(lift->yaw_ready == 0 && yaw_err <= LIFT_YAW_TOL)
	{
		lift->yaw_ready = 1;
	}

	lift->request = (lift->pitch_ready != 0 && lift->yaw_ready != 0) ? LIFT_RETRACT : LIFT_EXTEND;
}

static void Lift_Lock_Update(Lift_t* lift)
{
	if(lift->want == LIFT_EXTEND && lift->retracted == 0)
	{
		lift->lock_tick = 0;
		lift->pitch_lock = 0;
		lift->yaw_lock = 0;
		return;
	}

	if(lift->retracted != 0)
	{
		lift->lock_tick = 0;
		lift->pitch_lock = 1;
		lift->yaw_lock = 1;
		return;
	}

	lift->pitch_lock = 1;

	if(lift->want == LIFT_RETRACT)
	{
		lift->yaw_lock = 1;
		lift->lock_tick ++;

		if(lift->lock_tick > LIFT_LOCK_TIMEOUT)
		{
			lift->lock_tick = 0;
			lift->want = LIFT_EXTEND;
			lift->pitch_lock = 0;
			lift->yaw_lock = 0;
		}
		return;
	}

	lift->yaw_lock = 0;
	lift->lock_tick = 0;
}

static void Lift_Tx_Update(Lift_t* lift)
{
	board_tx_data.shoot_data.lift = lift->request;
}

void Lift_Work(Lift_t* lift)
{
	Lift_Switch_Update(lift);
	Lift_Feedback_Update(lift);
	Lift_Request_Update(lift);
	Lift_Lock_Update(lift);
	Lift_Tx_Update(lift);
}
