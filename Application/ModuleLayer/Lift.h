#ifndef __LIFT_H__
#define __LIFT_H__

#include "stdint.h"

#define LIFT_MIN_OFFSET_DOWN      50.f
#define LIFT_ANGLE_TOL       10.f
#define LIFT_DEFAULT_TRAVEL  1500.f

#define LIFT_FIND_SPEED      1000.f
#define LIFT_FIND_CURRENT    1000.f
#define LIFT_BLOCK_TICK      60
#define LIFT_FIND_TIMEOUT    8000
#define LIFT_FIND_PITCH_TOL  0.05f
#define LIFT_FIND_YAW_TOL    0.15f
#define LIFT_MOVE_TIMEOUT    8000
#define LIFT_STUCK_ANGLE_TOL 0.01f
#define LIFT_STUCK_TICK      1500

#define LIFT_LOST_TICK       120

#define LIFT_STATE_UNKNOWN   0
#define LIFT_STATE_UP        1
#define LIFT_STATE_DOWN      2
#define LIFT_STATE_MOVING    3

typedef enum
{
	LIFT_DOWN = 0,
	LIFT_UP,
	LIFT_TO_DOWN,
	LIFT_TO_UP,
}Lift_State_e;

typedef enum
{
	LIFT_STOP = 0,
	LIFT_SPEED,
	LIFT_ANGLE,
}Lift_Mode_e;

typedef struct Lift_Struct_t
{
	Lift_State_e state;
	Lift_Mode_e  mode;

	float angle_min;
	float angle_max;
	float target_angle;
	float target_speed;
	float out;

	float angle_last;
	uint16_t block_tick;
	uint16_t find_tick;
	uint16_t move_tick;
	uint16_t stuck_tick;

	uint8_t  is_find_limit;
	uint8_t  move_fault;
}Lift_t;

extern Lift_t lift;

void Lift_Work(Lift_t* lift);

#endif
