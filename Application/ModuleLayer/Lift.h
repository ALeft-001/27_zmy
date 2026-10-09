#ifndef __LIFT_H__
#define __LIFT_H__

#include "stdint.h"

#define LIFT_EXTEND          0
#define LIFT_RETRACT         1

#define LIFT_STATE_UNKNOWN   0
#define LIFT_STATE_UP        1
#define LIFT_STATE_DOWN      2
#define LIFT_STATE_MOVING    3

#define LIFT_PITCH_TOL       0.15f
#define LIFT_YAW_TOL         0.15f
#define LIFT_RC_LOST_TICK    60
#define LIFT_LOCK_TIMEOUT    15000
#define LIFT_SWITCH_DEBOUNCE 300

typedef struct Lift_Struct_t
{
	uint8_t want;
	uint8_t request;
	uint8_t retracted;
	uint8_t pitch_ready;
	uint8_t yaw_ready;
	uint8_t pitch_lock;
	uint8_t yaw_lock;
	uint16_t lock_tick;
}Lift_t;

extern Lift_t lift;

void Lift_Work(Lift_t* lift);

#endif
