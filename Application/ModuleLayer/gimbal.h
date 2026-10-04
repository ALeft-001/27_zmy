#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stdint.h"
#include "motor.h"

#define   PITCH_MOTOR_ANGLE_MIDDLE    10.615f

#define   GIMBAL_PITCH_TORQUE_MAX     (T_MAX * 0.4f)

#define   PITCH_MEC_MAX_ANGLE         91.5f
#define   PITCH_MEC_MIN_ANGLE         (-55.5f)

#define   PITCH_MEC_DAMPING           0.85f
#define   PITCH_SPEED_DEADBAND        0.03f
#define   PITCH_SPEED_FILTER_ALPHA    0.3f

typedef struct{
	float yaw_target;
	float pitch_target;
}Gimbal_Target_t;

typedef struct{
	float yaw_out;
	float pitch_out;
}Gimbal_Out_t;

typedef struct Gimbal_Struct_t{
	Gimbal_Target_t    target;
	Gimbal_Out_t       out;
	uint8_t            work_flag;

	float              pitch_mec_angle;
}Gimbal_t;

extern Gimbal_t gimbal;

/* Exported functions --------------------------------------------------------*/
void Gimbal_Work(Gimbal_t* gimbal);

#endif
