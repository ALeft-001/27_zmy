#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stdint.h"
#include "imu_sensor.h"

#define   GIMBAL_YAW_STEP        0.18f
#define   GIMBAL_PITCH_STEP_RAD  (0.05f * PI / 180.f)
#define   GIMBAL_PITCH_MIN -0.9687f
#define   GIMBAL_PITCH_MAX 1.5970f
#define   GIMBAL_YAW_FRONT       -1.46009731
#define   GIMBAL_RC_LOST_TICK    60
#define   GIMBAL_PITCH_MIDDLE    0.f

typedef enum{
	G_SLEEP,
	G_INIT,
	G_GYRO,

}Gimbal_Mode_e;

typedef struct{
	float yaw_imu;

}Gimbal_Data_t;

typedef struct{
	float yaw_current;
	float yaw_last;
}Chassis_yaw_t;

typedef struct{
	float yaw_imu_tar;
	float pitch_mec_tar;

}Gimbal_Target_t;

typedef struct Gimbal_Struct_t{
	Gimbal_Mode_e    mode;
	Gimbal_Data_t    data;
	Gimbal_Target_t  target;

	uint8_t          init_flag;

}Gimbal_t;

extern Gimbal_t gimbal;
extern Chassis_yaw_t chassis_yaw;

void Gimbal_Work(Gimbal_t* gimbal);

#endif
