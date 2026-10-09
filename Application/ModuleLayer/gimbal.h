#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stdint.h"
#include "imu_sensor.h"

#define   GIMBAL_YAW_STEP        0.18f
#define   GIMBAL_PITCH_STEP_RAD  (0.05f * PI / 180.f)
#define   GIMBAL_PITCH_MIN -0.9687f
#define   GIMBAL_PITCH_MAX 1.5970f
#define   GIMBAL_YAW_FRONT       -0.3709933f
#define   GIMBAL_RC_LOST_TICK    60
#define   GIMBAL_PITCH_MIDDLE    0.f
#define   GIMBAL_CENTER_TIMEOUT  2000

typedef enum{
	G_SLEEP,
	G_INIT,
	G_GYRO,

}Gimbal_Mode_e;

typedef struct{
	float yaw_imu;

}Gimbal_Data_t;

typedef struct{
	float yaw_imu_tar;
	float yaw_mec_tar;
	float pitch_mec_tar;

	uint8_t yaw_mode;

}Gimbal_Target_t;

typedef struct Gimbal_Struct_t{
	Gimbal_Mode_e    mode;
	Gimbal_Data_t    data;
	Gimbal_Target_t  target;

	uint8_t          init_flag;
	uint8_t          center_done;
	uint16_t         center_tick;

}Gimbal_t;

extern Gimbal_t gimbal;

void Gimbal_Work(Gimbal_t* gimbal);

#endif
