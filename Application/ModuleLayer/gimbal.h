#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stdint.h"

#define   GIMBAL_YAW_STEP        0.18f

#define   BOARD_LOST_TICK        50
#define   GIMBAL_RC_LOST_TICK    60

typedef enum{
	G_SLEEP,
	G_INIT,
	G_GYRO,

}Gimbal_Mode_e;

typedef struct{
	float yaw_imu;

}Gimbal_Info_t;

typedef struct{
	float yaw_imu_tar;

}Gimbal_Target_t;

typedef struct Gimbal_Struct_t{
	Gimbal_Mode_e    mode;
	Gimbal_Info_t    info;
	Gimbal_Target_t  target;

	uint8_t          init_flag;

	void (*init)(struct Gimbal_Struct_t* gimbal);
	void (*work)(struct Gimbal_Struct_t* gimbal);

}Gimbal_t;

extern Gimbal_t gimbal;

#endif
