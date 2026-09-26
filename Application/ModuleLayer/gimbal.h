#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stdint.h"

typedef struct{
	float yaw_target;
}Gimbal_Target_t;

typedef struct{
	float yaw_out;
}Gimbal_Out_t;

typedef struct Gimbal_Struct_t{
	Gimbal_Target_t    target;
	Gimbal_Out_t       out;
	uint8_t            work_flag;
}Gimbal_t;

extern Gimbal_t gimbal;

/* Exported functions --------------------------------------------------------*/
void Gimbal_Work(Gimbal_t* gimbal);

#endif
