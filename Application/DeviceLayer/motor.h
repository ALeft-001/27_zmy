#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "KT_motor.h"
#include "HT_motor.h"
#include "DM_motor.h"
#include "drv_can.h"


#define ID_GIMB_YAW 	0x142
#define ID_GIMB_Y 		0x012

extern  KT_motor_t kt_motor[1];
extern  Motor_HT_t L_Wheel;
extern  Motor_DM_t Yaw_Motor;
extern  Motor_RM_t R_Fric;
extern  Motor_RM_Group_t RM_Group;

extern  pid_ctrl_t yaw_gyro_outer;
extern  pid_ctrl_t yaw_gyro_inner;
/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
void kt_motor_list_init(void);
void ht_motor_list_init(void);

#endif

