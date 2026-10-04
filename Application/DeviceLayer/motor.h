#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "DM_motor.h"
#include "drv_can.h"


#define ID_GIMB_Y 		0x012
#define ID_GIMB_P 		0x011

extern  Motor_DM_t Yaw_Motor;
extern  Motor_DM_t Pitch_Motor;
extern  Motor_RM_t R_Fric;
extern  Motor_RM_Group_t RM_Group;

extern  pid_ctrl_t yaw_gyro_outer;
extern  pid_ctrl_t yaw_gyro_inner;
extern  pid_ctrl_t pitch_mec_outer;

#endif

