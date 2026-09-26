#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "arm_math.h"
#include "HT_Motor.h"
#include "drv_can.h"
#include "drv_tick.h"
#include "rp_math.h"
/* Exported typedef ----------------------------------------------------------*/
#ifndef __HT_MOTOR_H
typedef enum Motor_MIT_Command_enum_e
{
	Enter_Motor_Mode,
	Exit_Motor_Mode,
	Zero_Position_Sensor,

}Motor_MIT_Command_e;
#endif

#define P_MIN -12.5f    // Radians
#define P_MAX 12.5f
#define V_MIN -30.0f    // Rad/s
#define V_MAX 30.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -10.0f    // N.m
#define T_MAX 10.0f
#define C_MIN -10.0f    // A
#define C_MAX 10.0f

typedef enum Motor_DM_Work_state_enum_e
{
	Motor_Enable,
	Motor_Unenable,
	Over_Voltage,
	Lack_Voltage,
	Over_Current,
	MOS_OverTemp,
	Motor_OverTemp,
	Commun_Loss,
	Unknow_Err,
}Motor_DM_Work_state_e;

typedef struct Motor_DM_Born_Data_struct_t
{
	uint32_t tx_id;

#ifdef __STM32F4xx_HAL_H
	CAN_HandleTypeDef *hcan;
#endif

#ifdef STM32H7xx_HAL_H
	FDCAN_HandleTypeDef *hcan;
#endif

	int8_t order_correction;
}Motor_DM_Born_Data_t;

typedef struct Motor_DM_Rx_Data_struct_t
{
	float speed;

	float torque;

	float motor_angle_sum;

	float motor_angle;

	float motor_angle_last;
}Motor_DM_Rx_Data_t;

typedef struct Motor_DM_Tx_Data_struct_t
{
	float torque;

	float target_speed;

	float target_angle;

	float Kp;

	float Kd;

	uint8_t tx_buff[8];
}Motor_DM_Tx_Data_t;

typedef struct Motor_DM_State_Data_struct_t
{
	dev_work_state_t status;

	Motor_DM_Work_state_e motor_state;

	Motor_DM_Work_state_e last_motor_state;
}Motor_DM_State_Data_t;

typedef struct Motor_DM_struct_t
{
	Motor_DM_Born_Data_t* born_data;

	Motor_DM_Rx_Data_t* rx_data;

	Motor_DM_Tx_Data_t* tx_data;

	Motor_DM_State_Data_t* state_data;
}Motor_DM_t;

/* Exported functions --------------------------------------------------------*/
void DM_Motor_Update(Motor_DM_t *motor, uint8_t *rxBuf);
void DM_Motor_Sleep(Motor_DM_t *motor);
void DM_Motor_ZeroPos(Motor_DM_t *motor);
void DM_Set_Torque(Motor_DM_t *motor);
void DM_Set_Speed(Motor_DM_t *motor);
void DM_Set_Angle(Motor_DM_t *motor);

#endif
