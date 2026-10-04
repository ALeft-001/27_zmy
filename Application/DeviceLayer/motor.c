
/* Includes ------------------------------------------------------------------*/
#include "motor.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


pid_ctrl_t yaw_gyro_outer = { .kp = 20.0f, .ki = 0.05f, .kd = 0.0f, .integral_max = 200.0f, .out_max = 500.0f };
pid_ctrl_t yaw_gyro_inner = { .kp = 0.04f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .out_max = 10.0f };
pid_ctrl_t pitch_mec_outer = { .kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 500.0f, .out_max = 10.0f };
pid_ctrl_t R_Fric_Speed_Ctrl = { .kp = 10.f, .ki = 0.2f, .kd = 0.f, .integral_max = 6000.f, .out_max = 8000.f };

Motor_DM_Born_Data_t Yaw_Born_Data = { .tx_id = 0x002, .hcan = &hcan2 };
Motor_DM_Born_Data_t Pitch_Born_Data = { .tx_id = 0x001, .hcan = &hcan1 };
Motor_RM_Born_Data_t R_Fric_Born = { .rxId = 0, .hcan = &hcan1, .type = _6020_Single, .stdId = 0x1FE };

Motor_DM_Rx_Data_t Yaw_Rx_Data;
Motor_DM_Tx_Data_t Yaw_Tx_Data;
Motor_DM_State_Data_t Yaw_State_Data;
Motor_DM_Rx_Data_t Pitch_Rx_Data;
Motor_DM_Tx_Data_t Pitch_Tx_Data;
Motor_DM_State_Data_t Pitch_State_Data;
Motor_RM_Rx_Data_t R_Fric_Rx;
Motor_RM_Tx_Data_t R_Fric_Tx;
Motor_RM_State_t R_Fric_State;

Motor_RM_Ctrl_Data_t R_Fric_Ctrl = { .speed_ctrl = &R_Fric_Speed_Ctrl };

Motor_DM_t Yaw_Motor =
{
	.born_data  = &Yaw_Born_Data,
	.rx_data    = &Yaw_Rx_Data,
	.tx_data    = &Yaw_Tx_Data,
	.state_data = &Yaw_State_Data,
};

Motor_DM_t Pitch_Motor =
{
	.born_data  = &Pitch_Born_Data,
	.rx_data    = &Pitch_Rx_Data,
	.tx_data    = &Pitch_Tx_Data,
	.state_data = &Pitch_State_Data,
};

Motor_RM_t R_Fric =
{
	.born_data = &R_Fric_Born,
	.rx_data = &R_Fric_Rx,
	.tx_data = &R_Fric_Tx,
	.state = &R_Fric_State,
	.ctrl = &R_Fric_Ctrl,
};

Motor_RM_Group_t RM_Group =
{
	.motor[0] = &R_Fric,
	.motor[1] = NULL,
	.motor[2] = NULL,
	.motor[3] = NULL,
	.stdId = 0x1FE,
	.hcan = &hcan1,
};

