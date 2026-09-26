
/* Includes ------------------------------------------------------------------*/
#include "motor.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


pid_ctrl_t yaw_gyro_outer = { .kp = 20.0f, .ki = 0.05f, .kd = 0.0f, .integral_max = 200.0f, .out_max = 500.0f };
pid_ctrl_t yaw_gyro_inner = { .kp = 0.04f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .out_max = 10.0f };
pid_ctrl_t R_Fric_Speed_Ctrl = { .kp = 10.f, .ki = 0.2f, .kd = 0.f, .integral_max = 6000.f, .out_max = 8000.f };

Motor_DM_Born_Data_t Yaw_Born_Data = { .tx_id = 0x002, .hcan = &hcan2 };
Motor_RM_Born_Data_t R_Fric_Born = { .rxId = 0, .hcan = &hcan1, .type = _6020_Single, .stdId = 0x1FE };
Motor_HT_Born_Data_t L_Wheel_Born_Data = { .stdId = 0x009, .hcan = &hcan1, .order_correction = 0 };

Motor_DM_Rx_Data_t Yaw_Rx_Data;
Motor_DM_Tx_Data_t Yaw_Tx_Data;
Motor_DM_State_Data_t Yaw_State_Data = { .motor_state = Motor_Unenable, .last_motor_state = Motor_Unenable };
Motor_RM_Rx_Data_t R_Fric_Rx;
Motor_RM_Tx_Data_t R_Fric_Tx;
Motor_RM_State_t R_Fric_State;
Motor_HT_Rx_Data_t L_Wheel_Rx_Data;
Motor_HT_Tx_Data_t L_Wheel_Tx_Data;
Motor_HT_State_t L_Wheel_State;

Motor_RM_Ctrl_Data_t R_Fric_Ctrl = { .speed_ctrl = &R_Fric_Speed_Ctrl };

Motor_DM_t Yaw_Motor =
{
	.born_data  = &Yaw_Born_Data,
	.rx_data    = &Yaw_Rx_Data,
	.tx_data    = &Yaw_Tx_Data,
	.state_data = &Yaw_State_Data,
};

Motor_RM_t R_Fric =
{
	.born_data = &R_Fric_Born,
	.rx_data = &R_Fric_Rx,
	.tx_data = &R_Fric_Tx,
	.state = &R_Fric_State,
	.ctrl = &R_Fric_Ctrl,
	.single_init = RM_Motor_Init,
};

Motor_HT_t L_Wheel =
{
	.born_data = &L_Wheel_Born_Data,
	.rx_data = &L_Wheel_Rx_Data,
	.tx_data = &L_Wheel_Tx_Data,
	.state = &L_Wheel_State,
	.single_init = &HT_Single_Motor_Init,
};

KT_motor_t kt_motor[] = {
	[0] = {
		.KT_motor_data = {
			.tx_data = {
				.angle_single_Control = 0,
				.angle_single_Control_maxSpeed = 0,
				.angle_single_Control_spinDirection = 0,
				.angle_add_Control = 0,
				.angle_add_Control_maxSpeed = 0,
				.angle_sum_Control = 0,
				.angle_sum_Control_maxSpeed = 0,
				.iqControl = 0,
				.speedControl = 0,
			},
			.id = {
				.tx_id = ID_GIMB_YAW,
				.rx_id = 0x88,
				.drive_type = M_CAN1,
				.motor_type = KT9015,
			},
		},
		.init = KT_motor_class_init,
	},
};

Motor_RM_Group_t RM_Group =
{
	.motor[0] = &R_Fric,
	.motor[1] = NULL,
	.motor[2] = NULL,
	.motor[3] = NULL,
	.stdId = 0x1FE,
	.hcan = &hcan1,
	.group_init = RM_Group_Motor_Init,
};

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{

	R_Fric.single_init(&R_Fric);
	RM_Group.group_init(&RM_Group);
}

void kt_motor_list_init()
{
	kt_motor[0].init(&kt_motor[0]);


}

void ht_motor_list_init()
{
	L_Wheel.single_init(&L_Wheel);

}

void rm_motor_list_heart_beat()
{
	RM_Group.group_heartbeat(&RM_Group);
}

