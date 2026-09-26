#ifndef __HT_MOTOR_H
#define __HT_MOTOR_H

#include "rp_config.h"
#include "drv_can.h"
#include "drv_tick.h"
#include "rp_math.h"
#include "arm_math.h"

#define HT_P_MIN -95.5f    // Radians
#define HT_P_MAX 95.5f        
#define HT_V_MIN -45.0f    // Rad/s
#define HT_V_MAX 45.0f
#define HT_KP_MIN 0.0f     // N-m/rad
#define HT_KP_MAX 500.0f
#define HT_KD_MIN 0.0f     // N-m/rad/s
#define HT_KD_MAX 5.0f
#define HT_T_MIN -18.0f    // N.m
#define HT_T_MAX 18.0f
#define HT_C_MIN -40.0f    // A
#define HT_C_MAX 40.0f
#define HT_TORQUE_CONSTANT 0.45f //ת�س���N.m/A
#define TIME_STEP 0.001
/*���ָ�*/
typedef enum Motor_MIT_Command_enum_e
{
	Enter_Motor_Mode,//ʹ�ܵ������(ָʾ�Ʊ���)
	Exit_Motor_Mode,//ʧ�ܵ������(ָʾ�Ʊ��)
	Zero_Position_Sensor,//�趨��ǰ����Ƕ�Ϊ��
	
}Motor_MIT_Command_e;

/*���ģʽ*/
typedef enum Motor_HT_Work_mode_enum_e
{
	Motor_Control,//����ɿ�
	Motor_UnControl,//������ɿ�
}Motor_HT_Work_mode_e;

/*�����ʼ������*/
typedef struct Motor_HT_Born_Data_struct_t
{	
    uint32_t stdId;//������Ʊ���ID

#ifdef __STM32F4xx_HAL_H
    CAN_HandleTypeDef *hcan;//can��ѡ��
#endif
	
#ifdef STM32H7xx_HAL_H
    FDCAN_HandleTypeDef *hcan;//can��ѡ��
#endif
	  int8_t order_correction;//������涨

}Motor_HT_Born_Data_t;

/*���յ��������Ϣ�ṹ��*/
typedef struct Motor_HT_Rx_Data_struct_t
{
	float encoder;//�ϵ��ĽǶ��ۼ�(��λrad),-95,5~95.5
	
	float encoder_last;
	
	float encoder_err;
	
	float speed;//����ٶ�(��λrad/s)
	
	float torque;//���ת��(��λN.m)
	
	float torque_current;//�������(��λA)
	
	float motor_angle_sum;
	
	float motor_angle_sum_vi;
	
	float motor_angle_sum_filter;

  float motor_angle;//��������ƾ��ԽǶȣ�0~2PI
	
	float motor_angle_last;
	
	/*����ʱ�����*/
	uint32_t time_now;
	
	uint32_t time_last;
	
	float time;
}Motor_HT_Rx_Data_t;

/*���͵��������Ϣ�ṹ��*/
typedef struct Motor_HT_Tx_Data_struct_t
{
	float torque;//��Ҫ���͵�ת��(��λN.m)
	
	float target_speed;//Ŀ���ٶ�(��λrad/s)
	
	float target_angle;//Ŀ��Ƕ�(��λrad)
	
	float Kp;//λ������
	
	float Kd;//�ٶ�����
	
	uint8_t single_tx_buff[8];//ʹ�õ������ʱ�ĸ�������
}Motor_HT_Tx_Data_t;
/* �����ο����� = (torque + Kp*err_angle + Kd*err_speed) */

/*���״̬�ṹ��*/
typedef struct Motor_HT_State_struct_t
{
    uint32_t offline_cnt;

    uint32_t offline_cnt_max;

    dev_work_state_t status;

		Motor_HT_Work_mode_e mode;
}Motor_HT_State_t;

/*������ܽṹ��*/
typedef struct Motor_HT_struct_t
{
	Motor_HT_Born_Data_t* born_data;
	
	Motor_HT_Rx_Data_t* rx_data;
	
	Motor_HT_Tx_Data_t* tx_data;
	
	Motor_HT_State_t* state;

	
	void (*single_init)(struct Motor_HT_struct_t *motor);
	
	void (*single_sleep)(struct Motor_HT_struct_t *motor);
	
	void (*zero_position)(struct Motor_HT_struct_t *motor);
	
	void (*single_set_torque)(struct Motor_HT_struct_t *motor);
	
	void (*single_set_speed)(struct Motor_HT_struct_t *motor);
	
	void (*single_set_angle)(struct Motor_HT_struct_t *motor);
	
	void (*rx)(struct Motor_HT_struct_t *motor, uint8_t *rxBuf);
	
	void (*single_heart_beat)(struct Motor_HT_struct_t *motor);
}Motor_HT_t;

/*�����ṹ�壬���ں�ֻ̩�е�������ƣ��ýṹ��ֻ�ǰѵ������һЩͨ�õĹ��������������������*/
typedef struct Motor_HT_Group_struct_t
{
		Motor_HT_t* motor[4];
	
	  void (*group_set_torque)(struct Motor_HT_Group_struct_t *group);
	
		void (*group_sleep)(struct Motor_HT_Group_struct_t *group);
	
	  void (*group_init)(struct Motor_HT_Group_struct_t *group);
	
	  void (*group_heartbeat)(struct Motor_HT_Group_struct_t *group);
	
}Motor_HT_Group_t;

void HT_Single_Motor_Init(Motor_HT_t *motor);
void HT_Group_Motor_Init(Motor_HT_Group_t *group);
extern uint8_t flag_rx;

#endif
