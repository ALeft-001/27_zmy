#ifndef __BOARD_PROTOCOL_H
#define __BOARD_PROTOCOL_H

#include "stdint.h"
#include "rp_device_config.h"

#define  BOARD_OFFLINE_CNT_MAX    50

#define  CAR_SLEEP    0
#define  CAR_RC       1
#define  CAR_KEY      2

#define  ID_PKT_01     0xD1
#define  ID_PKT_02     0xD2
#define  ID_PKT_03     0xD3
#define  ID_PKT_04     0xD4
#define  ID_PKT_05     0xD5
#define  ID_MEG_01     0xC1
#define  ID_MEG_02     0xC2
#define  ID_MEG_03     0xC3
#define  ID_MEG_04     0xC4
#define  ID_MEG_05     0xC5

typedef struct{
  uint8_t  car_state;       //0��ж����1��ң�أ�2�Ǽ���
//	uint8_t  gimbal_state;    //0��ж����1������
//	uint8_t  launch_state;    //0�ǹط��������1�ǿ��������
  uint8_t  gimbal_mode;        //0�ǻ�е��1������
	uint8_t  vision_mode;     //0���Ӿ�ģʽ��1����ͨ���飬2��С����3�Ǵ����4��ǰ�ڣ�5��Ӣ��

	uint8_t  game_start;
	uint8_t  my_color;
	
	float    v_x;
	float    v_y;
	
}Board_Car_Pkt_t;


typedef struct{
  float  shoot_speed;
	float  shoot_freq;
	int16_t  shoot_heat_err;
	uint16_t  allowance_max;
	
}Board_Judge_Shoot_Pkt_t;


typedef struct{
	float yaw_mec_tar;
	float yaw_imu_tar;
	float pitch_mec_tar;
	float pitch_imu_tar;
	uint8_t  is_hole;
}Board_Gimbal_Target_Pkt_t;

typedef struct{
	uint8_t  launch_state;    //0�ǹط��������1�ǿ��������
  uint8_t  shoot_mode;
	uint8_t  shoot_level;


}Board_Shoot_Pkt_t;


typedef struct{
	uint8_t blood[8];    
  //Ӣ��
  //����
  //�ڱ�
  //����
  //���˻�
  //�״�
	//����
	//ǰ��
}Board_Blood_Pkt_t;


typedef struct{
  Board_Car_Pkt_t                  car_pkt;
  Board_Judge_Shoot_Pkt_t          judge_shoot_pkt; 
  Board_Gimbal_Target_Pkt_t        gimbal_target_pkt;
  Board_Shoot_Pkt_t                shoot_pkt;
  Board_Blood_Pkt_t                blood_pkt;

}Board_Tx_Pkt_t;



typedef struct{
	float yaw_mec;
	float yaw_imu;
	float pitch_mec;
	float pitch_imu;
	
	

}Board_Gimbal_Meg_t;

typedef struct{
	float vision_yaw_tar;
	float vision_pitch_tar;
	uint8_t  is_find_target;
	
}Board_Vision_Meg_t;


typedef struct{
  uint8_t  yaw_motor_state;
	uint8_t  pitch_motor_state;
	uint8_t  height_motor_state;
	uint8_t  r_fric_state;
	uint8_t  l_fric_state;
	uint8_t  dial_motor_state;
	uint8_t  vision_state;
  	uint8_t  is_down;
}Board_State_Meg_t;


typedef struct{
	Board_Gimbal_Meg_t    gimbal_meg;
	Board_Vision_Meg_t    vision_meg;
	Board_State_Meg_t     state_meg;
	
}Board_Rx_Meg_t;


typedef  struct{
	uint16_t offline_cnt_max;
	dev_work_state_t status;
	uint16_t offline_cnt;
	uint32_t last_rx_tick;

}Board_Status_t;

typedef struct Board_Struct_t{
	Board_Status_t*    status;
	Board_Tx_Pkt_t*    tx_pkt;
  Board_Rx_Meg_t*    rx_meg;
	
	
	void (*tx_01)(struct Board_Struct_t* board);
	void (*tx_02)(struct Board_Struct_t* board);
	void (*tx_03)(struct Board_Struct_t* board);
	void (*tx_04)(struct Board_Struct_t* board);
	
	void (*rx_01)(struct Board_Struct_t* board, uint8_t *rxBuf);
	void (*rx_02)(struct Board_Struct_t* board, uint8_t *rxBuf);

	void (*init)(struct Board_Struct_t *board);

}Board_t;



extern  Board_t  board;

void Board_Init(Board_t* board);


void Board_Tx_Pkt_01(Board_t* board);

void Board_Tx_Pkt_02(Board_t* board);

void Board_Tx_Pkt_03(Board_t* board);

void Board_Tx_Pkt_04(Board_t* board);

void Board_Rx_Meg_01(Board_t* board,uint8_t* rxbuf);

void Board_Rx_Meg_02(Board_t* board,uint8_t* rxbuf);


#endif


