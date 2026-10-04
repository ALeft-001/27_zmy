#ifndef __BOARD_PROTOCOL_H
#define __BOARD_PROTOCOL_H

#include "stdint.h"

#define  BOARD_LOST_TICK    50

#define  CAR_SLEEP     0
#define  CAR_RC        1
#define  CAR_KEY       2

#define  ID_CAR_DATA        0xD1
#define  ID_GIMBAL_TARGET   0xD2
#define  ID_GIMBAL_DATA     0xC2

typedef struct{
	uint8_t  car_state;

}Board_Car_Data_t;

typedef struct{
	float    yaw_imu_tar;
	float    pitch_mec_tar;

}Board_Gimbal_Target_Data_t;

typedef struct{
	float    yaw_mec;
	float    yaw_imu;
	float    pitch_mec;
	float    pitch_imu;

}Board_Gimbal_Data_t;

typedef struct{
	Board_Gimbal_Data_t          gimbal_data;

}Board_Tx_Data_t;

typedef struct{
	Board_Car_Data_t             car_data;
	Board_Gimbal_Target_Data_t   gimbal_target_data;

}Board_Rx_Data_t;

typedef struct Board_Struct_t{
	uint32_t                last_rx_tick;
	Board_Rx_Data_t*        rx_data;
	Board_Tx_Data_t*        tx_data;

}Board_t;

uint8_t Board_Is_Online(Board_t* board);

void Board_Tx_Gimbal_Data(Board_t* board);

void Board_Rx_Car_Data(Board_t* board, uint8_t* rxBuf);
void Board_Rx_Gimbal_Target(Board_t* board, uint8_t* rxBuf);

#endif
