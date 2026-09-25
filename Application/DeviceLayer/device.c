/**
 * @file  device.c
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "chassis.h"
#include "gimbal.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	imu_sensor.init(&imu_sensor);
	rc_sensor.init(&rc_sensor);

	board.init(&board);

	gimbal.init(&gimbal);
}

