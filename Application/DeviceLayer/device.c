/**
 * @file  device.c
 */
 
/* Includes ------------------------------------------------------------------*/
#include "device.h"

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
	RM_Motor_Init(&R_Fric);
	RM_Group_Motor_Init(&RM_Group);
}

