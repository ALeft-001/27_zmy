/**
  ******************************************************************************
  * @file    control_task.c
  * @brief
  ******************************************************************************
  */
#include "control_task.h"
#include "chassis.h"
#include "gimbal.h"

void StartCtrlTask(void const * argument)
{

	for(;;)
	{
		Chassis_Work(&chassis);
		gimbal.work(&gimbal);

		osDelay(1);
	}
}
