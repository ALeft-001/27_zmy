#include "control_task.h"
#include "chassis.h"
#include "gimbal.h"
#include "Shoot.h"

void StartCtrlTask(void const * argument)
{

	for(;;)
	{
		Chassis_Work(&chassis);
		Gimbal_Work(&gimbal);
		Shoot_Work(&shoot);
		osDelay(1);
	}
}
