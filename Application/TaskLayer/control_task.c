#include "control_task.h"
#include "gimbal.h"

float pitch;
float roll;
float yaw;

void StartControlTask(void const * argument)
{
	for(;;)
	{
			imu_sensor.update(&imu_sensor);
			pitch = imu_sensor.info->base_info.pitch;
			roll = imu_sensor.info->base_info.roll;
			yaw = imu_sensor.info->base_info.yaw;

			Gimbal_Work(&gimbal);

		osDelay(1);
	}
}
