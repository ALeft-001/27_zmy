#include "rm_motor.h"
#include "pid.h"
#include "arm_math.h"
#include "rp_math.h"

static float RPM_to_Rads(Motor_RM_Rx_Data_t *rx_data);
static void Torque_to_Raw_Current(Motor_RM_t *motor);

void RM_Motor_Update(Motor_RM_t *motor, uint8_t *rxBuf)
{
    Motor_RM_Rx_Data_t *motor_data = motor->rx_data;

    motor_data->encoder = (uint16_t)(rxBuf[0] << 8 | rxBuf[1]);
    motor_data->encoder_speed = (int16_t)(rxBuf[2] << 8 | rxBuf[3]);
    motor_data->speed = RPM_to_Rads(motor_data);
    motor_data->torque_current_raw = (int16_t)(rxBuf[4] << 8 | rxBuf[5]);
    motor_data->temperature = rxBuf[6];
    motor_data->last_rx_tick = HAL_GetTick();
}

void RM_Set_Torque(Motor_RM_Set_t *set)
{
    int16_t torque_current = 0;
    uint8_t Id;

    for(uint8_t i = 0; i < 4; i ++)
    {
        if(set->motor[i] != NULL)
        {
            Id = set->motor[i]->born_data->rxId * 2;
            Torque_to_Raw_Current(set->motor[i]);
            torque_current = set->motor[i]->tx_data->torque_current_raw;
            set->tx_buff[Id] = (uint8_t)(torque_current >> 8);
            set->tx_buff[Id+1] = (uint8_t)(torque_current);
        }
    }

    CAN_SendData(set->hcan, set->stdId, set->tx_buff);

    memset(set->tx_buff, 0, 8);
}

static void Torque_to_Raw_Current(Motor_RM_t *motor)
{
    motor->tx_data->torque_current = motor->tx_data->torque / TORQUE_CONSTANT_3508;
    motor->tx_data->torque_current = constrain(motor->tx_data->torque_current,-MAX_CURRENT_3508 * 0.9f, MAX_CURRENT_3508 * 0.9f);
    motor->tx_data->torque_current_raw = (int16_t)(motor->tx_data->torque_current /MAX_CURRENT_3508 * 16384.f);
}

static float RPM_to_Rads(Motor_RM_Rx_Data_t *rx_data)
{
    return rx_data->encoder_speed / 60.f * 2.f * PI / REDUCT_RATIO_3508;
}
