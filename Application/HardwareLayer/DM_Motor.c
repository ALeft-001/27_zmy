/* Includes ------------------------------------------------------------------*/
#include "DM_Motor.h"

static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf);
static void Motor_SetControlPara(Motor_DM_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command);
static void Angle_Sum_Cal(Motor_DM_t *motor);
static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word);

void DM_Motor_Sleep(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		motor->tx_data->torque = 0;
		motor->tx_data->Kd = 0;
		motor->tx_data->Kp = 0;
	}
}

void DM_Motor_ZeroPos(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		DM_Motor_Sleep(motor);

		Motor_Send_Command(motor, Zero_Position_Sensor);
	}
}

void DM_Set_Torque(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		if(motor->state_data->motor_state == Motor_Unenable)
		{
			motor->state_data->motor_state = Motor_Enable;
			Motor_Send_Command(motor, Enter_Motor_Mode);
		}
		else
		{
			Motor_DM_Tx_Data_t* motor_tx_data = motor->tx_data;
			motor_tx_data->Kp = 0;
			motor_tx_data->Kd = 0;
			Motor_SetControlPara(motor);
			motor->tx_data->torque = 0;
		}
	}
}

void DM_Set_Speed(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		if(motor->state_data->motor_state == Motor_Unenable)
		{
			motor->state_data->motor_state = Motor_Enable;
			Motor_Send_Command(motor, Enter_Motor_Mode);
		}
		else
		{
			Motor_DM_Tx_Data_t* motor_tx_data = motor->tx_data;
			motor_tx_data->Kp = 0;
			Motor_SetControlPara(motor);
		}
	}
}

void DM_Set_Angle(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		if(motor->state_data->motor_state == Motor_Unenable)
		{
			motor->state_data->motor_state = Motor_Enable;
			Motor_Send_Command(motor, Enter_Motor_Mode);
		}
		else
		{
			motor->tx_data->target_speed = 0;
			Motor_SetControlPara(motor);
		}
	}
}

void DM_Motor_Update(Motor_DM_t *motor, uint8_t *rxBuf)
{
	Motor_DM_Rx_Data_t* motor_rx_data = motor->rx_data;

	Motor_ERR_Check(motor, rxBuf[0] >> 4);
	motor_rx_data->motor_angle = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), P_MIN, P_MAX, 16);
	motor_rx_data->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), V_MIN, V_MAX, 12);

	if(abs(motor_rx_data->speed) == 0.0109901428f)
	{
		motor_rx_data->speed = 0;
	}

	motor_rx_data->torque = uint_to_float((uint16_t)(((rxBuf[4]&0x0F) << 8) | rxBuf[5]), C_MIN, C_MAX, 12);

	Angle_Sum_Cal(motor);
}

static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command)
{
	switch(Command)
	{
		case Enter_Motor_Mode:
		Motor_Command[7] = 0xFC;
		break;
		case Exit_Motor_Mode:
		Motor_Command[7] = 0xFD;
		break;
		case Zero_Position_Sensor:
		Motor_Command[7] = 0xFE;
		break;
		default:
		break;
	}

	Motor_Send_Data(motor, Motor_Command);
}

static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf)
{
	Motor_DM_Born_Data_t* motor_born_data = motor->born_data;

	CAN_SendData(motor_born_data->hcan, motor_born_data->tx_id, buf);
}

static void Motor_SetControlPara(Motor_DM_t *motor)
{
	Motor_DM_Tx_Data_t* motor_tx_data = motor->tx_data;
	uint16_t p, v, kp, kd, t;
	uint8_t* buf = motor_tx_data->tx_buff;

	motor_tx_data->target_angle = constrain(motor_tx_data->target_angle, P_MIN, P_MAX);
	motor_tx_data->target_speed = constrain(motor_tx_data->target_speed, V_MIN, V_MAX);
	motor_tx_data->Kp = constrain(motor_tx_data->Kp, KP_MIN, KP_MAX);
	motor_tx_data->Kd = constrain(motor_tx_data->Kd, KD_MIN, KD_MAX);
	motor_tx_data->torque = constrain(motor_tx_data->torque, T_MIN, T_MAX);

	p = float_to_uint(motor_tx_data->target_angle,      P_MIN,  P_MAX,  16);
	v = float_to_uint(motor_tx_data->target_speed,      V_MIN,  V_MAX,  12);
	kp = float_to_uint(motor_tx_data->Kp,    KP_MIN, KP_MAX, 12);
	kd = float_to_uint(motor_tx_data->Kd,    KD_MIN, KD_MAX, 12);
	t = float_to_uint(motor_tx_data->torque,      T_MIN,  T_MAX,  12);

	buf[0] = p>>8;
	buf[1] = p&0xFF;
	buf[2] = v>>4;
	buf[3] = ((v&0xF)<<4)|(kp>>8);
	buf[4] = kp&0xFF;
	buf[5] = kd>>4;
	buf[6] = ((kd&0xF)<<4)|(t>>8);
	buf[7] = t&0xff;

	Motor_Send_Data(motor, buf);
}

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;

	return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;

	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

static void Angle_Sum_Cal(Motor_DM_t *motor)
{
	float err = 0.f;

	float order_correction = 0.f;

	if(motor->born_data->order_correction == 1 || motor->born_data->order_correction == -1)
	{
		order_correction = (float)motor->born_data->order_correction;
	}
	else
	{
		order_correction = 1.f;
	}

	if(!motor->rx_data->motor_angle_last && !motor->rx_data->motor_angle_sum)
	{
		err = 0.f;
	}
	else
	{
		err = motor->rx_data->motor_angle - motor->rx_data->motor_angle_last;
	}

	if(abs(err) > (float)PI)
	{
		if(err > 0.f)
		{
			motor->rx_data->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;
		}
		else
		{
			motor->rx_data->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;
		}
	}
	else
	{
		motor->rx_data->motor_angle_sum += err * order_correction;
	}

	motor->rx_data->motor_angle_last = motor->rx_data->motor_angle;
}

static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word)
{
	Motor_DM_State_Data_t* my_state = motor->state_data;

	switch(err_word)
	{
		case 0:
		my_state->motor_state = Motor_Unenable;
		break;
		case 1:
		my_state->motor_state = Motor_Enable;
		break;
		case 8:
		my_state->motor_state = Over_Voltage;
		break;
		case 9:
		my_state->motor_state = Lack_Voltage;
		break;
		case 10:
		my_state->motor_state = Over_Current;
		break;
		case 11:
		my_state->motor_state = MOS_OverTemp;
		break;
		case 12:
		my_state->motor_state = Motor_OverTemp;
		break;
		case 13:
		my_state->motor_state = Commun_Loss;
		break;
		default:
		my_state->motor_state = Unknow_Err;
		break;
	};

	if(my_state->motor_state != Motor_Enable &&
		my_state->motor_state != Motor_Unenable)
	{
		my_state->last_motor_state = my_state->motor_state;
	}
}
