#include "can_protocol.h"
#include "board.h"


void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x205:
		{
			R_Fric.rx(&R_Fric, rxBuf);
			break;
		}
		case ID_GIMB_P:
		{
			DM_Motor_Update(&Pitch_Motor, rxBuf);
			break;
		}

		default:
			break;
	}
}

void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{

	switch (canId)
	{
		case ID_CAR_DATA:
		{
			Board_Rx_Car_Data(&board, rxBuf);
			break;
		}
		case ID_GIMBAL_TARGET:
		{
			Board_Rx_Gimbal_Target(&board, rxBuf);
			break;
		}
		case ID_GIMB_Y:
		{
			DM_Motor_Update(&Yaw_Motor, rxBuf);
			break;
		}

		default:
			break;
	}
}
