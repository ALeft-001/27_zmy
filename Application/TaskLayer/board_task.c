#include "board_task.h"
#include "board.h"

void StartBoardTask(void const * argument)
{
	for(;;)
	{
		Board_Tx_State_Data(&board);
		Board_Tx_Gimbal_Data(&board);

		osDelay(1);
	}
}
