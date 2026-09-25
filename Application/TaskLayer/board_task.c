#include "board_task.h"
#include "board.h"

void StartBoardTask(void const * argument)
{
	for(;;)
	{
		Board_Tx_Car_Data(&board);
		Board_Tx_Gimbal_Target(&board);

		osDelay(1);
	}
}
