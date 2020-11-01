/*
 *  Created on: 23.02.2020
 *      Author: harald
 */

#include "task02.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>
#include <System/usb_printf.h>



void StartTask02(void *argument)
{
	UNUSED(argument);
	//displaynmenus::DisplayNMenus::instance().init();


	for(;;)
	{

		tx_printf("This is Task02\n");
		HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);


		//displaynmenus::DisplayNMenus::instance().cycle();
		osDelay(1000);


	}

}

