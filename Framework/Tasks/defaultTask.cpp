/*
 *  Created on: 23.02.2020
 *      Author: harald
 */

#include "defaultTask.h"
#include <usb_device.h>
#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>
#include <System/usb_printf.h>


void StartDefaultTask(void *argument)
{
	UNUSED(argument);
	MX_USB_DEVICE_Init();

	//displaynmenus::DisplayNMenus::instance().init();


	for(;;)
	{

		tx_printf("This is defaultTask\n");
		HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);

		//displaynmenus::DisplayNMenus::instance().cycle();
		osDelay(1000);

	}

}

