/*
 * uart_messaging.c
 *
 *  Created on: 16.12.2018
 *      Author: harald
 */

#include <usbd_cdc_if.h>
#include <stdio.h>
#include <stdarg.h>
#include <main.h>
#include <System/usb_printf.h>
#include "cmsis_os.h"

#define TX_BUFF_LEN 128
uint8_t txBuff[TX_BUFF_LEN];
uint16_t tx_act_pos = 0;


#ifndef min
#define min(a,b)   (((a) < (b)) ? (a) : (b))
#endif

/*
 to enable float formatted output, see:
 https://stackoverflow.com/questions/28334435/stm32-printf-float-variable.
 - add -u _printf_float to your LDFLAGS.
 or
 - printf("Result is: %d.%d", i/10, i%10);
 */

int tx_printf(const char *format, ...) {
	tx_buff_clear();

	va_list arg;
	va_start (arg, format);
	tx_act_pos = vsprintf ((char*) &txBuff[0], format, arg);
	va_end (arg);

	uint8_t result = CDC_Transmit_FS(txBuff, tx_act_pos);
	// @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY

	if (result == USBD_OK)
		return SUCCESS;
	else
		return ERROR;
}

void tx_buff_clear(void) {
	for(uint16_t i=0; i < TX_BUFF_LEN; i++) {
		txBuff[i] = '\0';
	}
	tx_act_pos = 0;
}

void usb_receive(uint8_t* buf, uint32_t len) {

	uint8_t buffer[20];
	memset(buffer, '\0', 20);
	memcpy(buffer, buf, (size_t) len);

	tx_printf("received %i bytes: %s \n", len, buffer);

}







