/*
 * gps.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ankit
 */

#include <stdio.h>
#include <stdlib.h>
#include "gps.hpp"

#define UART_BLOCKING_TIMEOUT 0xff
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#if 1

//LPC_UART2
/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
**************************************************************************************************/
bool sim808_gps::sim808_gpsInit(LPC_UART_TypeDef *UARTx)
{
	// Turn on UART2 module
	LPC_SC->PCONP |= (1 << 24);

	// Select clock for UART2 PCLK = CCLK
	LPC_SC->PCLKSEL1 &= ~(2<<16);
	LPC_SC->PCLKSEL1 |= (1<<16);

	// Configure UART3 :8-bit character length1 :1 stop bit :Parity Disabled
	UARTx->LCR = 0x03;

	// Enable DLAB (DLAB = 1)
	UARTx->LCR |= (1<<7);

	// For baudrate 115200
	//div = 48000000/16*115200;// = 312;
	// DLM+DLL = div

	UARTx->DLL = 0x1A;
	UARTx->DLM = 0;

	// Disabling DLAB (DLAB =0)
	UARTx->LCR &= ~(1<<7) ;

	// Pin select for P2.8 TXD2 and P2.9 RXD2
	LPC_PINCON->PINSEL4 &= ~((0xF<<16));
	LPC_PINCON->PINSEL4 |= ((1<<17)|(1<<19));

	mUARTx = UARTx;

	return true;
}



/**************************************************************************************************
 * @brief		Receive a single character byte from UART peripheral
 * @return 		Data received
**************************************************************************************************/
int8_t sim808_gps::sim808_getch()
{
	return mUARTx->RBR;
}

/**************************************************************************************************
 * @brief		Send a single character byte
 * @param[in]	data	UART data to be sent
 * @return 		None
**************************************************************************************************/
void sim808_gps::sim808_putch(uint8_t data)
{
	while(!(mUARTx->LSR & (1<<5)));
		mUARTx->THR = data;
}

/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
 * Note: when using UART in BLOCKING mode, a time-out condition is used
 * 		 via defined symbol UART_BLOCKING_TIMEOUT.
**************************************************************************************************/
uint32_t sim808_gps::sim808_send(int8_t *txbuf)
{
	uint32_t bRemain, bSent, timeOut;
	int8_t *pChar = txbuf;
	uint32_t buflen = 0;

	buflen = strlen((const char *)txbuf);
	bRemain = buflen;

	bSent = 0;
	while (bRemain)
	{
		timeOut = UART_BLOCKING_TIMEOUT;
		// Wait for THR empty with timeout
		while (!(mUARTx->LSR & UART_LSR_THRE))
		{
			if (timeOut == 0)
				break;
			timeOut--;
		}
		// Time out!
		if(timeOut == 0)
			break;

		while (bRemain)
		{
			sim808_putch((*pChar++));
			bRemain--;
			bSent++;
		}
	}
	return bSent;
}


/**************************************************************************************************
 * @brief		Flush input stream of UART peripheral
 * @return 		None
**************************************************************************************************/
void sim808_gps::sim808_flushin()
{
    // Read all available serial input to flush pending data.
    uint16_t timeoutloop = 0;

    while (timeoutloop++ < 40)
    {
        while((mUARTx->LSR & UART_LSR_RDR))
        {
        	sim808_getch();
            timeoutloop = 0;  // If char was received reset the timer
        }
        delay_ms(1);
    }
}

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	timeout	UART peripheral selected, should be:
 *  			timeout: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
**************************************************************************************************/
uint16_t sim808_gps::sim808_readline(uint16_t timeout, bool multiline )
{
	uint16_t replyidx = 0;

	while (timeout--)
	{
		if (replyidx >= 254)
		{
			//DEBUG_PRINTLN(F("SPACE"));
			break;
		}

		while((mUARTx->LSR & UART_LSR_RDR))
		{
			char c =  sim808_getch();
			if (c == '\r')
				continue;
			if (c == 0xA)
			{
				if (replyidx == 0)   // the first 0x0A is ignored
					continue;

				if (!multiline)
				{
					timeout = 0;         // the second 0x0A is the end of the line
					break;
				}
			}
			replybuffer[replyidx] = c;
			//DEBUG_PRINT(c, HEX); DEBUG_PRINT("#"); DEBUG_PRINTLN(c);
			replyidx++;
		}

		if (timeout == 0)
		{
			printf("TIMEOUT\n");
			break;
		}

		delay_ms(1);
	}

	replybuffer[replyidx] = 0;  // null term

	return replyidx;
}

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
**************************************************************************************************/
bool sim808_gps::sim808_parseReply(int8_t *toreply, uint16_t *v, int8_t divider, uint8_t index)
{
	char *p = strstr((const char *)replybuffer, (const char *)toreply);

	if (p == 0)
		return false;

	p+=strlen((const char*)toreply);
	printf("%s\n",p);

	for (uint8_t i=0; i<index;i++)
	{
		// increment dividers
		p = strchr(p, divider);
		if (!p)
			return false;
		p++;
		printf("%s\n",p);
	}
	*v = atoi(p);

	return true;
}

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
 * @return 		Data received
 **************************************************************************************************/
uint8_t sim808_gps::sim808_getReply(int8_t *send )
{
	uint8_t readLength =0;
	uint16_t timeout = UART_BLOCKING_TIMEOUT;

	sim808_flushin();

	sim808_send(send);

	readLength = sim808_readline(timeout);

	printf("%s\n",replybuffer);

	return readLength;
}

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
 **************************************************************************************************/
bool sim808_gps::sim808_sendCheckReply(int8_t *send, int8_t *reply)
{
	if (! sim808_getReply(send) )
		return false;

	/*
	for (uint8_t i=0; i<strlen(replybuffer); i++) {
	DEBUG_PRINT(replybuffer[i], HEX); DEBUG_PRINT(" ");
	}
	DEBUG_PRINTLN();
	for (uint8_t i=0; i<strlen(reply); i++) {
	DEBUG_PRINT(reply[i], HEX); DEBUG_PRINT(" ");
	}
	DEBUG_PRINTLN();
	*/

	return (strcmp((const char*)replybuffer,(const char*) reply) == 0);
}

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	tosend	:
 * @param[in]	toreply	:
 * @param[in]	v		:
 * @param[in]	divider :
 * @param[in] 	index	:
 * @return 		True	: Successful Return
 * 				False   : Error Occured
**************************************************************************************************/
bool sim808_gps::sendParseReply(int8_t * tosend,int8_t* toreply,uint16_t *v, int8_t divider, uint8_t index)
{
	sim808_getReply(tosend);

	if (! sim808_parseReply(toreply, v, divider, index))
		return false;

	sim808_readline(); // eat 'OK'

	return true;
}

// use AT+CGNS as this is version 2
/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
**************************************************************************************************/
bool sim808_gps::enableGPS(bool onoff)
{
	uint16_t state;

	if (!sendParseReply((int8_t *)"AT+CGNSPWR?",(int8_t *) "+CGNSPWR: ", &state) )
		return false;


	if (onoff && !state)
	{
		if (!sim808_sendCheckReply((int8_t *)"AT+CGNSPWR=1",(int8_t *) "OK"))
		{
			return false;
		}
	}

	else if (!onoff && state)
	{
		if (!sim808_sendCheckReply((int8_t *)"AT+CGNSPWR=0",(int8_t *) "OK"))
		{
			return false;
		}
	}

	return true;
}

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
**************************************************************************************************/
uint8_t sim808_gps::getGPS(uint8_t arg, int8_t *buffer, uint8_t maxbuff)
{
	sim808_getReply((int8_t *)"AT+CGNSINF");

	char *p = strstr((const char *)replybuffer, "SINF");
	if (p == 0)
	{
		buffer[0] = 0;
		return 0;
	}

	p+=6;

	uint8_t len = MAX(maxbuff-1, strlen(p));
	strncpy((char *)buffer,(const char *) p, len);
	buffer[len] = 0;

	sim808_readline(); // eat 'OK'
	return len;
}


/*
int8_t sim808_gps::GPSstatus(void)
{
	// 808 V2 uses GNS commands and doesn't have an explicit 2D/3D fix status.
	// Instead just look for a fix and if found assume it's a 3D fix.
	getReply(F("AT+CGNSINF"));
	char *p = prog_char_strstr(replybuffer, (prog_char*)F("+CGNSINF: "));
	if (p == 0) return -1;
	p+=10;
	readline(); // eat 'OK'
	if (p[0] == '0') return 0; // GPS is not even on!

	p+=2; // Skip to second value, fix status.
	//DEBUG_PRINTLN(p);
	// Assume if the fix status is '1' then we have a 3D fix, otherwise no fix.
	if (p[0] == '1') return 3;
	else return 1;

	return 0;
}
*/

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
**************************************************************************************************/
bool sim808_gps::getGPS(float *lat, float *lon, float *speed_kph, float *heading, float *altitude)
{
	int8_t gpsbuffer[120];
	uint8_t res_len = 0;

	// grab the mode 2^5 gps csv from the sim808
	res_len = getGPS(32, gpsbuffer, 120);

	// make sure we have a response
	if (res_len == 0)
		return false;

	// skip GPS run status
	char *tok = strtok((char *)gpsbuffer, ",");
	if (! tok)
		return false;

	// skip fix status
	tok = strtok(NULL, ",");
	if (! tok)
		return false;

	// skip date
	tok = strtok(NULL, ",");
	if (! tok)
		return false;

	// grab the latitude
	char *latp = strtok(NULL, ",");
	if (! latp)
		return false;

	// grab longitude
	char *longp = strtok(NULL, ",");
	if (! longp)
		return false;

	*lat = atof(latp);
	*lon = atof(longp);

	// only grab altitude if needed
	if (altitude != NULL)
	{
		// grab altitude
		char *altp = strtok(NULL, ",");
		if (! altp) return false;

		*altitude = atof(altp);
	}

	// only grab speed if needed
	if (speed_kph != NULL)
	{
		// grab the speed in km/h
		char *speedp = strtok(NULL, ",");
		if (! speedp) return false;

		*speed_kph = atof(speedp);
	}

	// only grab heading if needed
	if (heading != NULL)
	{

		// grab the speed in knots
		char *coursep = strtok(NULL, ",");
		if (! coursep)
			return false;

		*heading = atof(coursep);
	}

	return true;
}
#endif
