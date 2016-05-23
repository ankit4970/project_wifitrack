/*
 * gps.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ankit
 */

#include <stdio.h>
#include <stdlib.h>
#include "gps.hpp"
#include "lpc_sys.h"
#include "common.hpp"


#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define UARTRXQUEUESIZE		0x10
#define SystemCoreClock 48000000

char gBuffer[256];							/// Global buffer for received data
uint32_t bRecvInt=0;						/// Counter variable
SemaphoreHandle_t gUartSemaphore = NULL;	/// Uart Semaphore

/**************************************************************************************************
 * @brief		Initialize UART module for SIM808
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  					- LPC_UART0: UART0 peripheral
 * 						- LPC_UART1: UART1 peripheral
 * 						- LPC_UART2: UART2 peripheral
 * 						- LPC_UART3: UART3 peripheral
 * 				baudrate Baudrate for UART
 * @return 		true if successful
**************************************************************************************************/
bool sim808_gps::sim808_gpsInit(LPC_UART_TypeDef *UARTx, uint32_t baudrate)
{
	uint16_t baud = 0;
	const unsigned int pclk = sys_get_cpu_clock();

	gUartSemaphore = xSemaphoreCreateBinary();

	// Turn on UART2 module
	LPC_SC->PCONP |= (1 << 24);

	// Select clock for UART2 PCLK = CCLK
	LPC_SC->PCLKSEL1 &= ~(2<<16);
	LPC_SC->PCLKSEL1 |= (1<<16);


	// Configure UART2 :8-bit character length1 :1 stop bit :Parity Disabled
	UARTx->LCR = 0x03;

	// Enable DLAB (DLAB = 1)
	UARTx->LCR |= (1<<7);

	// For baudrate 115200
	//div = 48000000/16*baudRate;// = 312;
	//divWord = 48000000/(16*115200);
	baud = (pclk / (16 * baudrate));

	UARTx->DLL = (baud & 0xFF);
	UARTx->DLM = (baud >> 8);

	// Disabling DLAB (DLAB =0)
	UARTx->LCR &= ~(1<<7) ;

	// Enable & Reset FIFOs and set 4 char timeout for Rx
	UARTx->FCR = 0x07;
	UARTx->FCR |= (3<<6);
	//UARTx->FCR &= ~(1<<3);

	// Pin select for P2.8 TXD2 and P2.9 RXD2
	LPC_PINCON->PINSEL4 &= ~((1<<16) | (1<<18));
	LPC_PINCON->PINSEL4 |= ((1<<17) | (1<<19));

	// Pin select for P4.28 TXD3 and P4.29 RXD3
	//LPC_PINCON->PINSEL9 |= ((1<<24) | (1<<25) | (1<<26) | (1<<27));
	NVIC_EnableIRQ(UART2_IRQn);
	UARTx->IER = (1 << 0) | (1 << 2); // B0:Rx, B1: Tx
	sim808_reset(LPC_GPIO0, GPS_RESET_PIN);
	printf("Starting communication\n");

	mUARTx = UARTx;
	return true;

}

/**************************************************************************************************
 * @brief		Receive a single character byte from UART peripheral
 * @return 		Data byte
**************************************************************************************************/
uint8_t sim808_getch()
{
	return (LPC_UART2->RBR & UART_RBR_MASKBIT);
}


/****************************************************************************************
 * @brief		UART2 interrupt handler sub-routine
 * @param[in]	None
 * @return 		None
 ***************************************************************************************/
extern "C"
{
	void UART2_IRQHandler(void)
	{
		uint32_t intsrc, tmp, tmp1;
		char c = 0;
		uint32_t bToRecv=256, timeOut=0;
		uint8_t recvData = 0;
		/* Determine the interrupt source */
		intsrc = (LPC_UART2->IIR & 0x03CF);
		tmp = intsrc & UART_IIR_INTID_MASK;

		// Receive Line Status
		if (tmp == UART_IIR_INTID_RLS)
		{
			// Check line status
			tmp1 = ((LPC_UART2->LSR) & UART_LSR_BITMASK);
			// Mask out the Receive Ready and Transmit Holding empty status
			tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
					| UART_LSR_BI | UART_LSR_RXFE);
			// If any error exist
			if (tmp1)
			{
					while(1);
			}
		}

		// Receive Data Available or Character time-out
		if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI))
		{
			while (bToRecv)
			{
				if (!(LPC_UART2->LSR & UART_LSR_RDR))
				{
					printf("here1\n");
					gBuffer[bRecvInt++] = '\0';
					printf("-->%s\n",gBuffer);
					bRecvInt =0;
					xSemaphoreGiveFromISR(gUartSemaphore,NULL);
					break;
				}
				else
				{
					recvData= LPC_UART2->RBR;
					printf("%c\n",recvData);

					if ((recvData == 0xA) | (recvData == 0xD))
					{
						continue;
					}
					//delay_ms(5);
					printf("%c\n",recvData);
					gBuffer[bRecvInt] = recvData;
					bRecvInt++;
					bToRecv--;
				}
			}

		}

		// Transmit Holding Empty
		if (tmp == UART_IIR_INTID_THRE)
		{
				//UART_IntTransmit();
		}
		//bRecvInt ++;
		//gBuffer[bRecv] = '\0';

	}
}



/**************************************************************************************************
 * @brief		Reset SIM808 module
 * @param[in]	GPIOx     : pointer to GPIO base
 *  			pinNumber : GPIO pin connected for reset
 * @return		None
**************************************************************************************************/
void sim808_gps::sim808_reset(LPC_GPIO_TypeDef *GPIOx,uint8_t pinNumber)
{
	/* Set GPIO0.0 pin as output , GPS Reset pin is connected to this pin*/
	GPIOx->FIODIR |= (1 << pinNumber);

	/* PIN high */
	GPIOx->FIOPIN |= (1 << pinNumber);

	delay_ms(1000);

	/* pin low*/
	GPIOx->FIOPIN &= ~(1 << pinNumber);

	delay_ms(1000);

	/* PIN high */
	GPIOx->FIOPIN |= (1 << pinNumber);

}

/**************************************************************************************************
 * @brief		Receive a single character byte from UART peripheral
 * @return 		Data received
**************************************************************************************************/
uint8_t sim808_gps::sim808_getch()
{
	return (mUARTx->RBR & UART_RBR_MASKBIT);
}

/**************************************************************************************************
 * @brief		Send a single character byte
 * @param[in]	data	UART data to be sent
 * @return 		None
**************************************************************************************************/
void sim808_gps::sim808_putch(uint8_t data)
{
	mUARTx->THR = (data & UART_THR_MASKBIT);
	while(! (mUARTx->LSR & (1 << 6)));
}

/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf :	Pointer to Transmit buffer
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
 * @param[in]	send	pointer to data tobe sent
 * @return 		Number of bytes received
 **************************************************************************************************/
uint8_t sim808_gps::sim808_getReply(int8_t *send, bool multiline )
{
	uint8_t readLength =0;

	printf("sim808_getReply : Command is %s\n",send);


	sim808_send(send);

	delay_ms(4000);


	if(multiline == true)
	{
		//readLength = gps_uartReceive(replybuffer,MAX_BUFFER_SIZE,NONE_BLOCKING, multiline);
		//printf("%s\n",replybuffer);
	}


	return readLength;
}

/**************************************************************************************************
 * @brief		Send data on UART peripheral and wait for reply
 * @param[in]	send	:	pointer to data to be sent:
 * @return 		Data received
 **************************************************************************************************/
bool sim808_gps::sim808_sendCheckReply(int8_t *send, int8_t *reply)
{
	printf("sim808_sendCheckReply : Command is %s\n",send);

	if(sim808_getReply(send) <= 0) // added for testing
	{
		//printf("Error in get reply\n");
		//return false;
	}


	return (strcmp((const char*)replybuffer,(const char*) reply) == 0 | strcmp((const char*)replybuffer,(const char*)"OK") == 0);

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
bool sim808_gps::enableGPS(bool onoff)
{

	//sim808_send((int8_t*)"ATE0\r\n");

	//delay_ms(2000);

	if (true == onoff)
	{
		sim808_send((int8_t*)"AT+CGNSPWR=1\r\n");
	}

	else if (false == onoff)
	{
		sim808_send((int8_t*)"AT+CGNSPWR=0\r\n");
	}

	delay_ms(2000);

	sim808_send((int8_t*)"AT+CGNSSEQ=\"RMC\"\r\n");

	delay_ms(2000);

	return true;
}

/**************************************************************************************************
 * @brief		change baudrate of SIM808
 * @param[in]	baudRate	: baudrate for the SIM808 module
 * @return 		true if successful
**************************************************************************************************/
bool sim808_gps::changeBaudRate(uint32_t baudRate)
{
	delay_ms(2000);
	if(!sim808_sendCheckReply((int8_t*)"AT+IPR=115200\r\n",(int8_t* )"OK"))
	{
		printf("AT+IPR error\n");
		return false;
	}
	return true;
}


/**************************************************************************************************
 * @brief		Get GPS data from the module
 * @param[in]	sensor	: pointer to sensor structure
 * @return 		true if successful
**************************************************************************************************/
bool sim808_gps::getGPS(senseordata *sensor)
{
	int8_t gpsbuffer[256];
	uint8_t res_len = 0;
	uint32_t yr =0;
	uint8_t temp1 =0,temp2=0,temp3=0,temp4=0;

	// Send info command
	sim808_send((int8_t*)"AT+CGNSINF\r\n");

	delay_ms(10000);

	if (xSemaphoreTake(gUartSemaphore, portMAX_DELAY))
	{
		// GPS run status
		char *tok = strtok((char *)gBuffer, ",");
		if (! tok)
			return false;

		// fix status
		tok = strtok(NULL, ",");
		if (! tok)
			return false;

		// date
		char *timeGps = strtok(NULL, ",");
		if (! tok)
			return false;

		temp1 = (timeGps[0] -'0');
		temp2 = (timeGps[1] -'0');
		temp3 = (timeGps[2] -'0');
		temp4 = (timeGps[3] -'0');
		sensor->s_year = temp1*1000+temp2*100+temp3*10+temp4;

		temp1 = timeGps[4]-'0';
		temp2 = timeGps[5]-'0';
		sensor->s_month=(temp1)*10+(temp2);

		temp1 = timeGps[6]-'0';
		temp2 = timeGps[7]-'0';
		sensor->s_day=temp1*10+temp2;

		temp1 = timeGps[8]-'0';
		temp2 = timeGps[9]-'0';
		sensor->s_hour=temp1*10+temp2;

		temp1 = timeGps[10]-'0';
		temp2 = timeGps[11]-'0';
		sensor->s_min=temp1*10+temp2;

		temp1 = timeGps[12]-'0';
		temp2 = timeGps[13]-'0';
		sensor->s_sec=temp1*10+temp2;

		// latitude
		char *latp = strtok(NULL, ",");
		if (! latp)
			return false;

		// longitude
		char *longp = strtok(NULL, ",");
		if (! longp)
			return false;

		sensor->latitude = atof(latp);
		sensor->longitude = atof(longp);

		bzero((void *)gBuffer,sizeof(gBuffer));
	}

	return true;
}

