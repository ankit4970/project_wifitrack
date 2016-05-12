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

#define UART_BLOCKING_TIMEOUT 0xffffffff
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define UARTRXQUEUESIZE		0x10
#if 1
#define SystemCoreClock 48000000
//LPC_UART2
char gBuffer[256];
uint16_t gI;
/*********************************************************************//**
 * @brief		Determines best dividers to get a target clock rate
 * @param[in]	UARTx	Pointer to selected UART peripheral, should be:
 * 				- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @param[in]	baudrate Desired UART baud rate.
 * @return 		Error status, could be:
 * 				- SUCCESS
 * 				- ERROR
 **********************************************************************/
bool sim808_gps::set_divisors(uint32_t baudrate)
{
	int errorStatus = -1;

	uint64_t uClk=48000000;
	uint32_t d, m, bestd, bestm, tmp;
	uint64_t best_divisor, divisor;
	uint32_t current_error, best_error;
	uint32_t recalcbaud;
	//LPC_UART_TypeDef *UARTx,

	uClk = 48000000;

	/* In the Uart IP block, baud rate is calculated using FDR and DLL-DLM registers
	* The formula is :
	* BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)
	* It involves floating point calculations. That's the reason the formulae are adjusted with
	* Multiply and divide method.*/
	/* The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
	* 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15 */
	//best_error = 0xFFFFFFFF; 	/* Worst case */
	bestd = 0;
	bestm = 0;
	best_divisor = 0;
	for (m = 1 ; m <= 15 ;m++)
	{
		for (d = 0 ; d < m ; d++)
		{
		  divisor = ((uint64_t)uClk<<28)*m/(baudrate*(m+d));
		  current_error = divisor & 0xFFFFFFFF;

		  tmp = divisor>>32;

		  /* Adjust error */
		  if(current_error > ((uint32_t)1<<31)){
			current_error = -current_error;
			tmp++;
			}

		  if(tmp<1 || tmp>65536) /* Out of range */
		  continue;

		  if( current_error < best_error){
			best_error = current_error;
			best_divisor = tmp;
			bestd = d;
			bestm = m;
			if(best_error == 0) break;
			}
		} /* end of inner for loop */

		if (best_error == 0)
		  break;
	} /* end of outer for loop  */

	if(best_divisor == 0)
		return -1; /* can not find best match */

	recalcbaud = (uClk>>4) * bestm/(best_divisor * (bestm + bestd));

	/* reuse best_error to evaluate baud error*/
	if(baudrate>recalcbaud) best_error = baudrate - recalcbaud;
	else best_error = recalcbaud -baudrate;

	best_error = best_error * 100 / baudrate;

	if (best_error < UART_ACCEPTED_BAUDRATE_ERROR)
	{

	LPC_UART3->LCR |= UART_LCR_DLAB_EN;
	LPC_UART3->/*DLIER.*/DLM = UART_LOAD_DLM(best_divisor);
	LPC_UART3->/*RBTHDLR.*/DLL = UART_LOAD_DLL(best_divisor);
			/* Then reset DLAB bit */
	LPC_UART3->LCR &= (~UART_LCR_DLAB_EN) & UART_LCR_BITMASK;
	LPC_UART3->FDR = (UART_FDR_MULVAL(bestm) \
					| UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;

		errorStatus = 0;
	}

	return errorStatus;
}
uint32_t UART_GetIntId()
{
	return (LPC_UART3->IIR & 0x03CF);
}
uint8_t UART_GetLineStatus()
{
	return ((LPC_UART3->LSR) & UART_LSR_BITMASK);
}
void UART_IntErr(uint8_t bLSErrType)
{
	while(1);
}

uint8_t sim808_gps:: UART_ReceiveByte()
{
	return (LPC_UART3->/*RBTHDLR.*/RBR & UART_RBR_MASKBIT);

}
uint8_t sim808_gps::UART_Receive( uint8_t *rxbuf, uint32_t buflen,TRANSFER_BLOCK_Type flag,bool multiline)
{
	uint32_t bytesToRecv = 0, bRecvd =0, timeOut = 0;
	uint8_t *pChar = rxbuf;
	uint8_t recvData = 0;
	bytesToRecv = buflen;

	// Blocking mode
	if (BLOCKING == flag)
	{
		bRecvd = 0;
		while (bytesToRecv)
		{
			timeOut = UART_BLOCKING_TIMEOUT;
			while (!(LPC_UART3->LSR & UART_LSR_RDR))
			{
				if (timeOut == 0)
					break;
				timeOut--;
			}

			// Time out!
			if(timeOut == 0)
				break;

			// Get data from the buffer
			recvData = sim808_getch();
			if ((recvData == 0xA) | (recvData == 0xD))
			{
				if (bRecvd == 0)   // the first 0x0A is ignored
					continue;

				if (!multiline)
				{
					//lTimeOut = 0;         // the second 0x0A is the end of the line
					break;
				}
			}
			(*pChar++) = recvData;
			bytesToRecv--;
			bRecvd++;
		}
	}
	// None blocking mode
	else
	{

		bRecvd = 0;
		while (bytesToRecv)
		{
			if (!(LPC_UART3->LSR & UART_LSR_RDR))
			{
				break;
			}
			else
			{
				recvData = sim808_getch();
				if ((recvData == 0xA) | (recvData == 0xD))
				{
					if (bRecvd == 0)   // the first 0x0A is ignored
						continue;

					if (!multiline)
					{
						//lTimeOut = 0;         // the second 0x0A is the end of the line
						break;
					}
				}
				(*pChar++) = recvData;
				bRecvd++;
				bytesToRecv--;
			}
		}
	}

	(*pChar++) = '\0';
	return bRecvd;
}
/*********************************************************************//**
 * @brief		UART0 interrupt handler sub-routine
 * @param[in]	None
 * @return 		None
 **********************************************************************/
extern "C"
{
	void UART3_IRQHandler(void)
	{
		uint32_t intsrc, tmp, tmp1;
		char c = 0;
		/* Determine the interrupt source */
		intsrc = UART_GetIntId();
		tmp = intsrc & UART_IIR_INTID_MASK;

		// Receive Line Status
		if (tmp == UART_IIR_INTID_RLS){
			// Check line status
			tmp1 = UART_GetLineStatus();
			// Mask out the Receive Ready and Transmit Holding empty status
			tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
					| UART_LSR_BI | UART_LSR_RXFE);
			// If any error exist
			if (tmp1) {
					UART_IntErr(tmp1);
			}
		}

		// Receive Data Available or Character time-out
		if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
				//UART_IntReceive();
				c = LPC_UART3->RBR;
			    printf("%d-->%c\n",gI++,c);
		}

		// Transmit Holding Empty
		if (tmp == UART_IIR_INTID_THRE){
				//UART_IntTransmit();
		}

	}
}
#if 0
extern "C"
{
    void UART3_IRQHandler()
    {
    		const uint16_t transmitterEmpty = (1 << 1);
    		const uint16_t dataAvailable    = (2 << 1);
    		const uint16_t dataTimeout      = (6 << 1);
    		const uint16_t rls      		= (3 << 1);
    		long higherPriorityTaskWoken = 0;
    		long switchRequired = 0;
    		char c = 0;
    		unsigned charsSent = 0;

    		uint16_t reasonForInterrupt = (LPC_UART3->IIR & 0xE);
    		{
    			/**
    			 * If multiple sources of interrupt arise, let this interrupt exit, and re-enter
    			 * for the new source of interrupt.
    			 */
    			//printf("rsfi == %x\n",reasonForInterrupt);
    			switch (reasonForInterrupt)
    			{

    				case dataAvailable:
    				case dataTimeout:
    				{
    					//mLastActivityTime = xTaskGetTickCountFromISR();
    					/**
    					 * While receive Hardware FIFO not empty, keep queuing the data.
    					 * Even if xQueueSendFromISR() Fails (Queue is full), we still need to
    					 * read RBR register otherwise interrupt will not clear
    					 */
    					while ((LPC_UART3->LSR & (1 << 0)))
    					{
    						c = LPC_UART3->RBR;
    						printf("%d-->%c\n",gI++,c);

    					}

    				}
    				break;

    				case rls:
    					//printf("**>rls\n");
    					break;

    				default:
    					/* Read LSR register to clear Line Status Interrupt */
    					reasonForInterrupt = LPC_UART3->LSR;
    					c = LPC_UART3->RBR;
    					printf("%d<--> %c\n",gI,c);
    					//printf("%d-->%c\n",gI++,c);
    					break;
    			}
    		}

    		//gBuffer[gI] = 0;
    		//printf("%d--> %c\n",gI,c);
    		gI = 0;
    }
}
#endif
#if 0
// UART3 Interrupt handler
bool sim808_gps::getChar(char* pInputChar, unsigned int timeout)
{
    if (!pInputChar || !gpsRxQueue) {
        return false;
    }
    else if (taskSCHEDULER_RUNNING == xTaskGetSchedulerState()) {
        return xQueueReceive(gpsRxQueue, pInputChar, timeout);
    }
    else {
        unsigned int timeout_of_char = sys_get_uptime_ms() + timeout;
        while (! xQueueReceive(gpsRxQueue, pInputChar, 0)) {
            if (sys_get_uptime_ms() > timeout_of_char) {
                return false;
            }
        }
    }
    return true;
}
#endif
#if 0
void sim808_gps::handleInterruptGps(void)
{
	uint8_t IIRValue, LSRValue;
	static uint8_t i =0;
/*
	i++;
	while ((LPC_UART3->LSR & (1 << 0)))
	{
		data = LPC_UART3->RBR;
		printf("<%d-- %c\n",i,data);

	}
*/
	const uint16_t transmitterEmpty = (1 << 1);
	const uint16_t dataAvailable    = (2 << 1);
	const uint16_t dataTimeout      = (6 << 1);

	long higherPriorityTaskWoken = 0;
	long switchRequired = 0;
	char c = 0;
	unsigned charsSent = 0;

	uint16_t reasonForInterrupt = (LPC_UART3->IIR & 0xE);
	{
		/**
		 * If multiple sources of interrupt arise, let this interrupt exit, and re-enter
		 * for the new source of interrupt.
		 */
		switch (reasonForInterrupt)
		{

			case dataAvailable:
			case dataTimeout:
			{
				//mLastActivityTime = xTaskGetTickCountFromISR();
				/**
				 * While receive Hardware FIFO not empty, keep queuing the data.
				 * Even if xQueueSendFromISR() Fails (Queue is full), we still need to
				 * read RBR register otherwise interrupt will not clear
				 */
				while (0 != (LPC_UART3->LSR & (1 << 0)))
				{
					c = LPC_UART3->RBR;
					xQueueSendFromISR(gpsRxQueue, &c, &higherPriorityTaskWoken);
					if(higherPriorityTaskWoken) {
						switchRequired = 1;
					}
				}

			}
			break;

			default:
				/* Read LSR register to clear Line Status Interrupt */
				reasonForInterrupt = LPC_UART3->LSR;
				break;
		}
	}


}
#endif

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
 *  			- LPC_UART0: UART0 peripheral
 * 				- LPC_UART1: UART1 peripheral
 * 				- LPC_UART2: UART2 peripheral
 * 				- LPC_UART3: UART3 peripheral
 * @return 		Data received
**************************************************************************************************/
bool sim808_gps::sim808_gpsInit(LPC_UART_TypeDef *UARTx, uint32_t baudrate)
{
	uint32_t Fdiv;
	uint32_t pclkdiv;
	const unsigned int pclk = sys_get_cpu_clock();
	uint16_t baud = 0;
#if 0
	// Pin select for P2.8 TXD2 and P2.9 RXD2
	LPC_PINCON->PINSEL4 &= ~((0xF<<16));
	LPC_PINCON->PINSEL4 |= ((1<<17)|(1<<19));

	// Turn on UART2 module
	LPC_SC->PCONP |= (1 << 24);


	pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
	switch ( pclkdiv )
	{
		case 0x00:
		default:
			pclk = SystemCoreClock/4;
			break;
		case 0x01:
			pclk = SystemCoreClock;
			break;
		case 0x02:
			pclk = SystemCoreClock/2;
			break;
		case 0x03:
			pclk = SystemCoreClock/8;
			break;
	}

	LPC_UART2->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
	Fdiv = ( pclk / 16 ) / baudrate ;	/*baud rate */
	LPC_UART2->DLM = Fdiv / 256;
	LPC_UART2->DLL = Fdiv % 256;
	LPC_UART2->LCR = 0x03;		/* DLAB = 0 */
	LPC_UART2->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

	NVIC_EnableIRQ(UART2_IRQn);

	LPC_UART2->IER = IER_RBR | IER_RLS;	/* Enable UART3 interrupt */


//
//	// Configure UART3 :8-bit character length1 :1 stop bit :Parity Disabled
//	UARTx->LCR = 0x03;
//
//	// Enable DLAB (DLAB = 1)
//	UARTx->LCR |= (1<<7);
//
//	// For baudrate 115200
//	//div = 48000000/16*115200;// = 312;
//	// DLM+DLL = div
//
//	UARTx->DLL = 0x38;
//	UARTx->DLM = 0x01;
//
//	// Disabling DLAB (DLAB =0)
//	UARTx->LCR &= ~(1<<7) ;



#endif
#if 0
	// Pin select for P4.28 TXD3 and P4.29 RXD3
	LPC_PINCON->PINSEL9 |= ((1<<24) | (1<<25) | (1<<26) | (1<<27));
	// Turn on UART3 module
	LPC_SC->PCONP |= (1 << 25);

	// Select clock for UART3 PCLK = CCLK
	LPC_SC->PCLKSEL1 &= ~(2<<18);
	LPC_SC->PCLKSEL1 |= (1<<18);

	// Configure UART3 :8-bit character length1 :1 stop bit :Parity Disabled
	LPC_UART3->LCR = 0x03;


	baud = (pclk / (16 * baudrate)) + 0.5;

	// Set baud rate
	LPC_UART3->LCR |= (1<<7);				// Enable DLAB (DLAB = 1)
	//LPC_UART3->DLM = (baud >> 8);
	//LPC_UART3->DLL = (baud & 0xFF);

	LPC_UART3->DLL = 0x1A;
	LPC_UART3->DLM = 0x00;

	LPC_UART3->LCR &= ~(1<<7) ;				// Disabling DLAB (DLAB =0)
	// Enable & Reset FIFOs and set 4 char timeout for Rx
	//LPC_UART3->FCR = (1 << 0) | (1 << 6);
	//LPC_UART3->FCR |= (1 << 1) | (1 << 2);
	// UART Fifo setting
	LPC_UART3->FCR = 0x07;
	LPC_UART3->FCR &= ~(1<<3);
	LPC_UART3->FCR &= ~(3<<6);
	//LPC_UART3->IER = (IER_RBR /*| IER_RLS*/);	/* Enable UART3 interrupt */
	//LPC_UART3->IER = (1 << 0)  | (1 << 2); // B0:Rx, B1: Tx
	//NVIC_SetPriority(UART3_IRQn, 12);
	//NVIC_EnableIRQ(UART3_IRQn);
	mUARTx = UARTx;

	/* Set GPIO0.0 pin as output , GPS Reset pin is connected to this pin*/
	LPC_GPIO0->FIODIR |= (1 << 0);

	/* PIN high */
	LPC_GPIO0->FIOPIN |= (1 << 0);

	delay_ms(10);

	/* pin low*/
	LPC_GPIO0->FIOPIN &= ~(1 << 0);

	delay_ms(100);
	/* PIN high */
	LPC_GPIO0->FIOPIN |= (1 << 0);

	printf("Starting communication\n");

	int16_t timeout = 7000;


	sim808_sendCheckReply((int8_t *)("AT\r\n"),(int8_t *)"OK");


	delay_ms(1000);
	//if (sim808_sendCheckReply((int8_t *)("AT\r\n"),(int8_t *)"OK"))
#endif

	// Pin select for P4.28 TXD3 and P4.29 RXD3
	LPC_PINCON->PINSEL9 |= ((1<<24) | (1<<25) | (1<<26) | (1<<27));
	// Turn on UART3 module
	LPC_SC->PCONP |= (1 << 25);

	// Select clock for UART3 PCLK = CCLK
	LPC_SC->PCLKSEL1 &= ~(2<<18);
	LPC_SC->PCLKSEL1 |= (1<<18);

	// Configure UART3 :8-bit character length1 :1 stop bit :Parity Disabled
	LPC_UART3->LCR = 0x03;


	baud = (pclk / (16 * baudrate)) + 0.5;

	// Set baud rate
	LPC_UART3->LCR |= (1<<7);				// Enable DLAB (DLAB = 1)
	//LPC_UART3->DLM = (baud >> 8);
	//LPC_UART3->DLL = (baud & 0xFF);

	LPC_UART3->DLL = 0x1A;
	LPC_UART3->DLM = 0x00;

	LPC_UART3->LCR &= ~(1<<7) ;				// Disabling DLAB (DLAB =0)
	// Enable & Reset FIFOs and set 4 char timeout for Rx
	//LPC_UART3->FCR = (1 << 0) | (1 << 6);
	//LPC_UART3->FCR |= (1 << 1) | (1 << 2);
	// UART Fifo setting
	LPC_UART3->FCR = 0x07;
	LPC_UART3->FCR &= ~(1<<3);
	//LPC_UART3->FCR |= (3<<6);
	//LPC_UART3->IER = (IER_RBR /*| IER_RLS*/);	/* Enable UART3 interrupt */
	//LPC_UART3->IER = (1 << 0)  | (1 << 2); // B0:Rx, B1: Tx
	//NVIC_SetPriority(UART3_IRQn, 12);
	//NVIC_EnableIRQ(UART3_IRQn);
	mUARTx = UARTx;

	/* Set GPIO0.0 pin as output , GPS Reset pin is connected to this pin*/
	LPC_GPIO0->FIODIR |= (1 << 0);

	/* PIN high */
	LPC_GPIO0->FIOPIN |= (1 << 0);

	delay_ms(1000);

	/* pin low*/
	LPC_GPIO0->FIOPIN &= ~(1 << 0);

	delay_ms(1000);
	/* PIN high */
	LPC_GPIO0->FIOPIN |= (1 << 0);

	printf("Starting communication\n");

	int16_t timeout = 7000;


	//sim808_sendCheckReply((int8_t *)("AT\r\n"),(int8_t *)"OK");


	delay_ms(1000);
	//if (sim808_sendCheckReply((int8_t *)("AT\r\n"),(int8_t *)"OK"))


	return true;
}



/**************************************************************************************************
 * @brief		Receive a single character byte from UART peripheral
 * @return 		Data received
**************************************************************************************************/
uint8_t sim808_gps::sim808_getch()
{
	return (LPC_UART3->RBR & UART_RBR_MASKBIT);
}

/**************************************************************************************************
 * @brief		Send a single character byte
 * @param[in]	data	UART data to be sent
 * @return 		None
**************************************************************************************************/
void sim808_gps::sim808_putch(uint8_t data)
{
	printf("Putting character %c\n",data);
	LPC_UART3->THR = (data & UART_THR_MASKBIT);
	while(! (LPC_UART3->LSR & (1 << 6)));
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
	printf("sim808_send : Command is %s\n",txbuf);
	printf("Command length is %d\n",buflen);
	bRemain = buflen;

	bSent = 0;
	while (bRemain)
	{
		timeOut = UART_BLOCKING_TIMEOUT;
		// Wait for THR empty with timeout
		while (!(LPC_UART3->LSR & UART_LSR_THRE))
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
        while((LPC_UART3->LSR & UART_LSR_RDR))
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
uint16_t sim808_gps::sim808_readline(uint32_t timeout, bool multiline )
{
	uint16_t replyidx = 0;
	uint32_t  bRecv, lTimeOut=0;
	char recvData = 0;

	lTimeOut = 0xFFFFFFFF;

#if 1
	while (lTimeOut--)
	{
		if (replyidx >= 254)
		{
			break;
		}

		while(!(LPC_UART3->LSR & UART_LSR_RDR))
		{
			if (lTimeOut == 0)
			{
				printf("TIMEOUT\n");
				break;
			}
			lTimeOut--;
		}


		recvData  = sim808_getch();
		getChar(&recvData);
		if (recvData == '\r')
			continue;

		if (recvData == 0xA)
		{
			if (replyidx == 0)   // the first 0x0A is ignored
				continue;

			if (!multiline)
			{
				lTimeOut = 0;         // the second 0x0A is the end of the line
				break;
			}
		}

			printf("Received character is %c\n",recvData);
			replybuffer[replyidx] = recvData;
			replyidx++;

	}
#endif
	replybuffer[replyidx] = 0;  // null term
	return replyidx;
}

/**************************************************************************************************
 * @brief		Receive a single data from UART peripheral
 * @param[in]	UARTx	UART peripheral selected, should be:
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
uint8_t sim808_gps::sim808_getReply(int8_t *send, bool multiline )
{
	uint8_t readLength =0;

	printf("sim808_getReply : Command is %s\n",send);
	//sim808_flushin();

	sim808_send(send);

	delay_ms(1000);
	readLength = UART_Receive(replybuffer,sizeof(replybuffer),NONE_BLOCKING);

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
	printf("sim808_sendCheckReply : Command is %s\n",send);

	if(sim808_getReply(send) <= 0) // added for testing
	{
		printf("Error in get reply\n");
		return false;
	}

	//return true;
	printf("replybuffer --> %s\n",replybuffer);
	printf("reply       --> %s\n",reply);
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
	if(!sim808_sendCheckReply((int8_t *)"ATE0\r\n",(int8_t *) "OK"))
	{
		printf("ATE0 error\n");
		return false;
	}

	delay_ms(1000);

	if (onoff)
	{
		if (!sim808_sendCheckReply((int8_t *)"AT+CGNSPWR=1\r\n",(int8_t *) "OK"))
		{
			printf("AT+CGNSPWR=1 error\n");
			return false;
		}
	}

	else if (!onoff)
	{
		if (!sim808_sendCheckReply((int8_t *)"AT+CGNSPWR=0\r\n",(int8_t *) "OK"))
		{
			printf("AT+CGNSPWR=0 error\n");
			return false;
		}
	}

	delay_ms(1000);

	if(!sim808_sendCheckReply((int8_t*)"AT+CGNSSEQ=\"RMC\"\r\n",(int8_t* )"OK"))
	{
		printf("AT+CGNSSEQ=\"RMC\" error\n");
		return false;
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
	sim808_getReply((int8_t *)"AT+CGNSINF\r\n",true);
#if 0
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
#endif
	//sim808_readline(); // eat 'OK'
	//return len;
	return 0;
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
bool sim808_gps::getGPS(float *lat, float *lon, float *speed_kph, float *heading, float *altitude)
{
	int8_t gpsbuffer[256];
	uint8_t res_len = 0;

	// grab the mode 2^5 gps csv from the sim808
	res_len = getGPS(32, gpsbuffer, 120);
#if 0
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
#endif
	return true;
}
#endif
