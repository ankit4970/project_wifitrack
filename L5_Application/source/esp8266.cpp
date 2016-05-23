/*
 * esp8266.cpp
 *
 *  Created on: Mar 31, 2016
 *      Author: ankit
 */

#include <string.h>
#include "lpc_sys.h"
#include "esp8266_wifi.hpp"
#include "utilities.h"

/**************************************************************************************************
 * @brief		Initialize UART peripheral for SIM808
 * @param[in]	UARTx	UART peripheral selected
 *  					- LPC_UART0: UART0 peripheral
 * 						- LPC_UART1: UART1 peripheral
 * 						- LPC_UART2: UART2 peripheral
 * 						- LPC_UART3: UART3 peripheral
 * @return 		true if success
**************************************************************************************************/
bool esp8266_wifi::esp8266_init(LPC_UART_TypeDef *UARTx, uint32_t baudrate)
{
	uint16_t baud = 0;
	const unsigned int pclk = sys_get_cpu_clock();
	// Turn on UART3 module
	LPC_SC->PCONP |= (1 << 25);

	// Select clock for UART3 PCLK = CCLK
	LPC_SC->PCLKSEL1 &= ~(2<<18);
	LPC_SC->PCLKSEL1 |= (1<<18);

	// Configure UART3 :8-bit character length1 :1 stop bit :Parity Disabled
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
	UARTx->FCR &= ~(1<<3);

	// Pin select for P4.28 TXD3 and P4.29 RXD3
	LPC_PINCON->PINSEL9 |= ((1<<24) | (1<<25) | (1<<26) | (1<<27));

	mUARTx = UARTx;
    return true;
}

/**************************************************************************************************
 * @brief		Receive data from UART peripheral
 * @param[in]	rxbuf	: pointer to receive data buffer:
 * 				buflen	: length of the buffer
 * 				flag	: Tranfer type (Blocking or Non blocking)
 * @return 		No of bytes received
 **************************************************************************************************/
uint8_t esp8266_wifi::esp_uartReceive( uint8_t *rxbuf, uint32_t buflen,TRANSFER_BLOCK_Type flag, bool multiline)
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
			while (!(mUARTx->LSR & UART_LSR_RDR))
			{
				if (timeOut == 0)
					break;
				timeOut--;
			}

			// Time out!
			if(timeOut == 0)
				break;

			// Get data from the buffer
			recvData = esp8266_getch();
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
			if (!(mUARTx->LSR & UART_LSR_RDR))
			{
				break;
			}
			else
			{
				recvData = esp8266_getch();
				//printf("Received character is %c\n",recvData);
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

/**************************************************************************************************
 * @brief		Receive response from ESP8266
 * @param[in]	send	:	pointer Data to be sent
 * 				multiline:	If multiple lines are expected in response
 * @return 		Received data length
 **************************************************************************************************/
uint8_t esp8266_wifi::esp8266_getReply(int8_t *send, bool multiline)
{
	uint8_t readLength =0;

	esp8266_send(send,strlen((const char *)send));

	delay_ms(4000);
	readLength = esp_uartReceive(replybuffer,sizeof(replybuffer),NONE_BLOCKING, multiline);

	printf("%s\n",replybuffer);

	return readLength;
}


/**************************************************************************************************
 * @brief		Send command and check reply
 * @param[in]	send	:	pointer Data to be sent
 * 				reply	:	Expected response
 * 				multiline:	If multiple lines are expected in response
 * @return 		true if success
 **************************************************************************************************/
bool esp8266_wifi::esp8266_sendCheckReply(int8_t *send, int8_t *reply,bool multiline)
{
	if(esp8266_getReply(send, multiline) <= 0) // added for testing
	{
		printf("Error in get reply\n");
		return false;
	}

	//return true;
	printf("replybuffer --> %s\n",replybuffer);
	printf("reply       --> %s\n",reply);

	return(strcmp((const char*)replybuffer,(const char*) reply) == 0 | strcmp((const char*)replybuffer,(const char*)"OK") == 0);
}

/**************************************************************************************************
 * @brief		Send data to the ESP module.
 * @param[in]	ptr 	A pointer to the string to send.
 * @return 		Number of bytes sent.
**************************************************************************************************/
uint32_t esp8266_wifi::esp8266_send(int8_t *txbuf, uint8_t datalen)
{

	uint32_t bRemain, bSent, timeOut;
	int8_t *pChar = txbuf;
	uint32_t buflen = 0;

	buflen = datalen;
	printf("sim808_send : Command is %s\n",txbuf);
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
			esp8266_putch((*pChar++));
			bRemain--;
			bSent++;
		}
	}
	return bSent;
}



/**************************************************************************************************
 * @brief		Receive a byte from UART peripheral
 * @return 		received byte.
**************************************************************************************************/
int8_t esp8266_wifi::esp8266_getch()
{
	return (mUARTx->RBR & UART_RBR_MASKBIT);
}


/**************************************************************************************************
 * @brief		Send a byte of data via UART peripheral
 * @param[in]	data 	data to be sent
 * @return 		None
**************************************************************************************************/
void esp8266_wifi::esp8266_putch(int8_t data)
{
	mUARTx->THR = (data & UART_THR_MASKBIT);
	while(! (mUARTx->LSR & (1 << 6)));
}


/**************************************************************************************************
 * @brief		Open a TCP or UDP connection.
 * 				This sends the AT+CIPSTART command to the ESP module.
 * @param[in]	protocol: Protocol (TCP or UDP)
 * @param[in]	ip 		: The IP or hostname to connect to(as string)
 * @param[in]	port	: The port to connect to
 * @return 		true	: If the connection is opened successfully
**************************************************************************************************/
bool esp8266_wifi::esp8266_setup(uint8_t protocol, const int8_t* ip, uint16_t port)
{

	char tempBuffer[256] = {0};

    sprintf(tempBuffer,"AT+CIPSTART=\"TCP\",\"%s\",%d\r\n",ip,port);

	if(!esp8266_sendCheckReply((int8_t *)tempBuffer,(int8_t *) "CONNECT"))
	{
		printf("AT+CIPSTART error\n");
		return false;
	}

    return true;
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * 				This sends the AT+CIPSEND command followed byte data to the ESP module.
 * @param[in]	data 	:Pointer to Transmit buffer
 * 				datalen	: data length
 * @return 		true if success
**************************************************************************************************/
bool esp8266_wifi::esp8266_sendData(int8_t *data,uint8_t datalen)
{
	char tempBuffer[256] = {0};
    sprintf(tempBuffer,"AT+CIPSEND=%d\r\n",datalen);

    esp8266_send((int8_t*)tempBuffer,(uint8_t)strlen(tempBuffer));

    delay_ms(2000);
    //while (esp8266_getch() != '>');

    printf("Datalength is %d\n",datalen);
    printf("Data is %s\n",data);
    esp8266_send(data,datalen);
    printf("data sent\n");

    return true;
}


/**************************************************************************************************
 * @brief		Set mode for ESP8266
 * @param[in]	mode 	Mode for the esp8266
 * @return 		true if success
**************************************************************************************************/
bool esp8266_wifi::esp8266_mode(uint8_t mode)
{

	esp8266_reset();

	delay_ms(10000);

	esp8266_flushin();

	delay_ms(2000);
	esp8266_send((int8_t *)"ATE0\r\n",strlen("ATE0\r\n"));

	esp8266_flushin();
	delay_ms(3000);

    if(!esp8266_sendCheckReply((int8_t *)"AT+CWMODE=1\r\n",(int8_t *) "OK"))
	{
		printf("AT+CWMODE=1 error\n");
		return false;
	}

    delay_ms(2000);

    return true;
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	ssid 	: 	ssid of the network
 * 				passwd 	:	Password for the network
 * @return 		true if success
**************************************************************************************************/
bool esp8266_wifi::esp8266_connect(const int8_t * ssid, const int8_t * passwd)
{
	char tempBuffer[256] = {0};

	sprintf(tempBuffer,"AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,passwd);

	if(!esp8266_sendCheckReply((int8_t *)tempBuffer,(int8_t *) "OK"))
	{
		printf("AT+CWJAP error\n");
		return false;
	}

	return true;//esp8266_waitResponse();
}


/**************************************************************************************************
 * @brief		Disconnect from the access point.
 * 				This sends the AT+CWQAP command to the ESP module.
 * @return 		true if success
**************************************************************************************************/
bool esp8266_wifi::esp8266_disconnect(void)
{

    if(!esp8266_sendCheckReply((int8_t *)"AT+CWQAP\r\n",(int8_t *) "OK"))
	{
		printf("AT+CWQAP error\n");
		return false;
	}

    return true;
}


/**************************************************************************************************
 * @brief		Get IP assigned by the network
 * @param[in]	ipAddr :Pointer to IP address buffer buffer
 * @return 		None
**************************************************************************************************/
void esp8266_wifi::esp8266_getIp(uint8_t* ipAddr)
{
	uint8_t received;

	if(esp8266_getReply((int8_t *)"AT+CIFSR\r\n") <= 0) // added for testing
	{
		printf("Error in get reply\n");
		return ;
	}

    do
    {
        received = esp8266_getch();
    } while (received < '0' || received > '9');

    for (uint8_t i = 0; i < 4; i++)
    {
    	ipAddr[i] = 0;
        do
        {
        	ipAddr[i] = 10 * ipAddr[i] + received - '0';
            received = esp8266_getch();
            printf("******IP address is %d\n",ipAddr[i]);
        } while (received >= '0' && received <= '9');
        received = esp8266_getch();
    }

}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	data 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
bool esp8266_wifi::esp8266_read(uint8_t * data, uint16_t max_length, bool discard_headers)
{
	esp8266_waitFor((uint8_t *)"+IPD,");
	uint16_t length = 0;
	uint8_t received = esp8266_getch();
	do
	{
		length = length * 10 + received - '0';
		received = esp8266_getch();
	} while (received >= '0' && received <= '9');

	if (discard_headers)
	{
		length -= esp8266_waitFor((uint8_t *)"\r\n\r\n");
	}

	if (length < max_length)
	{
		max_length = length;
	}


	uint16_t i;
	for (i = 0; i < max_length; i++)
	{
		data[i] = esp8266_getch();
	}

	data[i] = 0;

	for (; i < length; i++)
	{
		esp8266_getch();
	}

	esp8266_waitFor((uint8_t *)"OK");

	return true;
}


/**************************************************************************************************
 * @brief		Get Firmware version of ESP module
 * @return 		true if success
**************************************************************************************************/
bool esp8266_wifi::esp8266_getFirmwareVersion(void)
{
    if(!esp8266_sendCheckReply((int8_t *)"AT+GMR\r\n",(int8_t *) "OK"))
	{
		printf("AT+GMR error\n");
		return false;
	}

    return true;
}

/**************************************************************************************************
 * @brief		Flush input stream of UART peripheral
 * @return 		None
**************************************************************************************************/
void esp8266_wifi::esp8266_flushin()
{
    // Read all available serial input to flush pending data.
    uint16_t timeoutloop = 0;

    while (timeoutloop++ < 40)
    {
        while((mUARTx->LSR & UART_LSR_RDR))
        {
        	esp8266_getch();
            timeoutloop = 0;  // If char was received reset the timer
        }
        delay_ms(1);
    }
}

/**************************************************************************************************
 * @brief		Restart the ESP module
 * @return 		true if the module restarted properly
**************************************************************************************************/
bool esp8266_wifi::esp8266_reset(void)
{
	printf("Inside reset function\n");

	esp8266_send((int8_t *)"AT+RST\r\n",strlen("AT+RST\r\n"));

    return true;
}


