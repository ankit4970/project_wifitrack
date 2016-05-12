/*
 * esp8266.cpp
 *
 *  Created on: Mar 31, 2016
 *      Author: ankit
 */

#include <string.h>
#include "esp8266_wifi.hpp"
#include "utilities.h"
/*

	AT+CWJAP="The_Blue_Pill","TheArchitect1322260"

	AT+CIPSTART="TCP","10.0.0.179",80

	AT+CIPSEND=73

	>GET /data.php?temperature=56&lightvalue=23 HTTP/1.1\r\nHost: 10.0.0.179\r\n\r\n

 * */
/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
 * Note: when using UART in BLOCKING mode, a time-out condition is used
 * 		 via defined symbol UART_BLOCKING_TIMEOUT.
**************************************************************************************************/
bool esp8266_wifi::esp8266_init(LPC_UART_TypeDef *UARTx)
{
	// Turn on UART3 module
	LPC_SC->PCONP |= (1 << 25);

	// Select clock for UART3 PCLK = CCLK
	LPC_SC->PCLKSEL1 &= ~(2<<18);
	LPC_SC->PCLKSEL1 |= (1<<18);

	// Configure UART3 :8-bit character length1 :1 stop bit :Parity Disabled
	LPC_UART3->LCR = 0x03;

	// Enable DLAB (DLAB = 1)
	LPC_UART3->LCR |= (1<<7);

	// For baudrate 115200
	//div = 48000000/16*baudRate;// = 312;
	//divWord = 48000000/(16*115200);


	LPC_UART3->DLL = 0x1A;
	LPC_UART3->DLM = 0;

	// Disabling DLAB (DLAB =0)
	LPC_UART3->LCR &= ~(1<<7) ;

	// Pin select for P4.28 TXD3 and P4.29 RXD3
	LPC_PINCON->PINSEL9 |= ((1<<24) | (1<<25) | (1<<26) | (1<<27));

	mUARTx = UARTx;
    return true;
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
bool esp8266_wifi::esp8266_init_change()
{

	// Turn on UART3 module
	LPC_SC->PCONP |= (1 << 25);

	// Select clock for UART3 PCLK = CCLK
	LPC_SC->PCLKSEL1 &= ~(2<<18);
	LPC_SC->PCLKSEL1 |= (1<<18);

	// Configure UART3 :8-bit character length1 :1 stop bit :Parity Disabled
	LPC_UART3->LCR = 0x03;

	// Enable DLAB (DLAB = 1)
	LPC_UART3->LCR |= (1<<7);

	// For baudrate 9600
	//div = 48000000/16*baudRate;// = 312;

	//divWord = 48000000/(16*9600);

	// For baudrate 57600
	LPC_UART3->DLL = 0x34;
	LPC_UART3->DLM = 0;

	// Disabling DLAB (DLAB =0)
	LPC_UART3->LCR &= ~(1<<7) ;

	// Pin select for P4.28 TXD3 and P4.29 RXD3
	LPC_PINCON->PINSEL9 |= ((1<<24) | (1<<25) | (1<<26) | (1<<27));

	// Enable IRQ vector for UART3
	//NVIC_EnableIRQ(UART3_IRQn);

	//Enable RX interrupt
	//LPC_UART3->IER |= 1;

    return true;
}


/**************************************************************************************************
 * @brief		Output a string to the ESP module.
 * @param[in]	ptr 	A pointer to the string to send.
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
void esp8266_wifi::esp8266_print(int8_t *ptr)
{
    while (*ptr != 0)
    {
    	esp8266_putch(*ptr++);
    }
    return;
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
uint16_t esp8266_wifi::esp8266_waitFor(uint8_t *string)
{
	uint8_t so_far = 0;
	uint8_t received;
    uint16_t counter = 0;
    do
    {
        received = esp8266_getch();
        counter++;
        if (received == string[so_far])
        {
            so_far++;
        }
        else
        {
            so_far = 0;
        }
    } while (string[so_far] != 0);
    return counter;
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
unsigned char esp8266_wifi::esp8266_waitResponse(void)
{
	uint8_t response = 0, i =0;
	uint8_t so_far[6] = {0,0,0,0,0,0};
	const  uint8_t lengths[6] = {2,5,4,9,6,6};
    const  uint8_t strings[6][16] = {"OK", "ready", "FAIL", "no change", "Linked", "Unlink"};
    const  uint8_t responses[6] = {ESP8266_OK, ESP8266_READY, ESP8266_FAIL, ESP8266_NOCHANGE, ESP8266_LINKED, ESP8266_UNLINK};
    uint8_t received;

    bool continue_loop = true;
    while (continue_loop)
    {
        received = esp8266_getch();
        for (i = 0; i < 6; i++)
        {
            if (strings[i][so_far[i]] == received)
            {
                so_far[i]++;
                if (so_far[i] == lengths[i])
                {
                    response = responses[i];
                    continue_loop = false;
                }
            }
            else
            {
                so_far[i] = 0;
            }
        }
    }
    return response;
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
int8_t esp8266_wifi::esp8266_getch()
{
	return LPC_UART3->RBR;
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
**************************************************************************************************/
void esp8266_wifi::esp8266_putch(int8_t data)
{
	while(!(LPC_UART3->LSR & (1<<5)));
	LPC_UART3->THR = data;
}


/**************************************************************************************************
 * @brief		Open a TCP or UDP connection.
 * 				This sends the AT+CIPSTART command to the ESP module.
 * @param[in]	protocol: Either ESP8266_TCP or ESP8266_UDP
 * @param[in]	ip 		: The IP or hostname to connect to; as a string
 * @param[in]	port	: The port to connect to
 * @return 		true	: If the connection is opened successfully
**************************************************************************************************/
bool esp8266_wifi::esp8266_setup(uint8_t protocol, const int8_t* ip, uint16_t port)
{
	uint8_t port_str[5] = {0};
	esp8266_print((int8_t *)"AT+CIPSTART=\"");
    if (protocol == ESP8266_TCP)
    {
        esp8266_print((int8_t *)"TCP");
    }
    else
    {
        esp8266_print((int8_t *)"UDP");
    }

    esp8266_print((int8_t *)"\",\"");
    esp8266_print((int8_t *)ip);
    esp8266_print((int8_t *)"\",");

    sprintf((char *)port_str, "%u", port);

    esp8266_print((int8_t *)port_str);
    esp8266_print((int8_t *)"\r\n");
    /*if (esp8266_waitResponse() != ESP8266_OK)
    {
        return 0;
    }
    if (esp8266_waitResponse() != ESP8266_LINKED)
    {
        return 0;
    }*/

    return 1;
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
**************************************************************************************************/
bool esp8266_wifi::esp8266_baudrate_change(uint16_t baudRate)
{
	uint8_t tempBuffer[6] = {0};
	sprintf((char *)tempBuffer, "%d%s%s", baudRate,'\r','\n');
	esp8266_print((int8_t *)tempBuffer);
	return true;
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
bool esp8266_wifi::esp8266_watchdog_disable()
{
	esp8266_print((int8_t *)"AT+CSYSWDTDISABLE\r\n");
	return true;
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
bool esp8266_wifi::esp8266_send(int8_t *data,uint8_t datalen)
{
	uint8_t length_str[6] = {0};
    sprintf((char *)length_str, "%d", datalen);

    esp8266_print((int8_t *)"AT+CIPSEND=");
    esp8266_print((int8_t *)length_str);
    esp8266_print((int8_t *)"\r\n");
    delay_ms(200);
    //while (esp8266_getch() != '>');
    esp8266_print(data);
    //if (esp8266_waitResponse() == ESP8266_OK)
    {
        //return 1;
    }
    return true;
}


/**
 * Set the WiFi mode.
 *
 * ESP8266_STATION : Station mode
 * ESP8266_SOFTAP : Access point mode
 *
 * This sends the AT+CWMODE command to the ESP module.
 *
 * @param mode an ORed bitmask of ESP8266_STATION and ESP8266_SOFTAP
 */
/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
void esp8266_wifi::esp8266_mode(uint8_t mode)
{
	esp8266_print((int8_t *)"AT+CWMODE=");
	esp8266_putch(mode + '0');
    esp8266_print((int8_t *)"\r\n");
    //esp8266_waitResponse();
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
bool esp8266_wifi::esp8266_isStarted(void)
{
    esp8266_print((int8_t *)"AT\r\n");
    return (esp8266_waitResponse() == ESP8266_OK);
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
uint8_t esp8266_wifi::esp8266_connect(const int8_t * ssid, const int8_t * passwd)
{
	esp8266_print((int8_t *)"AT+CWJAP=\"");
	esp8266_print((int8_t *)ssid);
	esp8266_print((int8_t *)"\",\"");
	esp8266_print((int8_t *)passwd);
	esp8266_print((int8_t *)"\"\r\n");
	return 0;//esp8266_waitResponse();
}


/**************************************************************************************************
 * @brief		Disconnect from the access point.
 * 				This sends the AT+CWQAP command to the ESP module.
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
void esp8266_wifi::esp8266_disconnect(void)
{
    esp8266_print((int8_t *)"AT+CWQAP\r\n");
    esp8266_waitFor((uint8_t *)"OK");
}


/**************************************************************************************************
 * @brief		Send a block of data via UART peripheral
 * @param[in]	txbuf 	Pointer to Transmit buffer
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
void esp8266_wifi::esp8266_getIp(uint8_t* ipAddr)
{
	uint8_t received;

	esp8266_print((int8_t *)"AT+CIFSR\r\n");

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
   // esp8266_waitFor((unsigned char *)"OK");
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

	/*sprintf(store_in, "%u,%u:%c%c", length, max_length, _esp8266_getch(), _esp8266_getch());
	return;*/

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
 * @brief		Send a block of data via UART peripheral
 *
 * @return 		Number of bytes sent.
 *
**************************************************************************************************/
bool esp8266_wifi::esp8266_getFirmwareVersion(void)
{
    esp8266_print((int8_t *)"AT+GMR\r\n");

    if (esp8266_waitResponse() != ESP8266_OK)
    {
        return false;
    }

    return (esp8266_waitResponse() == ESP8266_READY);
}


/**************************************************************************************************
 * @brief		Restart the ESP module
 * @return 		true if the module restarted properly
**************************************************************************************************/
bool esp8266_wifi::esp8266_reset(void)
{
	char data = 0;
	uint8_t i =0;
	printf("INside reset function\n");
	esp8266_print((int8_t *)"AT+RST\r\n");

    //if (esp8266_waitResponse() != ESP8266_OK)
    {
      //  return false;
    }
    printf("Getting out of reset function\n");

    data = esp8266_getch();
    printf("Received data 1 is %c\n",data);

    data = esp8266_getch();
	printf("Received data 2 is %c\n",data);


    return true;//(esp8266_waitResponse() == ESP8266_READY);
}


