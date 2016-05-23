/*
 * common.hpp
 *
 *  Created on: May 12, 2016
 *      Author: ankit
 */
#include <stdlib.h>

#ifndef L5_APPLICATION_COMMON_HPP_
#define L5_APPLICATION_COMMON_HPP_

/*********************************************************************//**
 * Macro defines for Macro defines for UARTn Transmit Holding Register
 **********************************************************************/
#define UART_THR_MASKBIT   	((uint8_t)0xFF) 		/*!< UART Transmit Holding mask bit (8 bits) */

#define UART_IIR_INTSTAT_PEND	((uint32_t)(1<<0))	/*!<Interrupt Status - Active low */
#define UART_IIR_INTID_RLS		((uint32_t)(3<<1)) 	/*!<Interrupt identification: Receive line status*/
#define UART_IIR_INTID_RDA		((uint32_t)(2<<1)) 	/*!<Interrupt identification: Receive data available*/
#define UART_IIR_INTID_CTI		((uint32_t)(6<<1)) 	/*!<Interrupt identification: Character time-out indicator*/
#define UART_IIR_INTID_THRE		((uint32_t)(1<<1)) 	/*!<Interrupt identification: THRE interrupt*/
#define UART1_IIR_INTID_MODEM	((uint32_t)(0<<1)) 	/*!<Interrupt identification: Modem interrupt*/
#define UART_IIR_INTID_MASK		((uint32_t)(7<<1))	/*!<Interrupt identification: Interrupt ID mask */
#define UART_IIR_FIFO_EN		((uint32_t)(3<<6)) 	/*!<These bits are equivalent to UnFCR[0] */
#define UART_IIR_ABEO_INT		((uint32_t)(1<<8)) 	/*!< End of auto-baud interrupt */
#define UART_IIR_ABTO_INT		((uint32_t)(1<<9)) 	/*!< Auto-baud time-out interrupt */
#define UART_IIR_BITMASK		((uint32_t)(0x3CF))	/*!< UART interrupt identification register bit mask */

/*********************************************************************//**
 * Macro defines for Macro defines for UARTn Receiver Buffer Register
 **********************************************************************/
#define UART_RBR_MASKBIT   	((uint8_t)0xFF) 		/*!< UART Received Buffer mask bit (8 bits) */

#define UART_BLOCKING_TIMEOUT 0xffffffff

#define MAX_BUFFER_SIZE 255

typedef enum
{
	NONE_BLOCKING = 0,		///< None Blocking type
	BLOCKING				///< Blocking type
} TRANSFER_BLOCK_Type;

typedef struct sensorData{
	uint16_t s_year;			///< Blocking type
	uint8_t s_month;			///< Stores month
	uint8_t s_day;				///< Stores day
	uint8_t s_hour;				///< Stores hour
	uint8_t s_min;				///< Stores minute
	uint8_t s_sec;				///< Stores second
	float temperature;			///< Stores temperature
	float lightPercentValue;	///< Stores Light Percent Value
	float latitude;				///< Stores Latitude
	float longitude;			///< Stores Longitude
}senseordata;

#endif /* L5_APPLICATION_COMMON_HPP_ */
