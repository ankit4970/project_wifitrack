/*
 * gps.hpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ankit
 */

#ifndef L5_APPLICATION_GPS_HPP_
#define L5_APPLICATION_GPS_HPP_


/**
 * Acceleration Sensor class used to get acceleration data reading from the on-board sensor.
 * Acceleration data reading can provide absolute tilt of the board (if under no movement),
 * and it can also provide the movement activity of the board.
 *
 * @ingroup BoardIO
 */
#include <io.hpp>
#include "utilities.h"
#include "uart2.hpp"
#include "common.hpp"

#define FONA_HEADSETAUDIO 0
#define FONA_EXTAUDIO 1

#define FONA_STTONE_DIALTONE 1
#define FONA_STTONE_BUSY 2
#define FONA_STTONE_CONGESTION 3
#define FONA_STTONE_PATHACK 4
#define FONA_STTONE_DROPPED 5
#define FONA_STTONE_ERROR 6
#define FONA_STTONE_CALLWAIT 7
#define FONA_STTONE_RINGING 8
#define FONA_STTONE_BEEP 16
#define FONA_STTONE_POSTONE 17
#define FONA_STTONE_ERRTONE 18
#define FONA_STTONE_INDIANDIALTONE 19
#define FONA_STTONE_USADIALTONE 20

#define SIM808_DEFAULT_TIMEOUT_MS (0xFFFFFFFFUL)

#define FONA_HTTP_GET   0
#define FONA_HTTP_POST  1
#define FONA_HTTP_HEAD  2

#define FONA_CALL_READY 		0
#define FONA_CALL_FAILED 		1
#define FONA_CALL_UNKNOWN 		2
#define FONA_CALL_RINGING 		3
#define FONA_CALL_INPROGRESS 	4



/* Accepted Error baud rate value (in percent unit) */
#define UART_ACCEPTED_BAUDRATE_ERROR	(3)			/*!< Acceptable UART baudrate error */
/*********************************************************************//**
 * Macro defines for Macro defines for UART line control register
 **********************************************************************/
#define UART_LCR_WLEN5     		((uint8_t)(0))   		/*!< UART 5 bit data mode */
#define UART_LCR_WLEN6     		((uint8_t)(1<<0))   	/*!< UART 6 bit data mode */
#define UART_LCR_WLEN7     		((uint8_t)(2<<0))   	/*!< UART 7 bit data mode */
#define UART_LCR_WLEN8     		((uint8_t)(3<<0))   	/*!< UART 8 bit data mode */
#define UART_LCR_STOPBIT_SEL	((uint8_t)(1<<2))   	/*!< UART Two Stop Bits Select */
#define UART_LCR_PARITY_EN		((uint8_t)(1<<3))		/*!< UART Parity Enable */
#define UART_LCR_PARITY_ODD		((uint8_t)(0))         	/*!< UART Odd Parity Select */
#define UART_LCR_PARITY_EVEN	((uint8_t)(1<<4))		/*!< UART Even Parity Select */
#define UART_LCR_PARITY_F_1		((uint8_t)(2<<4))		/*!< UART force 1 stick parity */
#define UART_LCR_PARITY_F_0		((uint8_t)(3<<4))		/*!< UART force 0 stick parity */
#define UART_LCR_BREAK_EN		((uint8_t)(1<<6))		/*!< UART Transmission Break enable */
#define UART_LCR_DLAB_EN		((uint8_t)(1<<7))    	/*!< UART Divisor Latches Access bit enable */
#define UART_LCR_BITMASK		((uint8_t)(0xFF))		/*!< UART line control bit mask */
/*********************************************************************//**
 * Macro defines for Macro defines for UARTn Divisor Latch MSB register
 **********************************************************************/
#define UART_DLM_MASKBIT	((uint8_t)0xFF)			/*!< Divisor latch MSB bit mask */
#define UART_LOAD_DLM(div)  (((div) >> 8) & 0xFF)	/**< Macro for loading most significant halfs of divisors */
/*********************************************************************//**
 * Macro defines for Macro defines for UARTn Divisor Latch LSB register
 **********************************************************************/
#define UART_LOAD_DLL(div)	((div) & 0xFF)	/**< Macro for loading least significant halfs of divisors */
#define UART_DLL_MASKBIT	((uint8_t)0xFF)	/*!< Divisor latch LSB bit mask */
/*********************************************************************//**
 * Macro defines for Macro defines for UART Fractional divider register
 **********************************************************************/
#define UART_FDR_DIVADDVAL(n)	((uint32_t)(n&0x0F))		/**< Baud-rate generation pre-scaler divisor */
#define UART_FDR_MULVAL(n)		((uint32_t)((n<<4)&0xF0))	/**< Baud-rate pre-scaler multiplier value */
#define UART_FDR_BITMASK		((uint32_t)(0xFF))			/**< UART Fractional Divider register bit mask */

/* --------------------- BIT DEFINITIONS -------------------------------------- */

#define GPS_RESET_PIN 0

class sim808_gps :  public SingletonTemplate<sim808_gps>
{
    public:
        bool sim808_gpsInit(LPC_UART_TypeDef *UARTx,uint32_t baudrate);
		bool enableGPS(bool onoff);
		bool getGPS(senseordata *sensor);
		bool sim808_parseReply(int8_t *toreply,uint16_t *v, int8_t divider, uint8_t index);
		bool sim808_sendCheckReply(int8_t *send, int8_t *reply);
		bool sendParseReply(int8_t * tosend,int8_t* toreply,uint16_t *v, int8_t divider= ',', uint8_t index=0);
		void sim808_putch(uint8_t data);
		void sim808_flushin();
		uint8_t sim808_getReply(int8_t *send,bool multiline = false);
		uint32_t sim808_send(int8_t *txbuf);
		uint8_t sim808_getch();
		bool changeBaudRate(uint32_t baudRate);
		void sim808_reset(LPC_GPIO_TypeDef *GPIOx,uint8_t pinNumber);
		uint8_t gps_uartReceive( uint8_t *rxbuf, uint32_t buflen,TRANSFER_BLOCK_Type flag, bool multiline = false);
		QueueHandle_t gpsTxQueue = NULL;         ///< Queue for UARTs transmit buffer

    private:
		sim808_gps()  ///< Private constructor of this Singleton class
    	{
		}
		friend class SingletonTemplate<sim808_gps>;  ///< Friend class used for Singleton Template
		uint8_t replybuffer[MAX_BUFFER_SIZE]={0};
		volatile uint32_t   UART3RxQueueWritePos = 0;
		LPC_UART_TypeDef* mUARTx = NULL;

};

#endif /* L5_APPLICATION_GPS_HPP_ */
