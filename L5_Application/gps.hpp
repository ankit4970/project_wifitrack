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

#define SIM808_DEFAULT_TIMEOUT_MS 500

#define FONA_HTTP_GET   0
#define FONA_HTTP_POST  1
#define FONA_HTTP_HEAD  2

#define FONA_CALL_READY 		0
#define FONA_CALL_FAILED 		1
#define FONA_CALL_UNKNOWN 		2
#define FONA_CALL_RINGING 		3
#define FONA_CALL_INPROGRESS 	4


class sim808_gps :  public SingletonTemplate<sim808_gps>
{
    public:
        bool sim808_gpsInit(LPC_UART_TypeDef *UARTx);
		bool enableGPS(bool onoff);
		bool getGPS(float *lat, float *lon, float *speed_kph=0, float *heading=0, float *altitude=0);
		bool enableGPSNMEA(uint8_t nmea);
		bool sim808_parseReply(int8_t *toreply,uint16_t *v, int8_t divider, uint8_t index);
		bool sim808_sendCheckReply(int8_t *send, int8_t *reply);
		bool sendParseReply(int8_t * tosend,int8_t* toreply,uint16_t *v, int8_t divider= ',', uint8_t index=0);
		void sim808_putch(uint8_t data);
		void sim808_flushin();
		uint8_t sim808_getReply(int8_t *send);
		uint16_t sim808_readline(uint16_t timeout=SIM808_DEFAULT_TIMEOUT_MS,bool multiline = false);
		uint32_t sim808_send(int8_t *txbuf);
		int8_t sim808_getch();
		uint8_t getGPS(uint8_t arg, int8_t *buffer, uint8_t maxbuff);

    private:
		sim808_gps()  ///< Private constructor of this Singleton class
    	{
		}
		friend class SingletonTemplate<sim808_gps>;  ///< Friend class used for Singleton Template
		int8_t replybuffer[256];
		LPC_UART_TypeDef* mUARTx = NULL;
};


#endif /* L5_APPLICATION_GPS_HPP_ */
