/*
 * esp8266_wifi.hpp
 *
 *  Created on: Apr 25, 2016
 *      Author: ankit
 */

#ifndef L5_APPLICATION_ESP8266_WIFI_HPP_
#define L5_APPLICATION_ESP8266_WIFI_HPP_


/*
 * esp8266.hpp
 *
 *  Created on: Mar 31, 2016
 *      Author: ankit
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "uart3.hpp"

#define ESP8266_STATION 	0x01
#define ESP8266_SOFTAP 		0x02

#define ESP8266_TCP 		1
#define ESP8266_UDP 		0

#define ESP8266_OK 			1
#define ESP8266_READY 		2
#define ESP8266_FAIL 		3
#define ESP8266_NOCHANGE 	4
#define ESP8266_LINKED 		5
#define ESP8266_UNLINK 		6


class esp8266_wifi : public SingletonTemplate<esp8266_wifi>
{
    public:
        //bool init();   ///< Initializes the sensor
        bool esp8266_init(LPC_UART_TypeDef *UARTx);
        bool esp8266_init_change();
        /**
        * Set the module as "client" and connects to `ssid` with `password`.
        */
        unsigned char esp8266_connect(const int8_t * ssid, const int8_t * passwd);
        void esp8266_disconnect(void);
        /**
		* Send a `HIGH` signal to `resetPin` so that it hard resets the module.
		*/
		void esp8266_hardReset();
		/**
		* Read incoming data from the TCP server and return them. Note that
		* `mux_id` and `length` are updated with the right values.
		*/
		bool esp8266_read(uint8_t * data, uint16_t max_length, bool discard_headers);
		/**
		* Send data to a TCP channel (`mux_id`).
		*/
		bool esp8266_write(const uint32_t mux_id, uint8_t* data);
		/**
		* Close the TCP connection with a client, not the serial connection (see
		* `end()` method for that).
		*/
		bool esp8266_close(const uint32_t mux_id);

		bool esp8266_isStarted(void);
		void esp8266_mode(uint8_t mode);
		void esp8266_putch(int8_t);
		bool esp8266_send(int8_t* data,uint8_t datalen);
		int8_t esp8266_getch();
		void esp8266_print(int8_t *ptr);
		uint8_t esp8266_waitResponse(void);
		//uint16_t esp8266_waitFor(unsigned char *string) ;
		bool esp8266_reset(void);
		void esp8266_getIp(uint8_t* ipAddr);
		bool esp8266_setup(uint8_t protocol, const int8_t* ip, uint16_t port);
		bool esp8266_getFirmwareVersion(void);
		uint16_t esp8266_waitFor(uint8_t *string);
		bool esp8266_watchdog_disable();
		bool esp8266_baudrate_change(uint16_t baudRate);
    private:
		esp8266_wifi()  ///< Private constructor of this Singleton class
		{

		}
		friend class SingletonTemplate<esp8266_wifi>;  ///< Friend class used for Singleton Template
		LPC_UART_TypeDef* mUARTx = NULL;
};




#endif /* L5_APPLICATION_ESP8266_WIFI_HPP_ */
