/*
 * new_main.cpp
 *
 *  Created on: Apr 23, 2016
 *      Author: ankit
 */

#include "tasks.hpp"
#include "examples/examples.hpp"
#include <stdio.h>
#include <string.h>
#include <char_dev.hpp>
#include "utilities.h"          // printMemoryInfo()
#include "i2c2.hpp"
#include "singleton_template.hpp"
#include "i2c_base.hpp"
#include "uart0_min.h"
#include "io.hpp"
#include "acceleration_sensor.hpp"
#include "eint.h"
#include "timers.h"
#include "printf_lib.h"
#include "event_groups.h"
#include "../src/FileSystemObject.hpp"
#include "storage.hpp"
#include <stdlib.h>
#include "acceleration_sensor.hpp"
#include <unistd.h>
#include <io.hpp>

const int8_t wifi_ssid[32]="The_Blue_Pill";				/// SSID of WIFI network
const int8_t wifi_passwd[32]="TheArchitect1322260";		/// Password for above mentioned WIFI network
const int8_t wifi_serverip[32] = "10.0.0.201";	/// Server IP address
const uint16_t wifi_server_port = 8080;			/// Server listening port
char tempBuffer[256] = {0};						/// Buffer to store temporary data
bool sdCardPresent = false;						/// Represents SD card status
SemaphoreHandle_t gSensorSemaphore = NULL;		/// Semaphore for synchronization between sensor and WIFI task
senseordata sensor;								/// Global structure to store sensor data

#define 	GPS_BAUD_RATE 			9600			///	SIM808 Module Initial baudrate
#define 	ESP8266_BAUD_RATE 		115200			/// ESP8266 Module Initial baudrate


esp8266_wifi& wifi = esp8266_wifi::getInstance();	/// WIFI class global object
sim808_gps& gps = sim808_gps::getInstance();		/// SIM808 class global object


/**************************************************************************************************
 * @brief		sensorTask
 * 				This task takes sensor data at regular interval and stores in a global structure.
 * 				Then it signals WIFI task about the completion.
**************************************************************************************************/
class sensorTask : public scheduler_task
{
    public:
	sensorTask(uint8_t priority) :
            scheduler_task("sensorTask", 2000, priority)
        {
            /* Nothing to init */
        }


        bool init(void)
        {

        	gSensorSemaphore = xSemaphoreCreateBinary();
			delay_ms(10000);
			gps.sim808_gpsInit(LPC_UART2, 9600);
			delay_ms(3000);

			if(!gps.enableGPS(true))
			{
				return false;
			}
        	return true;
        }

        bool run(void *p)
        {
        	sensor.temperature = TS.getCelsius();
        	sensor.lightPercentValue = LS.getPercentValue();

        	gps.getGPS(&sensor);
        	printf("Temperature is %f\n",sensor.temperature);
			printf("Lightvalue is %f\n",sensor.lightPercentValue);
        	printf("Year is %d,\n",sensor.s_year);
        	printf("Month is %d,\n",sensor.s_month);
        	printf("Day is %d,\n",sensor.s_day);
        	printf("Time is %d:%d:%d\n",sensor.s_hour,sensor.s_min,sensor.s_sec);
        	printf("Temperature is %f\n",sensor.temperature);
        	printf("Lightvalue is %f\n",sensor.lightPercentValue);
        	printf("latitude is %f\n",sensor.latitude);
			printf("longitude is %f\n",sensor.longitude);

        	xSemaphoreGive(gSensorSemaphore);

        	vTaskDelay(2000);
        	return true;
        }


};

/**************************************************************************************************
 * @brief		espWifiTask
 * 				This task is a high priority task,it waits for the semaphore from sensor task
 * 				Once received, it sends data to TCP server(Connected in initialization)
**************************************************************************************************/
class espWifiTask : public scheduler_task
{
    public:
	espWifiTask(uint8_t priority) :
            scheduler_task("espWifiTask", 4000, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
        	printf("Wifi Initialization \n");
        	wifi.esp8266_init(LPC_UART3,ESP8266_BAUD_RATE);
        	delay_ms(2000);

        	printf("Wifi setting mode \n");
			if(!wifi.esp8266_mode(1))
			{
				printf("Wifi set mode error \n");
				return false;
			}
			delay_ms(3000);

			printf("Wifi connecting \n");
			if(!wifi.esp8266_connect(wifi_ssid,wifi_passwd))
			{
				printf("Wifi connecting error \n");
				return false;
			}
			delay_ms(5000);

			printf("Wifi setting TCP Connection\n");
			if(!wifi.esp8266_setup((uint8_t)ESP8266_TCP,wifi_serverip,wifi_server_port))
			{
				printf("Wifi setting TCP Connection \n");
				return false;
			}
			delay_ms(2000);

			printf("Wifi setup done \n");

        	return true;

        }
        bool run(void *p)
        {
        	if (xSemaphoreTake(gSensorSemaphore, portMAX_DELAY))
        	{
				printf("Semaphore Taken \n");
				sprintf(tempBuffer,"%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.6f,%.6f\n",sensor.s_year,sensor.s_month, \
						sensor.s_day,sensor.s_hour,sensor.s_min,sensor.s_sec,sensor.temperature,sensor.lightPercentValue,\
						sensor.latitude,sensor.longitude);
				wifi.esp8266_sendData((int8_t *)tempBuffer,(strlen(tempBuffer)));

				if(true == sdCardPresent)
				{
					Storage::append("1:sensordata.txt",tempBuffer,strlen(tempBuffer),0);
				}

				memset(tempBuffer,0,sizeof(tempBuffer));
			}

        	return true;
        }
};

/**************************************************************************************************
 * @brief	main
 * 			Main entry of the program
 * 			Checks for the SD card presence
 * 			Creates two tasks --> espWifiTask
 * 						      --> sensorTask
 * 			Starts the scheduler
**************************************************************************************************/
int main(void)
{
	FileSystemObject& drive =Storage::getSDDrive();
	unsigned int totalKb = 0;
	unsigned int availKb = 0;
	const char st = drive.mount();
	bool mounted = (0 == st);

	// Check if SD card is present
	if(mounted && FR_OK == drive.getDriveInfo(&totalKb, &availKb))
	{
		const unsigned int maxBytesForKbRange = (32 * 1024);
		const char *size = (totalKb < maxBytesForKbRange) ? "KB" : "MB";
		unsigned int div = (totalKb < maxBytesForKbRange) ? 1 : 1024;
		printf("%s: OK -- Capacity %-5d%s, Available: %-5u%s\n","SD_card", totalKb/div, size, availKb/div, size);
		sdCardPresent = true;
	}
	else
	{
		printf("%s: Error or not present.  Error #%i, Mounted: %s\n","SD_card", st, mounted ? "Yes" : "No");
	}


	scheduler_add_task(new espWifiTask(PRIORITY_HIGH));
	scheduler_add_task(new sensorTask(PRIORITY_MEDIUM));

	scheduler_start(); 				///< This shouldn't return


	return -1;
}



