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
//#define	WIFI_SSID     	"The_Blue_Pill"
//#define WIFI_PASSWD  	"TheArchitect1322260"

const char wifi_ssid[32]="The_Blue_Pill";
const char wifi_passwd[32]="TheArchitect1322260";
const char wifi_serverip[32] = "10.0.0.179";
uint16_t wifi_server_port = 8080;
SemaphoreHandle_t gSensorSemaphore = NULL;
unsigned char ipBuffer[64]={};

typedef struct sensorData{
	float temperature;
	uint8_t lightPercentValue;
	int16_t xAxis;  		///< @returns X-Axis value
	int16_t yAxis;  		///< @returns Y-Axis value
	int16_t zAxis;  		///< @returns Z-Axis value
	int8_t 	position;		///< @returns Position
	float latitude;
	float longitude;
	float speed_kph;
	float heading;
	float altitude;
}senseordata;

esp8266_wifi& wifi = esp8266_wifi::getInstance();
sim808_gps& gps = sim808_gps::getInstance();

senseordata sensor;
#if 1
/**
 * sensorTask
 */
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
        	gps.enableGPS(true);
        	return true;
        }

        bool run(void *p)
        {
        	sensor.temperature = TS.getCelsius();
        	sensor.lightPercentValue = LS.getPercentValue();
        	//sensor.position = LSM.getPosition();
        	printf("Temperature is %f\n",sensor.temperature);
        	// Sensor readings taken signal Semaphore
        	xSemaphoreGive(gSensorSemaphore);

        	vTaskDelay(5000);
        	return true;
        }


};

/**
 * wifiTask task
 */
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
        	return true;

        }
        bool run(void *p)
        {
        	if (xSemaphoreTake(gSensorSemaphore, portMAX_DELAY))
        	{
				printf("Semaphore Taken \n");
				wifi.esp8266_send((unsigned char *)"ankit",6);
			}

        	return true;
        }
};

#endif
int main(void)
{

	gSensorSemaphore = xSemaphoreCreateBinary();

	wifi.esp8266_init();
	//printf("Wifi init done \n");
	//wifi.esp8266_baudrate_change(57600);
	//wifi.esp8266_init_change();
	//wifi.esp8266_watchdog_disable();
	//wifi.esp8266_reset();
	delay_ms(1000);
	//for();
	//usleep(1000);
	//printf("Wifi reset done \n");
	wifi.esp8266_mode(1);
	delay_ms(2000);
	//printf("Wifi mode set done \n");
	//vTaskDelay(100);
	wifi.esp8266_connect(wifi_ssid,wifi_passwd);
	delay_ms(5000);
	//printf("Wifi connection done\n");
	//wifi.esp8266_getIp(ipBuffer);
	//printf("Assigned ip address is %s\n",ipBuffer);
	wifi.esp8266_setup((uint8_t)ESP8266_TCP,wifi_serverip,wifi_server_port);
	delay_ms(5000);
	printf("Wifi setup done \n");



	scheduler_add_task(new sensorTask(PRIORITY_MEDIUM));
	scheduler_add_task(new espWifiTask(PRIORITY_HIGH));

	scheduler_start(); ///< This shouldn't return

	return -1;
}
