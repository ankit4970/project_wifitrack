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

const int8_t wifi_ssid[32]="The_Blue_Pill";
const int8_t wifi_passwd[32]="TheArchitect1322260";
const int8_t wifi_serverip[32] = "10.0.0.179";
const uint16_t wifi_server_port = 8080;
SemaphoreHandle_t gSensorSemaphore = NULL;
unsigned char ipBuffer[64]={};
#define GPS_BAUD_RATE 9600
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

// UART3 Interrupt handler
extern "C"
{
	void UART2_IRQHandler(void)
	{
		int i =0;
		char data =0;
		const uint16_t transmitterEmpty = (1 << 1);
		const uint16_t dataAvailable    = (2 << 1);
		const uint16_t dataTimeout      = (6 << 1);

		uint16_t reasonForInterrupt = (LPC_UART2->IIR & 0xE);

			switch (reasonForInterrupt)
			{
				case transmitterEmpty:
			    	printf("transmitterEmpty\n");
			    	break;

				case dataAvailable:
					data = LPC_UART2->RBR;
					printf("--> %c\n",data);
					break;

				case dataTimeout:
					printf("dataTimeout\n");
					break;
		}

//		for(i=0;i<16;i++)
//		{
//			data = LPC_UART3->RBR;
//			printf("--> %c \n",data);
//		}
	}
}
volatile uint8_t data =0;
#if 0
// UART3 Interrupt handler
extern "C"
{
	void UART3_IRQHandler(void)
	{
		uint8_t IIRValue, LSRValue;
		static uint8_t i =0;

		i++;
		while ((LPC_UART3->LSR & (1 << 0)))
		{
			data = LPC_UART3->RBR;
			printf("<%d-- %c\n",i,data);

		}

	}
}
#endif
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

        	return true;
        }

        bool run(void *p)
        {
        	sensor.temperature = TS.getCelsius();
        	sensor.lightPercentValue = LS.getPercentValue();

        	gps.getGPS(&sensor.latitude,&sensor.longitude,&sensor.speed_kph,&sensor.heading,&sensor.altitude);
        	//sensor.position = LSM.getPosition();
        	printf("Temperature is %f\n",sensor.temperature);
        	//printf("lightPercentValue is %d\n",sensor.lightPercentValue);
        	//printf("latitude is %f\n",sensor.latitude);
        	//printf("longitude is %f\n",sensor.longitude);
        	//printf("speed_kph is %f\n",sensor.speed_kph);
        	//printf("heading is %f\n",sensor.heading);
        	//printf("altitude is %f\n",sensor.altitude);
        	//xSemaphoreGive(gSensorSemaphore);

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
        	wifi.esp8266_init(LPC_UART3);
        	delay_ms(1000);
			wifi.esp8266_mode(1);
			delay_ms(2000);
			wifi.esp8266_connect(wifi_ssid,wifi_passwd);
			delay_ms(5000);
			wifi.esp8266_setup((uint8_t)ESP8266_TCP,wifi_serverip,wifi_server_port);
			delay_ms(5000);
			printf("Wifi setup done \n");
        	return true;

        }
        bool run(void *p)
        {
        	if (xSemaphoreTake(gSensorSemaphore, portMAX_DELAY))
        	{
				printf("Semaphore Taken \n");
				wifi.esp8266_send((int8_t *)"ankitgandhi",sizeof("ankitgandhi"));
			}

        	delay_ms(5000);
        	return true;
        }
};

#endif


#if 0
int main(void)
{
    qh = xQueueCreate(1, sizeof(int));
    xTaskCreate(rx, "rx", 1024, NULL, PRIORITY_LOW, NULL);
    xTaskCreate(tx, "tx", 1024, NULL, PRIORITY_LOW, NULL);

    //xTaskCreate(tx1, "tx1", 2000, NULL, PRIORITY_LOW, NULL);
    //xTaskCreate(tx2, "tx2", 2000, NULL, PRIORITY_MEDIUM, NULL);
    //xTaskCreate(rx1, "rx1", 2000, NULL, PRIORITY_HIGH, NULL);
    vTaskStartScheduler();

	return 0;
}
#endif

#if 1
int main(void)
{

	gSensorSemaphore = xSemaphoreCreateBinary();
	delay_ms(10000);
	//gps.sim808_gpsInit(LPC_UART3,9600);
	//delay_ms(1000);
	//gps.changeBaudRate(115200);
	//delay_ms(1000);
	gps.sim808_gpsInit(LPC_UART3,9600);
	delay_ms(2000);
	if(!gps.enableGPS(true))
	{
		return -1;
	}
	delay_ms(2000);

	gps.getGPS(&sensor.latitude,&sensor.longitude,&sensor.speed_kph,&sensor.heading,&sensor.altitude);
	//scheduler_add_task(new sensorTask(PRIORITY_MEDIUM));
	//scheduler_add_task(new espWifiTask(PRIORITY_HIGH));
	delay_ms(500);
	//scheduler_start(); ///< This shouldn't return

	while(1)
	{
		//gps.getGPS(&sensor.latitude,&sensor.longitude,&sensor.speed_kph,&sensor.heading,&sensor.altitude);
		delay_ms(5000);
	}
	return -1;
}
#endif
#if 0
int main(void)
{
	char replybuffer[WIFI_RXQ_SIZE] = {0};
	Uart3 &u3 = Uart3::getInstance();
	u3.init(GPS_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
	//scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));

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

	delay_ms(1000);
	u3.putline("AT\r\n");
	u3.gets(replybuffer,256);
	printf("-->%s\n",replybuffer);

	u3.putline("ATE0\r\n");
	u3.gets(replybuffer,256);
	printf("-->%s\n",replybuffer);

	u3.putline("AT+CGNSPWR=1\r\n");
	u3.gets(replybuffer,256);
	printf("-->%s\n",replybuffer);

	u3.putline("AT+CGNSSEQ=\"RMC\"\r\n");
	u3.gets(replybuffer,256);
	printf("-->%s\n",replybuffer);


	u3.putline("AT+CGNSINF\r\n");
	u3.gets(replybuffer,256);
	printf("-->%s\n",replybuffer);

	while(1);
	return -1;
}
#endif

