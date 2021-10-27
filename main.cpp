/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "MMA8451Q.h"
#include "TCS3472_I2C.h"
#include "Si7021.h"

/*****************VARIABLES**************************/
//Sensors
AnalogIn input(PA_4);
MMA8451Q acc(PB_9, PB_8, 0x1d<<1);
TCS3472_I2C rgb_sensor (PB_9, PB_8);
int rgb_readings[4];
I2C tempSensor(PB_9, PB_8);
DigitalOut SENS_EN(D13);
Si7021 tempHumSensor(PB_9, PB_8);


//Modes
bool mode = true; //true = test mode, false = normal mode
DigitalOut LED1_TestMode (LED1); 
DigitalOut LED2_NormalMode (LED2);
DigitalIn button(PB_2);
bool button_pressed = false;
Thread thread(osPriorityNormal, 512);

//Tickers
Ticker ti_2sec;
Ticker ti_30sec;
Ticker ti_1h;

bool isMonitoringTime2 = false;
bool isMonitoringTime30 = false;

//Sensors values
unsigned short light_value;
float x_value, y_value, z_value, temp_value, hum_value, moisture_value;
int dominant_value, clear, red, green, blue;


/*****************FUNCTIONS**************************/
void monitoringTime2(void){isMonitoringTime2 = true;}
void monitoringTime30(void){isMonitoringTime30 = true;}

void button_change(void){
	while(1){
		if(button){
			if(!button_pressed){
				button_pressed = true;
				printf("Changing mode...\n");
				mode =! mode;
				if(mode){
					LED1_TestMode = 1;
					LED2_NormalMode = 0;
				}else{
					LED1_TestMode = 1;
					LED2_NormalMode = 0;
				}
				printf("Mode: %s\n", mode ? "TEST MODE" : "NORMAL MODE");
			}
		}else{
				button_pressed = false;
		}
	}
}

void printValues(){
	printf("Current value of temperature: %f\n\r", temp_value);
	printf("Current value of humidity: %f\n\r", hum_value);
	printf("Current value of ambient light: %d\n\r", light_value);
	//printf("Current value of soil moisture: %f\n\r", moisture_value);
	printf("Colours: red = %i, green = %i, blue = %i, clear = %i\n\r", red, green, blue, clear );
	printf("Current dominant colour of the leaf: %i\n\r", dominant_value);
	//printf("Current global location (time): ", gps_value);
	printf("Current value of acceleration: x = %f, y = %f, z = %f\n\r", x_value, y_value, z_value);
}

void monitoringValues(){
	//LIGHT
	light_value = input.read_u16();
	
	//ACCELEROMETER
	x_value = abs(acc.getAccX());
	y_value = abs(acc.getAccY());
	z_value = abs(acc.getAccZ());
	
	//TEMPERATURE
	temp_value = tempHumSensor.get_temperature();
	
	//HUMIDITY
	hum_value = tempHumSensor.get_humidity();
	
	//RGB SENSOR
	rgb_sensor.enablePowerAndRGBC(); //esto a lo mejor hay que sacarlo del main
  rgb_sensor.setIntegrationTime( 100 ); //esto a lo mejor hay que sacarlo del main
	rgb_sensor.getAllColors( rgb_readings );
	red = rgb_readings[0];
	green = rgb_readings[1];
	blue = rgb_readings[2];
	clear = rgb_readings[3];
	// to print the dominant value:
	if( (red>>green) & (red>>blue) & (red>>clear)){ dominant_value = red;}
	else if( (green>>red) & (green>>blue) & (green>>clear)){ dominant_value = green;}
	else if( (blue>>red) & (blue>>green) & (blue>>clear)){ dominant_value = blue;}
	else {dominant_value = clear;}
	
	//SOIL MOISTURE
	
	//GPS
}
int main(){
	
	printf("Loading...");
	thread.start(button_change);
	ti_2sec.attach_us(monitoringTime2, 2000000);
	ti_30sec.attach_us(monitoringTime30, 5000000); //change to 30
	
	while(1){
		if(mode){
			if(isMonitoringTime2){
				printf("TEST MODE");
				isMonitoringTime2 = false;
				monitoringValues();
				printValues();
				//check connections
				//measures each 2 secs
				//printf each 2 secs
				//RGB LED coloured with the dominant colour
				//LED 1
			}
			
			
		
		}else{
			if(isMonitoringTime30){
				printf("NORMAL MODE");
				isMonitoringTime30 = false;
				monitoringValues();
				printValues();
				//measures each 30 secs
				//printf each 30 secs
				//calculates mean, max, min of T, H, soil, light, accel (x, y, z) each 1h
				//printf the mean, max, min each 1h
				//calculates dominant colour each 1h  (the colour that appear the most)
				//global location of the plant each 30 secs
				//limits the value of the measurements, if exceeds a led will notice (diff led for each measurement)
				//LED 2
			}
		}
	}
	
}
