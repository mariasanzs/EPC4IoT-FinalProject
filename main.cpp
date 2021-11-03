/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "MMA8451Q.h"
#include "TCS3472_I2C.h"
#include "Si7021.h"
#include "RGBLed.h"

/*****************VARIABLES**************************/
//Sensors
AnalogIn input(PA_4);
MMA8451Q acc(PB_9, PB_8, 0x1d<<1);
TCS3472_I2C rgb_sensor (PB_9, PB_8);
int rgb_readings[4];
I2C tempSensor(PB_9, PB_8);
DigitalOut SENS_EN(D13);
Si7021 tempHumSensor(PB_9, PB_8);
RGBLed rgbled(PH_0, PH_1, PB_13);

//Modes
bool mode = true; //true = test mode, false = normal mode
DigitalOut LED1_TestMode (LED1); 
DigitalOut LED2_NormalMode (LED2);
DigitalIn button(PB_2);
bool button_pressed = false;
Thread threadButton(osPriorityNormal, 512);
Thread threadGPS(osPriorityNormal, 512);

//Tickers
Ticker ti_2sec;
Ticker ti_30sec;
Ticker ti_1h;

bool isMonitoringTime2 = false;
bool isMonitoringTime30 = false;
bool isMonitoringTime1 = false;

int redtimes, greentimes, bluetimes;

//Sensors values
unsigned short light_value;
float x_value, y_value, z_value, moisture_value;
int dominant_value, clear, red, green, blue;
float temp_value;
float hum_value;

//Mean, max, min
float mean_temp = 0, max_temp = -1000, min_temp = 1000;
float mean_hum = 0, max_hum = -1000, min_hum = 1000;
unsigned short mean_light = 0, max_light = 0, min_light = 1000;
float x_max = -1, x_min = 1, y_max = -1, y_min = 1, z_max = -1, z_min = 1;
int samples = 4; //Each hour (3600) we measure each 30'', so we measure 3600/30 = 120 times per hour

/*****************FUNCTIONS**************************/
void monitoringTime2(void){isMonitoringTime2 = true;}
void monitoringTime30(void){isMonitoringTime30 = true;}
void monitoringTime1(void){isMonitoringTime1 = true;}

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
					LED1_TestMode = 0;
					LED2_NormalMode = 1;
				}
				printf("Mode: %s\n", mode ? "TEST MODE" : "NORMAL MODE");
			}
		}else{
				button_pressed = false;
		}
	}
}

void printValues(){
	printf("TEMPERATURE: %1.1f C\n\r", temp_value);
  printf("HUMIDITY: %1.1f %\n\r", hum_value);
	printf("LIGHT: %d %\n\r", light_value);
	//printf("SOIL MOISTURE: %f\n\r", moisture_value);
	//printf("GPS (time): ", gps_value);
	printf("ACCELEROMETER: x = %f, y = %f, z = %f\n\r", x_value, y_value, z_value);
	printf("COLOR SENSOR: clear = %i, red = %i, green = %i, blue = %i\n\r", clear, red, green, blue );
}

void monitoringValues(){
	//LIGHT (%)
	light_value = input.read_u16();
	short a = (light_value * 100)/656; //We have set 656 as the 100% of the light since we have put the sensor under a lamp
	light_value = a;
	if(light_value>100){light_value = 100;} //In case it exceeds the max
	
	//ACCELEROMETER --------------------------------HE QUITADO EL ABS
	x_value = acc.getAccX();
	y_value = acc.getAccY();
	z_value = acc.getAccZ();
	
	//TEMPERATURE
	tempHumSensor.get_data();
	temp_value = tempHumSensor.get_temperature();
	temp_value = temp_value/1000;
	
	//HUMIDITY (%)
	hum_value = tempHumSensor.get_humidity();
	hum_value = hum_value/1000;
	
	//RGB SENSOR
	rgb_sensor.enablePowerAndRGBC(); 
  rgb_sensor.setIntegrationTime( 100 ); 
	rgb_sensor.getAllColors( rgb_readings );
	red = rgb_readings[1];
	green = rgb_readings[2];
	blue = rgb_readings[3];
	clear = rgb_readings[0];
	
	//SOIL MOISTURE
	
	//GPS
	
}

void dominantValue(){
	// to print the dominant value and RGBLED:
	if((red>green) && (red>blue)){
		dominant_value = red;
		rgbled.setColor(RGBLed::RED);
		printf("DOMINANT: RED\n");
	}
	else if( (green>red) && (green>blue)){ 
		dominant_value = green;
		rgbled.setColor(RGBLed::GREEN); 
		printf("DOMINANT: GREEN\n");
	}
	else{ 
		dominant_value = blue;
		rgbled.setColor(RGBLed::BLUE);
		printf("DOMINANT: BLUE\n");
	}
}
void gpsValues(){
	
}
float compareValuesMax(float current_value, float max){
	if(current_value > max) {return current_value;}
	else{ return max;}
}
float compareValuesMin(float current_value, float min){
	if(current_value < min) {return current_value;}
	else{ return min;}
}

void resetVariables(){
	printf("RESET VARIABLES\n");
	redtimes = 0;
	greentimes = 0;
	bluetimes = 0;
	mean_temp = 0;
	max_temp = -1000;
	min_temp = 1000;
	mean_hum = 0;
	max_hum = -1000;
	min_hum = 1000;
	mean_light = 0;
	max_light = 0; 
	min_light = 1000;
	x_max = -1;
	x_min = 1;
	y_max = -1;
	y_min = 1;
	z_max = -1;
	z_min = 1;
}
int main(){
	printf("Loading...");
	threadButton.start(button_change);
	ti_2sec.attach_us(monitoringTime2, 2000000);
	ti_30sec.attach_us(monitoringTime30, 5000000); //change to 30
	ti_1h.attach_us(monitoringTime1, 20000000); //change to 1 hour
		
	while(1){
		if(mode){
			if(isMonitoringTime2){
				printf("TEST MODE\n");
				isMonitoringTime2 = false;
				monitoringValues();
				printValues();
				dominantValue();
				//soilmoisture
				//gpsValues();
				printf("\n\n\r");
				//check connections
				//measures each 2 secs
				//printf each 2 secs
				//RGB LED coloured with the dominant colour
				//LED 1
			}			
		
		}else{
			if(isMonitoringTime30){
				printf("NORMAL MODE\n");
				isMonitoringTime30 = false;
				monitoringValues();
				printValues();
				dominantValue();
				if(dominant_value == red){ redtimes++;}
				if(dominant_value == green){ greentimes++;}
				if(dominant_value == blue){ bluetimes++;}
				
				//to calculate mean, max and min of T, H, Light + max, min 3axis Acc
				mean_temp += temp_value;
				mean_hum += hum_value;
				mean_light += light_value;
				max_temp = compareValuesMax(temp_value, max_temp);
				min_temp = compareValuesMin(temp_value, min_temp);
				max_hum = compareValuesMax(hum_value, max_hum);
				min_hum = compareValuesMin(hum_value, min_hum);
				max_light = compareValuesMax(light_value, max_light);
				min_light = compareValuesMin(light_value, min_light);
				x_max = compareValuesMax(x_value, x_max);
				x_min = compareValuesMin(x_value, x_min);
				y_max = compareValuesMax(y_value, y_max);
				y_min = compareValuesMin(y_value, y_min);
				z_max = compareValuesMax(z_value, z_max);
				z_min = compareValuesMin(z_value, z_min);
				
				printf("\n\n\r");
				
				if(isMonitoringTime1){
					isMonitoringTime1 = false;
					
					mean_temp = mean_temp/(float)samples;
					mean_hum = mean_hum/(float)samples;
					mean_light = mean_light/samples;
					
					//Mean, max, min
					printf("TEMPERATURE PARAMETERS: MEAN %1.1f, MAX: %1.1f, MIN: %1.1f\n", mean_temp, max_temp, min_temp);
					printf("HUMIDITY PARAMETERS (%): MEAN %1.1f, MAX: %1.1f, MIN: %1.1f\n", mean_hum, max_hum, min_hum);
					printf("LIGHT PARAMETERS (%): MEAN %d, MAX: %d, MIN: %d\n", mean_light, max_light, min_light);
					
					printf("ACCEL PARAMETERS: X_MAX: %f, X_MIN: %f, Y_MAX: %f, Y_MIN: %f, Z_MAX: %f, Z_MIN: %f\n", x_max, x_min, y_max, y_min, z_max, z_min);
					
					//Dominant color as the one that appears the most in 1h
					printf("NUMBER OF TIMES OF: Red: %d, Green: %d, Blue: %d", redtimes, greentimes, bluetimes);					
					if(redtimes >= greentimes && redtimes >= bluetimes){
						printf("  -- Dominant color: RED\r\n\n");
					}else if(greentimes >= redtimes && greentimes >= bluetimes){
						printf("  -- Dominant color: GREEN\r\n\n");
					}else if(bluetimes >= redtimes && bluetimes >= greentimes){
						printf("  -- Dominant color: BLUE\r\n\n");
					}
					
					resetVariables(); //Because we want to update these values every hour
				}
				
				
				//soilmoisture
				//global location of the plant each 30 secs
				//limits the value of the measurements, if exceeds a led will notice (diff led for each measurement)
				//gpsValues();
				printf("\n\n\r");
				//calculates mean, max, min of T, H, soil, light
				//calculates max, min accel (x, y, z) each 1h
				//printf the mean, max, min each 1h
				//measures each 30 secs
				//printf each 30 secs
				//calculates dominant colour each 1h  (the colour that appear the most)
				//LED 2
			}
		}
	}
}
