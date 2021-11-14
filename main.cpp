/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "MMA8451Q.h"
#include "TCS3472_I2C.h"
#include "Si7021.h"
#include "RGBLed.h"
#include "MBed_Adafruit_GPS.h"

//using namespace std::chrono;

/***************************VARIABLES*****************************/
//Sensors
AnalogIn input(PA_4);
AnalogIn soil(PA_0);
MMA8451Q acc(PB_9, PB_8, 0x1d<<1);
TCS3472_I2C rgb_sensor (PB_9, PB_8);
int rgb_readings[4];
Si7021 tempHumSensor(PB_9, PB_8);
RGBLed rgbled(PH_0, PH_1, PB_13);
BufferedSerial *gps_Serial = new BufferedSerial(PA_9, PA_10,9600);
Adafruit_GPS myGPS(gps_Serial); 

//Modes
bool mode = false; //true = test mode, false = normal mode
DigitalOut LED1_TestMode (LED1); 
DigitalOut LED2_NormalMode (LED2);
InterruptIn button(PB_2, PullDown);

//Threads
Thread threadAnalog(osPriorityNormal, 512);
Thread threadI2C(osPriorityNormal, 512);
Thread threadGPS(osPriorityNormal, 2048);
int wait_time = 2000000;

//Tickers
Ticker ti_2sec;
Ticker ti_30sec;
Ticker ti_1h;

bool isMonitoringTime2 = false, isMonitoringTime30 = false, isMonitoringTime1 = false;

int redtimes, greentimes, bluetimes;

//Sensors values
unsigned short light_value;
float x_value, y_value, z_value, moisture_value;
int dominant_value, clear, red, green, blue;
float temp_value, hum_value;

//Mean, max, min
float mean_temp = 0, max_temp = -1000, min_temp = 1000;
float mean_hum = 0, max_hum = -1000, min_hum = 1000;
unsigned short mean_light = 0, max_light = 0, min_light = 1000;
float x_max = -1, x_min = 1, y_max = -1, y_min = 1, z_max = -1, z_min = 1;
float mean_soil = 0, max_soil = -1000, min_soil = 1000;
int samples = 4; //Each hour (3600) we measure each 30'', so we measure 3600/30 = 120 times per hour

/***************************FUNCTIONS*****************************/
void monitoringTime2(void){isMonitoringTime2 = true;}
void monitoringTime30(void){isMonitoringTime30 = true;}
void monitoringTime1(void){isMonitoringTime1 = true;}
void pressButton(void){mode = !mode;}

///////////THREADS//////////////
void analogValues(void){
	while (1){
		//LIGHT (%)
		light_value = (input.read_u16() * 100)/3100; //We have set 3100 as the 100% of the light since we have put the sensor under a lamp
		if(light_value>100){light_value = 100;} //In case it exceeds the max

		//SOIL MOISTURE
		moisture_value = soil * 100;
		/*moisture_value = (soil * 100)/0.8; //We have set 0.8 as the 100% of the soil moisture since we have done several measures
		if(moisture_value>100){moisture_value = 100;} //In case it exceeds the max*/
		wait_us(wait_time);
	}
}
void gpsRead(void){
	while(1){
		myGPS.read();
		if (myGPS.newNMEAreceived() ) {
			if (!myGPS.parse(myGPS.lastNMEA()) ) {
					continue;
			}
		}
	}
	wait_us(wait_time);
}
void i2cValues(void){
	while (1){
		//ACCELEROMETER
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
		red = rgb_readings[0];
		green = rgb_readings[1];
		blue = rgb_readings[2];
		clear = rgb_readings[3];
		
		wait_us(wait_time);
	}
}
////////////END THREADS////////////

void printValues(){
	printf("TEMPERATURE: %1.1f C\n\r", temp_value);
  printf("HUMIDITY: %1.1f %\n\r", hum_value);
	printf("LIGHT: %d %\n\r", light_value);
	printf("SOIL MOISTURE: %1.1f %\n\r", moisture_value);
	printf("ACCELEROMETER: x = %f, y = %f, z = %f\n\r", x_value, y_value, z_value);
	printf("COLOR SENSOR: clear = %i, red = %i, green = %i, blue = %i\n\r", clear, red, green, blue );
	printf("GPS VALUES: \n");
	printf("-- Time: %d:%d:%d\r\n", myGPS.hour + 1, myGPS.minute, myGPS.seconds);
	printf("-- Date: %d/%d/20%d\r\n", myGPS.day, myGPS.month, myGPS.year);
	printf("-- Quality: %i\r\n", (int) myGPS.fixquality);
	printf("-- Location convert: %5.2f %c, %5.2f %c\r\n", myGPS.latitude/100, myGPS.lat, myGPS.longitude/100, myGPS.lon);
	printf("-- Location sin convert: %5.2f %c, %5.2f %c\r\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
}

void dominantColour(){
	if((red>green) && (red>blue)){
		dominant_value = 0;
	}
	else if( (green>red) && (green>blue)){ 
		dominant_value = 1;
	}
	else{ 
		dominant_value = 2;
	}
}

void rgbLedTestMode(int color){
	switch(color){
		case 0:
			rgbled.setColor(RGBLed::RED);
			printf("RGB LED: RED\n");
			break;
		case 1:
			rgbled.setColor(RGBLed::GREEN);
			printf("RGB LED: GREEN\n");
			break;
		case 2:
			rgbled.setColor(RGBLed::BLUE);
			printf("RGB LED: BLUE\n");
			break;
	}
}

void gpsInit(){
	myGPS.begin(9600);
	myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
	myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	myGPS.sendCommand(PGCMD_ANTENNA);
	printf("Connection established at 9600 baud...\r\n");
	wait_us(1000000);
}

void checkLimits(){
	if(temp_value>25){ //red
		rgbled.setColor(RGBLed::RED);
		printf("CHECK LIMITS: Temperature value exceeds the limits\n");
	}
	if(hum_value>70){ //blue
		rgbled.setColor(RGBLed::BLUE);
		printf("CHECK LIMITS: Humidity value exceeds the limits\n");
	}
	if(light_value<10 || light_value>85){ //green
		rgbled.setColor(RGBLed::GREEN);
		printf("CHECK LIMITS: Light value exceeds the limits\n");
	}
	if(moisture_value>75){ //magenta (R,B)
		rgbled.setColor(RGBLed::MAGENTA);
		printf("CHECK LIMITS: Soil Moisture value exceeds the limits\n");
	}
	if(x_value>0.8){ //Yellow (G,R)
		rgbled.setColor(RGBLed::YELLOW);
		printf("CHECK LIMITS: X value exceeds the limit\n");
	}
	if(y_value>0.8){ //Yellow (G,R)
		rgbled.setColor(RGBLed::YELLOW);
		printf("CHECK LIMITS: Y value exceeds the limit\n");
	}
	if(z_value>0.8){//Yellow (G,R)
		rgbled.setColor(RGBLed::YELLOW);
		printf("CHECK LIMITS: Z value exceeds the limit\n");
	}
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
	mean_soil = 0;
	max_soil = -1000;
	min_soil = 1000;
}
void switchLed(){
	if(mode){
		LED1_TestMode = true;
		LED2_NormalMode = false;
	}else{
		LED1_TestMode = false;
		LED2_NormalMode = true;
	}
}
int main(){
	printf("Loading...\n");
	gpsInit();
	
	threadGPS.start(gpsRead);
	threadAnalog.start(analogValues);
	threadI2C.start(i2cValues);
	
	ti_2sec.attach_us(monitoringTime2, 2000000);
	ti_30sec.attach_us(monitoringTime30, 30000000); 
	ti_1h.attach_us(monitoringTime1, 20000000); //change to 1 hour
	
	button.mode(PullDown);
	button.fall(pressButton);
	
	while(1){
		switchLed();
			
		if(mode){
			if(isMonitoringTime2){
				wait_time = 2000000;
				printf("TEST MODE\n");
				isMonitoringTime2 = false;
				printValues();
				dominantColour();
				rgbLedTestMode(dominant_value);
				printf("\n\n\r");
			}			
		}else{
			if(isMonitoringTime30){
				wait_time = 30000000;
				printf("NORMAL MODE\n");
				isMonitoringTime30 = false;
				printValues();
				checkLimits();
				dominantColour();

				if(dominant_value == 0){ redtimes++;}
				if(dominant_value == 1){ greentimes++;}
				if(dominant_value == 2){ bluetimes++;}
				
				//to calculate mean, max and min of T, H, Light + max, min 3axis Acc
				mean_temp += temp_value;
				mean_hum += hum_value;
				mean_light += light_value;
				mean_soil += moisture_value;
				max_temp = compareValuesMax(temp_value, max_temp);
				min_temp = compareValuesMin(temp_value, min_temp);
				max_hum = compareValuesMax(hum_value, max_hum);
				min_hum = compareValuesMin(hum_value, min_hum);
				max_light = compareValuesMax(light_value, max_light);
				min_light = compareValuesMin(light_value, min_light);
				max_soil = compareValuesMax(mean_soil, max_soil);
				min_soil = compareValuesMax(mean_soil, min_soil);
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
					mean_soil = mean_soil/(float)samples;
					
					//Mean, max, min
					printf("TEMPERATURE PARAMETERS (C): MEAN %1.1f, MAX: %1.1f, MIN: %1.1f\n", mean_temp, max_temp, min_temp);
					printf("HUMIDITY PARAMETERS (%): MEAN %1.1f, MAX: %1.1f, MIN: %1.1f\n", mean_hum, max_hum, min_hum);
					printf("LIGHT PARAMETERS (%): MEAN %d, MAX: %d, MIN: %d\n", mean_light, max_light, min_light);
					printf("SOIL MOISTURE PARAMETERS (%): MEAN %1.1f, MAX: %1.1f, MIN: %1.1f\n", mean_soil, max_soil, min_soil);
					
					printf("ACCEL PARAMETERS: \n");
					printf(" -- X: X_MAX: %f, X_MIN: %f\n", x_max, x_min);
					printf(" -- Y: Y_MAX: %f, Y_MIN: %f\n", y_max, y_min);
					printf(" -- Z: Z_MAX: %f, Z_MIN: %f\n", z_max, z_min);

					//Dominant color as the one that appears the most in 1h
					printf("NUMBER OF TIMES OF: Red: %d, Green: %d, Blue: %d", redtimes, greentimes, bluetimes);					
					if(redtimes >= greentimes && redtimes >= bluetimes){
						printf("  -- Dominant color: RED\r\n\n");
						rgbled.setColor(RGBLed::RED);
					}else if(greentimes >= redtimes && greentimes >= bluetimes){
						printf("  -- Dominant color: GREEN\r\n\n");
						rgbled.setColor(RGBLed::GREEN);
					}else if(bluetimes >= redtimes && bluetimes >= greentimes){
						printf("  -- Dominant color: BLUE\r\n\n");
						rgbled.setColor(RGBLed::BLUE);
					}
					
					resetVariables(); //Because we want to update these values every hour
				}
				
				printf("\n\n\r");

			}
		}
	}
}
