/*
 * Traffic_Logic.c
 *
 *  Created on: 09 Dec 2025
 *      Author: augus
 */


#include "Traffic_Logic.h"
#include <stddef.h>


void TL_init(void){
	/*Traffic_Light_1_State = TL_GREEN;
	Traffic_Light_2_State = TL_GREEN;
	Pedestrian_Light_1_State = PL_RED;
	Pedestrian_Light_2_State = PL_RED;*/

}

void toggleFreq (void){
	HAL_Delay(500);
}

void togglePed1 (void){
	updateTrafficLights(0x0, 0x0, 0x20);
	toggleFreq();
	updateTrafficLights(0x0, 0x0, 0x0);
	toggleFreq();
}

void togglePed2 (void){
	updateTrafficLights(0x0, 0x20, 0x0);
	toggleFreq();
	updateTrafficLights(0x0, 0x0, 0x0);
	toggleFreq();
}

void walkingDelay(void){
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
}

void readTrafficState(void){

}

void writeTrafficState(void){

}
