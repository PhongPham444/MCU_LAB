/*
 * button1.c
 *
 *  Created on: Oct 26, 2024
 *      Author: Admin
 */
#include "button1.h"

static int KeyReg0 = NORMAL_STATE;
static int KeyReg1 = NORMAL_STATE;
static int KeyReg2 = NORMAL_STATE;

static int KeyReg3 = NORMAL_STATE;

static int TimerForKeyPress = 200;

void KeyInputHandler1(){
	mode++;
	if(mode >= 4){
		mode = 0;
	}
	else if (mode != 0){
		display_timer_flag = 1;
	}
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_RED_SIDE_GPIO_Port, LED_RED_SIDE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_YELLOW_SIDE_GPIO_Port, LED_YELLOW_SIDE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_SIDE_GPIO_Port, LED_GREEN_SIDE_Pin, GPIO_PIN_RESET);
}
void LongKeyInputHandler1(){
	KeyInputHandler1();
}

void getKeyInput1(){
	KeyReg0 = KeyReg1;
	KeyReg1 = KeyReg2;
	KeyReg2 = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	if((KeyReg0 == KeyReg1) && (KeyReg1 == KeyReg2)){
		if(KeyReg3 != KeyReg2){
			KeyReg3 = KeyReg2;
			if(KeyReg2 == PRESSED_STATE){
				KeyInputHandler1();
				TimerForKeyPress = 200;
			}
		}
		else {
			TimerForKeyPress--;
			if(TimerForKeyPress == 0){
				if(KeyReg2 == PRESSED_STATE){
					LongKeyInputHandler1();
				}
				TimerForKeyPress = 200;
			}
		}
	}
}
