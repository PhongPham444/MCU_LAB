/*
 * fsm.c
 *
 *  Created on: Oct 26, 2024
 *      Author: Admin
 */
#include "fsm.h"

int temp[3] = {5000, 2000, 3000};
int mode = 0;
int curState = 0;
int curSideState = 1;

void display7SEG(int num){
	switch (num) {
		case 0:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, RESET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, RESET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, RESET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, SET);
			break;
		case 1:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, SET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, RESET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, SET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, SET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, SET);
			break;
		case 2:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, RESET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, SET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, RESET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, SET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, RESET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, RESET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, SET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, SET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, RESET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, SET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, RESET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, SET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, RESET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, RESET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, SET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, RESET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, RESET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, RESET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, SET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, SET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, SET);
			break;
		case 8:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, RESET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, RESET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, RESET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, RESET);
			break;
		case 9:
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, RESET);
			HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, RESET);
			HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, RESET);
			HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
			HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, RESET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, RESET);
			break;
		default:
			break;
	}
}
void updateClockBuffer(){
	int temp_main = timer_counter /100;
	int temp_side = timer_counter_side /100;
	int fst_main_digit = temp_main/10;
	int snd_main_digit = temp_main % 10;
	int fst_side_digit = temp_side /10;
	int snd_side_digit = temp_side % 10;

	HAL_GPIO_WritePin(GPIOB, EN0_Pin,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, EN1_Pin,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, EN2_Pin,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, EN3_Pin,  GPIO_PIN_SET);
	switch(idx){
	case 0:
		HAL_GPIO_WritePin(GPIOB, EN0_Pin,  GPIO_PIN_RESET);
		display7SEG(fst_main_digit);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, EN1_Pin,  GPIO_PIN_RESET);
		display7SEG(snd_main_digit);
		break;

	case 2:
		HAL_GPIO_WritePin(GPIOB, EN2_Pin,  GPIO_PIN_RESET);
		display7SEG(fst_side_digit);
		break;

	case 3:
		HAL_GPIO_WritePin(GPIOB, EN3_Pin,  GPIO_PIN_RESET);
		display7SEG(snd_side_digit);
		break;
	}
}
void displayContent() {
    int time_fst_digit = temp[mode-1] / 10000;
    int time_snd_digit = (temp[mode-1] % 10000) / 1000;

    HAL_GPIO_WritePin(GPIOB, EN0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, EN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, EN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, EN3_Pin, GPIO_PIN_SET);

    switch (idx) {
        case 0:
            HAL_GPIO_WritePin(GPIOB, EN0_Pin, GPIO_PIN_RESET);
            display7SEG(0);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOB, EN1_Pin, GPIO_PIN_RESET);
            display7SEG(mode);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOB, EN2_Pin, GPIO_PIN_RESET);
            display7SEG(time_fst_digit);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOB, EN3_Pin, GPIO_PIN_RESET);
            display7SEG(time_snd_digit);
            break;
    }
}
void fsm_run(){
	switch(mode){
	case 0:{
		if (display_timer_flag) {
			display_timer_flag = 0;
			setDisplayTimer(250);
			updateClockBuffer();
		}
		if (timer_flag) {
			timer_flag = 0;

			switch (curState) {
			case 0:
				curState = 1;
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
				setTimer(greenTime);
				break;

			case 1:
				curState = 2;
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
				setTimer(yellowTime);
				break;

			case 2:
				curState = 0;
				HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
				setTimer(redTime);
				break;
			}
		}

		if (timer_flag_side) {
			timer_flag_side = 0;

			switch (curSideState) {
			case 0:
				curSideState = 1;
				HAL_GPIO_WritePin(LED_RED_SIDE_GPIO_Port, LED_RED_SIDE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_YELLOW_SIDE_GPIO_Port, LED_YELLOW_SIDE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_GREEN_SIDE_GPIO_Port, LED_GREEN_SIDE_Pin, GPIO_PIN_SET);
				setTimerSide(greenTime);
				break;

			case 1:
				curSideState = 2;
				HAL_GPIO_WritePin(LED_RED_SIDE_GPIO_Port, LED_RED_SIDE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_GREEN_SIDE_GPIO_Port, LED_GREEN_SIDE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_YELLOW_SIDE_GPIO_Port, LED_YELLOW_SIDE_Pin, GPIO_PIN_SET);
				setTimerSide(yellowTime);
				break;

			case 2:
				curSideState = 0;
				HAL_GPIO_WritePin(LED_GREEN_SIDE_GPIO_Port, LED_GREEN_SIDE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_YELLOW_SIDE_GPIO_Port, LED_YELLOW_SIDE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_RED_SIDE_GPIO_Port, LED_RED_SIDE_Pin, GPIO_PIN_SET);
				setTimerSide(redTime);
				break;
			}
		}
		break;
	}
	case 1:{
		if (display_timer_flag) {
			display_timer_flag = 0;
			setDisplayTimer(250);
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			HAL_GPIO_TogglePin(LED_RED_SIDE_GPIO_Port, LED_RED_SIDE_Pin);
			displayContent();
		}

		break;
	}
	case 2:{
		if (display_timer_flag) {
			display_timer_flag = 0;
			setDisplayTimer(250);
			HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
			HAL_GPIO_TogglePin(LED_YELLOW_SIDE_GPIO_Port, LED_YELLOW_SIDE_Pin);
			displayContent();
		}
		break;
	}
	case 3:{
		if (display_timer_flag) {
			display_timer_flag = 0;
			setDisplayTimer(250);
			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			HAL_GPIO_TogglePin(LED_GREEN_SIDE_GPIO_Port, LED_GREEN_SIDE_Pin);
			displayContent();
		}
		break;
	}
	default:
		break;
	}
}
void init() {
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RED_SIDE_GPIO_Port, LED_RED_SIDE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_YELLOW_SIDE_GPIO_Port, LED_YELLOW_SIDE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_SIDE_GPIO_Port, LED_GREEN_SIDE_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    setTimer(redTime);

    HAL_GPIO_WritePin(LED_GREEN_SIDE_GPIO_Port, LED_GREEN_SIDE_Pin, GPIO_PIN_SET);
    setTimerSide(greenTime);

    setDisplayTimer(250);

    curState = 0;
    curSideState = 1;

    idx = 0;
}

