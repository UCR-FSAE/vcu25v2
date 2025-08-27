/*
 * pedals.c
 *
 *  Created on: Aug 24, 2025
 *      Author: Justin Im
 */

#include "pedals.h"

// variables
uint32_t appsRaw1 = 0;
uint32_t appsRaw1Max = 0;
uint32_t appsRaw1Min = 4096;

uint32_t appsRaw2 = 0;
uint32_t appsRaw2Max = 0;
uint32_t appsRaw2Min = 4096;

extern volatile float global_accel_position;
float appsConverted1 = 0.0f;
float appsConverted2 = 0.0f;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

// temporary variables used in the calibration phase
static int v1;
static int v2;

// filter values
static float filteredAppsRaw1 = 0.0f;
static float filteredAppsRaw2 = 0.0f;
const float alpha = 0.2f;

void calibratePedals(void) {
	// reset
    appsRaw1Max = 0;
    appsRaw1Min = 4096;
    appsRaw2Max = 0;
    appsRaw2Min = 4096;

    // Calibration
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);

	// begin max calibration
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);

	uint32_t t0 = HAL_GetTick(); // ms since power-up
	// for 3000 ms (3s) window
	while (HAL_GetTick() - t0 < 3000) {
		HAL_ADC_Stop(&hadc1);
		if (HAL_ADC_Start(&hadc1) != HAL_OK) {
			Error_Handler();
		}

		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			v1 = HAL_ADC_GetValue(&hadc1); v2 = HAL_ADC_GetValue(&hadc1); // get values for adc channels 5 and 6 in succession
			if (v1 > appsRaw1Max) appsRaw1Max = v1;
			if (v2 > appsRaw2Max) appsRaw2Max = v2;
		}
		else {
			v1 = 0; v2 = 0; HAL_ADC_Stop(&hadc1);
		}
		HAL_Delay(10);
	}

	// send begin min calibraion
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);

	t0 = HAL_GetTick();
	while (HAL_GetTick() - t0 < 3000) {
		HAL_ADC_Stop(&hadc1);

		if (HAL_ADC_Start(&hadc1) != HAL_OK) { Error_Handler(); }

		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {

			v1 = HAL_ADC_GetValue(&hadc1); v2 = HAL_ADC_GetValue(&hadc1); // get values for adc channels 5 and 6 in succession
			if (v1 < appsRaw1Min) appsRaw1Min = v1;
			if (v2 < appsRaw2Min) appsRaw2Min = v2;
		}
		else {
			v1 = 0; v2 = 0; HAL_ADC_Stop(&hadc1);
		}
		HAL_Delay(10);
	}

    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);

    // failsafe
    if (appsRaw1Max <= appsRaw1Min || appsRaw2Max <= appsRaw2Min) {
        appsRaw1Min = 0;
        appsRaw1Max = 4095;
        appsRaw2Min = 0;
        appsRaw2Max = 4095;
    }

    // begin BSE calibration
	// begin max calibration
//    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
//
//	t0 = HAL_GetTick(); // ms since power-up
//	// for 3000 ms (3s) window
//	while (HAL_GetTick() - t0 < 3000) {
//		HAL_ADC_Stop(&hadc2);
//		if (HAL_ADC_Start(&hadc2) != HAL_OK) {
//			Error_Handler();
//		}
//
//		if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
//			v1 = HAL_ADC_GetValue(&hadc1); // only one channel being used here
//			if (v1 > bseRawMax) { bseRawMax = v1; }
//		}
//		else {
//			v1 = 0; HAL_ADC_Stop(&hadc2);
//		}
//		HAL_Delay(10);
//	}
//
//	// send begin min calibraion
//    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
//
//	t0 = HAL_GetTick();
//	while (HAL_GetTick() - t0 < 3000) {
//		HAL_ADC_Stop(&hadc2);
//
//		if (HAL_ADC_Start(&hadc2) != HAL_OK) { Error_Handler(); }
//
//		if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
//
//			v1 = HAL_ADC_GetValue(&hadc2); // only one channel being used here
//			if (v1 < bseRawMin) { bseRawMin = v1; }
//		}
//		else {
//			v1 = 0; HAL_ADC_Stop(&hadc1);
//		}
//		HAL_Delay(10);
//	}
//	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
}

void pedalCapture(void) {
	// captures BSE and APPS inputs

	// APPS capture
	if (HAL_ADC_Start(&hadc1) != HAL_OK) { Error_Handler(); }

	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		appsRaw1 = HAL_ADC_GetValue(&hadc1);
		appsRaw2 = HAL_ADC_GetValue(&hadc1);
	}
	else {
		appsRaw1 = 0;
		appsRaw2 = 0;
		HAL_ADC_Stop(&hadc1);
	}

    if ((appsRaw1Max - appsRaw1Min) <= 0 || (appsRaw2Max - appsRaw2Min) <= 0) {
        global_accel_position = 0.0f;
        return;
    }

//    filteredAppsRaw1 = alpha * appsRaw1 + (1.0f - alpha) * filteredAppsRaw1;
//	filteredAppsRaw2 = alpha * appsRaw2 + (1.0f - alpha) * filteredAppsRaw2;

//    appsConverted1 = (float)(filteredAppsRaw1 - appsRaw1Min) / (float)(appsRaw1Max - appsRaw1Min);
//    appsConverted2 = (float)(filteredAppsRaw2 - appsRaw2Min) / (float)(appsRaw2Max - appsRaw2Min);

	appsConverted1 = (float)(appsRaw1 - appsRaw1Min) / (float)(appsRaw1Max - appsRaw1Min);
	appsConverted2 = (float)(appsRaw2 - appsRaw2Min) / (float)(appsRaw2Max - appsRaw2Min);

    // Clamp to valid range (safety check)
    if (appsConverted1 < 0.0f) appsConverted1 = 0.0f;
    if (appsConverted1 > 1.0f) appsConverted1 = 1.0f;
    if (appsConverted2 < 0.0f) appsConverted2 = 0.0f;
    if (appsConverted2 > 1.0f) appsConverted2 = 1.0f;

    global_accel_position = (appsConverted1 + appsConverted2) / 2;
    if (global_accel_position < 0) {
    	global_accel_position = 0.0f;
    }
    else if (global_accel_position > 1.0) {
    	global_accel_position = 1.0f;
    }

	// BSE capture
//		if (HAL_ADC_Start(&hadc2) != HAL_OK) { Error_Handler(); }
//
//		if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) { bseRaw = HAL_ADC_GetValue(&hadc2); }
//		else {
//			bseRaw = 0;
//			HAL_ADC_Stop(&hadc2);
//		}
}
