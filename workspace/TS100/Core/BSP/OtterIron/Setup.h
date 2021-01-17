/*
 * Setup.h
 *
 *  Created on: 29Aug.,2017
 *      Author: Ben V. Brown
 */

#ifndef SETUP_H_
#define SETUP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

extern IWDG_HandleTypeDef hiwdg;

void Setup_HAL();

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim); //Since the hal header file does not define this one

#ifdef __cplusplus
}
#endif

#endif /* SETUP_H_ */
