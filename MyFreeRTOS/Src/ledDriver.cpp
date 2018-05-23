/*******************************************************************************
*  FILENAME: ledDriver.cpp
*
*  DESCRIPTION: функции работы со светодиодами платы
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
******************************************************************************/

#include <ledDriver.h>			//Определение класса и тип tLeds
#include <stm32f4xx.h> 			//
#include "stm32f407xx.h"		// Регистры STM32
#include "assert.h"     		// Для ASSERT
#include "macros.h"				// Для макросов работы с битами
#include "main.h"
#include "cmsis_os.h"

#define LED1_PIN        (tU32)GPIO_ODR_ODR_12
#define LED1_PORT		(tPort)GPIOD
#define LED2_PIN        (tU32)GPIO_ODR_ODR_13
#define LED2_PORT       (tPort)GPIOD
#define LED3_PIN        (tU32)GPIO_ODR_ODR_14
#define LED3_PORT       (tPort)GPIOD
#define LED4_PIN        (tU32)GPIO_ODR_ODR_15
#define LED4_PORT       (tPort)GPIOD

tPort cLedDriver::ledsPort[LEDS_NUMBER] = {LED1_PORT, LED2_PORT, LED3_PORT, LED4_PORT};
const tU32 cLedDriver::ledsPin[LEDS_NUMBER] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};

/*******************************************************************************
* Function:  constructor
* Description:
* Threading usage and Assumptions:  none
******************************************************************************/
cLedDriver::cLedDriver(void) {
};

/*******************************************************************************
* Function:  ledOn
* Description: Зажигает выбранный светодид
* Threading usage and Assumptions:  none
******************************************************************************/
void cLedDriver::ledOn(const tU8 led) const {
  ASSERT(led < LEDS_NUMBER); 
  LED05SET();
}

/*******************************************************************************
* Function:  ledOff
* Description: Гасит выбранный светодид
* Threading usage and Assumptions:  none
******************************************************************************/
void cLedDriver::ledOff(const tU8 led) const {
  ASSERT(led < LEDS_NUMBER); 
  LED05RESET();
}

/*******************************************************************************
* Function:  ledToggle
* Description: Меняет состояние выбранного светодиода
* Threading usage and Assumptions:  none
******************************************************************************/
void cLedDriver::ledToggle(const tU8 led) const {
  ASSERT(led < LEDS_NUMBER); 
}
