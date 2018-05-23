/*******************************************************************************
*  FILENAME: ledController.cpp
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
*
*******************************************************************************/

#include "ledController.h"      // Определение класса
#include "frtosWrapper.h"
#include "types.h"             // Для типов проекта
#include <limits.h>            // Для ULONG_MAX

#define LED_DELAY (tU32) (500/portTICK_PERIOD_MS)

// Создает объект класса cLedDriver
/*******************************************************************************
* Function:  constructor
* Description: Создает объект класса cLedsDriver
******************************************************************************/
cLedController::cLedController(void) {
  this->pLedDriver =  new cLedDriver();
}

/*******************************************************************************
* Function:  runTask
* Description: Задача управления ледами.
******************************************************************************/
void cLedController::run(void) {
  tBoolean status = FALSE;
  while(1) {
	  oRTOS.taskDelay(LED_DELAY);
  }
}





