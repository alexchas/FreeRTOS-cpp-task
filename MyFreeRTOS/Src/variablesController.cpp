/*******************************************************************************
*  FILENAME: variablesController.cpp
*
*  DESCRIPTION:
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
******************************************************************************/

#include "variablesController.h"
#include "frtosWrapper.h"       // обёртка oRTOS
#include "assert.h"

#define VARIABLESCONTROLLER_DELAY (tU32) (40/portTICK_PERIOD_MS)

/*******************************************************************************
* Function:  constructor
* Description: включает АЦП
* Threading usage and Assumptions:  none
******************************************************************************/
cVariablesController::cVariablesController(const cAdcController* pAdcController) {
  ASSERT(pAdcDirector != NULL);
  this->pVdda =  new cVdda(pAdcController);
}

/*******************************************************************************
* Function:  run
* Description: Задача  расчета напряжения
* Threading usage and Assumptions:
******************************************************************************/
void cVariablesController::run(void) {
  for(;;) {
    this->pVdda->calculate();
    oRTOS.taskDelay(VARIABLESCONTROLLER_DELAY);
  }
}




