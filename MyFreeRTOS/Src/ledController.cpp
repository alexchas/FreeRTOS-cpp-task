/*******************************************************************************
*  FILENAME: ledController.cpp
*
*  Created on: 21 ��� 2018 �.
*  Author: Alexander Chashkin
*
*******************************************************************************/

#include "ledController.h"      // ����������� ������
#include "frtosWrapper.h"
#include "types.h"             // ��� ����� �������
#include <limits.h>            // ��� ULONG_MAX

#define LED_DELAY (tU32) (500/portTICK_PERIOD_MS)

// ������� ������ ������ cLedDriver
/*******************************************************************************
* Function:  constructor
* Description: ������� ������ ������ cLedsDriver
******************************************************************************/
cLedController::cLedController(void) {
  this->pLedDriver =  new cLedDriver();
}

/*******************************************************************************
* Function:  runTask
* Description: ������ ���������� ������.
******************************************************************************/
void cLedController::run(void) {
  tBoolean status = FALSE;
  while(1) {
	  oRTOS.taskDelay(LED_DELAY);
  }
}





