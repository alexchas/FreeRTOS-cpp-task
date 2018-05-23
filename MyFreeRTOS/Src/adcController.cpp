/*******************************************************************************
*  FILENAME: adcController.cpp
*
*  DESCRIPTION: контроллер работы с АЦП
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
*******************************************************************************/

#include "adcController.h"

/*******************************************************************************
* Function:  constructor
* Description: создает экземпляр АЦП, и передает ему адреса в RAM, куда АЦП с
*              помощью DMA будет скалдывать результат преобразований.
*
* Threading usage and Assumptions:  none
******************************************************************************/
cAdcController::cAdcController(void){
  this->pAdc = new cAdc((tU32)&channelValue[0], MEASURE_NUMBER);
  this->pAdc->switchOn();
}

/*******************************************************************************
* Function:  startConversion
* Description: Запускаем АЦП на измерение, все данные складываются по DMA в массив channelValue
* Threading usage and Assumptions:
******************************************************************************/
void cAdcController::startConversion(void) const {
  this->pAdc->startConversion();
}


