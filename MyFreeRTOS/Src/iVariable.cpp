/*******************************************************************************
*  FILENAME: iVariable.cpp
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
*******************************************************************************/
#include "iVariable.h"
#include "assert.h"

/*******************************************************************************
* Function:  constructor
* Description:
******************************************************************************/
iVariable::iVariable(const cAdcController *pAdcCon) {
  ASSERT(pAdcController != NULL);
  this->pAdcController = pAdcCon;
  this->value = (tF32)0.0;
}





