/*******************************************************************************
*  FILENAME: variablesController.h
*
*  DESCRIPTION: Описания класса  управления переменными устройства
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
******************************************************************************/

#ifndef VARIABLESCONTROLLER_H_
#define VARIABLESCONTROLLER_H_

#include "iActiveObject.h"
#include "vdda.h"

class cVariablesController : public iActiveObject
{
  public:
    explicit cVariablesController(const cAdcController* pAdcController);
    virtual void run(void);
    cVdda *pVdda;
};

#endif /* VARIABLESCONTROLLER_H_ */
