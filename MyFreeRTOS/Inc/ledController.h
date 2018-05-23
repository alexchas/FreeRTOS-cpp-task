/*******************************************************************************
*  FILENAME: ledController.h
*
*  DESCRIPTION: �������� ����� ���������� ������� ������ �����������
*
*  Created on: 21 ��� 2018 �.
*  Author: Alexander Chashkin
******************************************************************************/

//
#ifndef LEDCONTROLLER_H
#define LEDCONTROLLER_H

#include <ledDriver.h>    //��� cLedsDriver

#include "iactiveobject.h"  //��� iActiveObject

typedef enum
{
  LD_led1 = 0,
  LD_led2 = 1,
  LD_led3 = 2,
  LD_led4 = 3,
  LD_none = 4
} tLeds;

class cLedController: public iActiveObject
{
  public:
    explicit cLedController(void);
    virtual void run(void);
  private:

    cLedDriver* pLedDriver;
};

#endif //LEDSDIRECTOR_H
