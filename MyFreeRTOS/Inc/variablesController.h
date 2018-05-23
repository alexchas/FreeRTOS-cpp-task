/*******************************************************************************
*  FILENAME: variablesController.h
*
*  DESCRIPTION: �������� ������  ���������� ����������� ����������
*
*  Created on: 21 ��� 2018 �.
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
