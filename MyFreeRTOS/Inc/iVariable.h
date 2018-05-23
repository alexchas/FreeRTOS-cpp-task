/*******************************************************************************
*  FILENAME: ivariable.h
*
*  DESCRIPTION:   ��������� ��� ���� ����������
*
*  Created on: 21 ��� 2018 �.
*  Author: Alexander Chashkin
*
*****************************************************************************/

#ifndef IVARIABLE_H_
#define IVARIABLE_H_

#include "types.h"
#include "adcController.h"

class iVariable {
  public:
    explicit  iVariable(const cAdcController *pAdcCon);
    virtual tF32 calculate(void) = 0;
    tF32 getValue(void) const {return value;};
  protected:
    const cAdcController *pAdcController;
    tF32 value;
};

#endif /* IVARIABLE_H_ */
