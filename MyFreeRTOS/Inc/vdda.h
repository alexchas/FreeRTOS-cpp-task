/*******************************************************************************
*  FILENAME: vdda.h
*
*  DESCRIPTION: Описания класса измерения напряжения
*
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
*
******************************************************************************/

#ifndef VDDA_H_
#define VDDA_H_

#include "types.h"
#include "ivariable.h"
#include "filter.h"
#include "adcController.h"

class cVdda : public iVariable {

public:
	explicit cVdda(const cAdcController *pAdcController);
	virtual tF32 calculate(void);
private:
	cFilter oFilter;
};


#endif /* VDDA_H_ */
