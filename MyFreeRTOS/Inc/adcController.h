/*******************************************************************************
*  FILENAME: adcController.h
*
*  DESCRIPTION: Описание класса управления АЦП, этот класс будет получать
*               и запоминать все необходимые измерения с каналов
*
*  Created on: 21 мая 2018 г.
*      Author: Alexander Chashkin
******************************************************************************/
#ifndef ADCCONTROLLER_H_
#define ADCCONTROLLER_H_
#include "adc.h"

#define MEASURE_NUMBER (tU8) 4

class cAdcController {
public:
	explicit cAdcController(void);
	void startConversion(void) const;
	__IO uint16_t channelValue[MEASURE_NUMBER];
private:
	cAdc *pAdc;
};



#endif /* ADCCONTROLLER_H_ */
