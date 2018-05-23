/*******************************************************************************
*  FILENAME: adc.h
*
*  DESCRIPTION: Описания класса работы с АЦП
*  Created on: 21 мая 2018 г.
*      Author: Alexander Chashkin
*******************************************************************************/

#ifndef ADC_H_
#define ADC_H_

#include "types.h"

#define VDDA_CHANNEL 	 0
#define POTENTIOMETER1_CHANNEL 1
#define POTENTIOMETER2_CHANNEL 2
#define POTENTIOMETER3_CHANNEL 3

class cAdc {
	public:
	explicit cAdc(const tU32 memoryBaseAddr, const tU8 measureCount);
	tBoolean switchOn(void) const;
	tBoolean startConversion(void) const;
	tBoolean isConversionReady(void) const;
	tF32 getValue(void) const;

	private:
	void initDma(const tU32 memoryBaseAddr, const tU8 measureCount) const;
};

#endif /* ADC_H_ */
