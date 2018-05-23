/*******************************************************************************
*  FILENAME: adcController.h
*
*  DESCRIPTION: �������� ������ ���������� ���, ���� ����� ����� ��������
*               � ���������� ��� ����������� ��������� � �������
*
*  Created on: 21 ��� 2018 �.
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
