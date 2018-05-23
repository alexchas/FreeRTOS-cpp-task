/*******************************************************************************
*  FILENAME: adc.cpp
*
*  DESCRIPTION: ������� ��� ������ � ���������-�������� ����������������
*
*  Created on: 21 ��� 2018 �.
*  Author: Alexander Chashkin
******************************************************************************/

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f407xx.h>
#include "adc.h"
#include "assert.h"
#include "macros.h"

/***************************************************************************************************************
*  constructor: �������������� ����� DMA ������� � RAM, ���� ���������� ������ ��������� � ����������� ���������
****************************************************************************************************************/
cAdc::cAdc(const tU32 memoryBaseAddr, const tU8 measureCount) {
  ASSERT(measureCount != 0);
}

/***************************************************************************************************************
*  �������� ���
****************************************************************************************************************/
tBoolean cAdc::switchOn(void) const {
  tBoolean  result = FALSE;
  SETBIT(ADC1->CR2, ADC_CR2_ADON);    //�������� ���
  result =  tBoolean(CHECK_BIT_SET(ADC1->SR, ADC_SR_STRT_Msk));
  return result;
}

/***************************************************************************************************************
* ������ ��������������
****************************************************************************************************************/
tBoolean cAdc::startConversion(void) const {
  tBoolean  result = FALSE;
  SETBIT(ADC1->CR2, ADC_CR2_SWSTART);  // ��������� �������������� ���
  result = tBoolean(CHECK_BIT_SET(ADC1->SR, ADC_SR_STRT));
  return result;
}

/***************************************************************************************************************
* ���������� ���������� ��������������
****************************************************************************************************************/
tF32 cAdc::getValue(void) const {
  tF32  result = (tF32) ADC1->DR;
  return result;
}

/***************************************************************************************************************
* ��������, ������ �� ��������������
****************************************************************************************************************/
tBoolean cAdc::isConversionReady(void) const {
  tBoolean result = tBoolean(CHECK_BIT_SET(ADC1->SR, ADC_SR_EOC));
  return result;
}





