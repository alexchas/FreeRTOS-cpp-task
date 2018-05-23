/*******************************************************************************
*  FILENAME: adcController.cpp
*
*  DESCRIPTION: ���������� ������ � ���
*
*  Created on: 21 ��� 2018 �.
*  Author: Alexander Chashkin
*******************************************************************************/

#include "adcController.h"

/*******************************************************************************
* Function:  constructor
* Description: ������� ��������� ���, � �������� ��� ������ � RAM, ���� ��� �
*              ������� DMA ����� ���������� ��������� ��������������.
*
* Threading usage and Assumptions:  none
******************************************************************************/
cAdcController::cAdcController(void){
  this->pAdc = new cAdc((tU32)&channelValue[0], MEASURE_NUMBER);
  this->pAdc->switchOn();
}

/*******************************************************************************
* Function:  startConversion
* Description: ��������� ��� �� ���������, ��� ������ ������������ �� DMA � ������ channelValue
* Threading usage and Assumptions:
******************************************************************************/
void cAdcController::startConversion(void) const {
  this->pAdc->startConversion();
}


