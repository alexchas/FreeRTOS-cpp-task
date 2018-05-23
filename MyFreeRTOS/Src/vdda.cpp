#include "main.h"
#include "vdda.h"

#define VDDA_CAL_ADDR       (tU32) 0x1FF80076      //����� ���� VDDA ��� 3.0 �
#define VDDA_FILTER_CONST   (tF32) 10.0
#define VOLTS_3_0           (tF32) 3.0


// constructor
cVdda::cVdda(const cAdcController *pAdcCon) : iVariable(pAdcCon), oFilter(VDDA_FILTER_CONST)  {
}

// ������ ���������� Vdda
tF32 cVdda::calculate(void) {
  tF32 vdda = (tF32)0.0;  										 //�������� ���� vdda
  tF32 vddaCal = (tF32)(*((tU32 *)(VDDA_CAL_ADDR)) >> 16);       //������������ ����������

  vdda = (tF32)this->pAdcController->channelValue[VDDA_CHANNEL];
  vdda = VOLTS_3_0 * vdda / vddaCal;                             //�������

  this->oFilter.updateFilteredValue(vdda);
  this->value = this->oFilter.getFilteredValue();

  return  this->value;
}




