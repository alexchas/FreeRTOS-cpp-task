#include "main.h"
#include "vdda.h"

#define VDDA_CAL_ADDR       (tU32) 0x1FF80076      //Адрес кода VDDA при 3.0 В
#define VDDA_FILTER_CONST   (tF32) 10.0
#define VOLTS_3_0           (tF32) 3.0


// constructor
cVdda::cVdda(const cAdcController *pAdcCon) : iVariable(pAdcCon), oFilter(VDDA_FILTER_CONST)  {
}

// Расчет напряжения Vdda
tF32 cVdda::calculate(void) {
  tF32 vdda = (tF32)0.0;  										 //значение кода vdda
  tF32 vddaCal = (tF32)(*((tU32 *)(VDDA_CAL_ADDR)) >> 16);       //коэффициенты калибровки

  vdda = (tF32)this->pAdcController->channelValue[VDDA_CHANNEL];
  vdda = VOLTS_3_0 * vdda / vddaCal;                             //формула

  this->oFilter.updateFilteredValue(vdda);
  this->value = this->oFilter.getFilteredValue();

  return  this->value;
}




