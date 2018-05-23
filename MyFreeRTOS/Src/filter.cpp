/*******************************************************************************
*  FILENAME: filter.cpp
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
*
*******************************************************************************/

#include "assert.h"
#include "types.h"
#include "filter.h"

/*******************************************************************************
* Function:  constructor
* Description: Инициализируем начальное значение фильтра нулем
* Threading usage and Assumptions:  none
******************************************************************************/
cFilter::cFilter(const tF32 filterConst) {
  ASSERT(filerConst != 0);
  filteredValue = (tF32)0;
  previousFilteredValue = (tF32)0;
  this->filterConstant = filterConst;
}

/*******************************************************************************
* Function:  constructor
* Description: Обновляет фильтр новым значением
* Threading usage and Assumptions:  none
******************************************************************************/
void cFilter::updateFilteredValue(const tF32 nonFileredValue) {
  this->filteredValue += (nonFileredValue - this->filteredValue) / this->filterConstant;   // подсчёт среднего значения
}


