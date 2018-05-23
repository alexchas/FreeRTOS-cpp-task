/*******************************************************************************
*  FILENAME: filter.h
*
*  DESCRIPTION: ����� ��� ����������, ������ ��������� �������� � ���������
*  ���������
*
*  Created on: 21 ��� 2018 �.
*  Author: Alexander Chashkin
******************************************************************************/
#ifndef FILTER_H_
#define FILTER_H_
#include "types.h"

#define COUNT_FILTER	256

class cFilter {
  public:
	explicit cFilter(const tF32 filterConst);
    tF32 getFilteredValue() const { return filteredValue; };
    void  updateFilteredValue(const tF32 nonFileredValue);
  private:
    tF32 filteredValue;
    tF32 previousFilteredValue;
    tF32 filterConstant;
};

#endif /* FILTER_H_ */
