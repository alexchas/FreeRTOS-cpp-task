/*******************************************************************************
*  FILENAME: iActiveObject.h
*
*  DESCRIPTION: ��������� ��� �������� ��������. ������ �������� ������ ������
*               ����������� ���� ���������
*
*Author: Alexander Chashkin
*******************************************************************************/
#ifndef IACTIVEOBJECT_H
#define IACTIVEOBJECT_H

class iActiveObject {
  public:              
    virtual void run(void) = 0;
    void *taskHandle;
};

#endif
