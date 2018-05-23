/*******************************************************************************
*  FILENAME: iActiveObject.h
*
*  DESCRIPTION: Интерфейс для активных объектов. Каждый активный объект должен
*               наследовать этот интерфейс
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
