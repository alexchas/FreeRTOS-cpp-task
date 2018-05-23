/*******************************************************************************
*  FILENAME: frtosWrapper.h
*  DESCRIPTION: Обёртка для FreeRTOS
*  Author: Alexander Chashkin
*******************************************************************************/
#ifndef FRTOSWRAPPER_H
#define FRTOSWRAPPER_H

#include <FreeRTOS.h>      // Препроцессорные директивы для настройки компиляции RTOS
#include <task.h>          // Планировщик, задачи
#include <event_groups.h>  // События
#include <queue.h>         // Очереди
#include "types.h"         // Стандартные типы проекта
#include "iactiveobject.h" // Описание интерфейса iactiveobject

#define NO_TICKS_WAIT (tU32)0

//типы, заменяющие стандартные типы FreeRTOS
typedef EventGroupHandle_t tEventGroupHandle;
typedef QueueHandle_t tQueueHandle;
typedef TaskHandle_t tTaskHandle;
typedef eNotifyAction tNotifyAction;

typedef void (*tTaskFunction)( void * );

typedef enum
{
   RS_pass = (tU8)0,
   RS_fail = (tU8)1
}tRtosStatus;

class cRTOS
{
   public:
     explicit cRTOS(void);
     void startScheduler(void) const;            
     tRtosStatus taskCreate(iActiveObject *pActiveObject, const tU16 stackDepth,
                            tU32 priority, const char * const name) const;   
     //lint -restore       
     void taskDelay(const tU32 timeIncrement) const;       
     void schedulerDisable(void) const;        
     tRtosStatus schedulerEnable(void) const;        
     tEventGroupHandle eventGroupCreate(void) const;       
     tU32 eventGroupSetBits(tEventGroupHandle eventGroup, const tU32 bitsToSet) const;
     tRtosStatus eventGroupSetBitsFromISR(tEventGroupHandle eventGroup, const tU32 bitsToSet) const;
     tU32 eventGroupClearBits(tEventGroupHandle eventGroup, const tU32 bitsToClear) const;
     tU32 eventGroupClearBitsFromISR(tEventGroupHandle eventGroup, const tU32 bitsToClear) const;
     tU32 eventGroupWaitBits(const tEventGroupHandle eventGroup, const tU32 bitsToWaitFor, const tBoolean clearOnExit, const tBoolean waitForAllBits, const tU32 ticksToWait) const;
     tQueueHandle queueCreate(const tU32 queueLength, const tU32 itemSize) const;
     tRtosStatus queueSend(const tQueueHandle queue, const void * pItemToQueue, const tU32 ticksToWait) const;
     tRtosStatus queueSendToFront(const tQueueHandle queue, const void * pItemToQueue, const tU32 ticksToWait) const;
     tRtosStatus queueSendToBack(const tQueueHandle queue, const void * pItemToQueue, const tU32 ticksToWait) const;
     tBoolean   queueReceive(const tQueueHandle queue, void * pBuffer, const tU32 ticksToWait) const;
     tRtosStatus taskNotify(const tTaskHandle taskToNotify, const tU32 value, const tNotifyAction eAction) const;
     tBoolean taskNotifyWait(const tU32 bitsToClearOnEntry, const tU32 BitsToClearOnExit, const tU32 *value, const tU32 ticksToWait) const;
    
  private: 
    static void run(const void *parameters);	
};

extern cRTOS oRTOS;

#endif //FRTOSWRAPPER_H



