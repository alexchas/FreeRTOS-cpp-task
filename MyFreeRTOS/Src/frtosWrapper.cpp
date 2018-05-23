/*******************************************************************************
*  FILENAME: frtosWrapper.cpp
*
*  DESCRIPTION: реализация класса frtosWrapper
*
*  Created on: 21 мая 2018 г.
*  Author: Alexander Chashkin
*******************************************************************************/

#include "frtosWrapper.h"

/*******************************************************************************
* Function:  constructor
* Description:
******************************************************************************/
cRTOS::cRTOS(void){}   // constructor

/*******************************************************************************
* Function:  startScheduler
* Description: Запуск планировщика
* Threading usage and Assumptions:  Все задачи должны быть созданы с помощью
*  taskCreate() до вызова данной функции
******************************************************************************/
void cRTOS::startScheduler(void) const {     // sheduler start
   vTaskStartScheduler();
}

/*******************************************************************************
* Function:  taskCreate
* Description: Создание задачи, в качестве задачи будет использоваться
* статический метод run класса cRTOS, в котором будет вызываться реальный метод
* объекта наследника iActiveObject
******************************************************************************/
tRtosStatus cRTOS::taskCreate(iActiveObject *pActiveObject, const tU16 stackDepth, tU32 priority, const char * const name) const {
   tRtosStatus status;   
   const BaseType_t rowStatus = xTaskCreate((TaskFunction_t)cRTOS::run, name, (uint16_t)stackDepth, pActiveObject, (uint32_t)priority, &pActiveObject->taskHandle);  // tTaskFunction для совместимости со стандартом
   if(rowStatus == pdTRUE) status = RS_pass;
   else 				   status = RS_fail;
   return status;
}

/*******************************************************************************
* Function:  taskDelay
* Description: Приостановка задачи
******************************************************************************/
void cRTOS::taskDelay(const tU32 timeIncrement) const {    // // task pause
   vTaskDelay((TickType_t)timeIncrement);
}

/*******************************************************************************
* Function:  schedulerDisable
* Description: Остановка планировщика
******************************************************************************/
void cRTOS::schedulerDisable(void) const {   // sheduler stop
   vTaskSuspendAll();
}

/*******************************************************************************
* Function:  schedulerEnable
* Description: Восстановление работы планировщика
******************************************************************************/
tRtosStatus cRTOS::schedulerEnable(void) const {  // sheduler go
   tRtosStatus status;
   const BaseType_t rowStatus = xTaskResumeAll();
   if(rowStatus == pdTRUE) {
      status = RS_pass;
   }
   else {
      status = RS_fail;
   }
   return status;
}

/*******************************************************************************
* Function:  eventGroupCreate
* Description: Создание event group
******************************************************************************/
tEventGroupHandle cRTOS::eventGroupCreate(void) const {
   tEventGroupHandle createdEventGroup;
   createdEventGroup = (tEventGroupHandle)xEventGroupCreate();
   return createdEventGroup;
}

/*******************************************************************************
* Function:  eventGroupSetBits
* Description: Устанавливает биты в event group
******************************************************************************/
tU32 cRTOS::eventGroupSetBits(tEventGroupHandle eventGroup, const tU32 bitsToSet) const {
   tU32 bits;
   bits = (tU32)xEventGroupSetBits((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToSet);
   
   return bits;
}

/*******************************************************************************
* Function:  eventGroupSetBitsFromISR
* Description: Версия eventGroupSetBits, вызываемая из обработчика прерывания
******************************************************************************/
tRtosStatus cRTOS::eventGroupSetBitsFromISR(tEventGroupHandle eventGroup, const tU32 bitsToSet) const {
   tRtosStatus status;
   BaseType_t result;
   BaseType_t xHigherPriorityTaskWoken;
   xHigherPriorityTaskWoken = (BaseType_t)FALSE;
   result = xEventGroupSetBitsFromISR((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToSet, &xHigherPriorityTaskWoken);        // тип tEventGroupHandle используется для совместимости со стандартом
                                                                                                                                 // функция xTimerPendFunctionCallFromISR (см. event_groups.h) имеет 4 аргумента, первый из которых имеет тип 'PendedFunction_t
   if( result != pdFAIL ) {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);       // приведение типа unsigned long к указателю, нестрогое приведение к uint32 было в библиотеке STM
      status = RS_pass;
   }
   else {
      status = RS_fail;       //timer service queue was full
   }
   return status;
}


/*******************************************************************************
* Function:  eventGroupClearBits
* Description: Очищает выбранные биты в event group. Возвращает значение event
*  group до того как выбранные биты были очищены
******************************************************************************/
tU32 cRTOS::eventGroupClearBits(tEventGroupHandle eventGroup, const tU32 bitsToClear) const {
   tU32 bits;
   bits = (tU32)xEventGroupClearBits((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToClear);
   return bits;
}


/*******************************************************************************
* Function:  eventGroupClearBitsFromISR
* Description: Версия eventGroupClearBits, вызываемая из обработчика прерывания.
*  Возвращает значение event group до того как выбранные биты были очищены.
******************************************************************************/
tU32 cRTOS::eventGroupClearBitsFromISR(tEventGroupHandle eventGroup, const tU32 bitsToClear) const {
   tU32 bits;
   bits = (tU32)xEventGroupClearBitsFromISR((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToClear);   // 1. - тип tEventGroupHandle используется для совместимости со стандартом
                                                                                                         // 2. - функция xTimerPendFunctionCallFromISR (см. event_groups.h) имеет 4 аргумента, первый из которых имеет тип 'PendedFunction_t'.
   return bits;
}


/*******************************************************************************
* Function:  eventGroupWaitBits
* Description: Ожидает установки выбранных битов в event group и держит задачу в
               блокированном состоянии в течение времени ticksToWait

******************************************************************************/
tU32 cRTOS::eventGroupWaitBits(const tEventGroupHandle eventGroup, const tU32 bitsToWaitFor, const tBoolean clearOnExit, const tBoolean waitForAllBits, tU32 ticksToWait) const {
   tU32 uxBits;
   uxBits = (tU32)xEventGroupWaitBits((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToWaitFor,                    // тип tEventGroupHandle используется для совместимости со стандартом
            (BaseType_t)clearOnExit, (BaseType_t)waitForAllBits, (TickType_t)ticksToWait);
   
   return uxBits;
}


/*******************************************************************************
* Function:  queueCreate
* Description: Создает очередь, в случае нехватки памяти возвращает NULL
******************************************************************************/
tQueueHandle cRTOS::queueCreate( const tU32 queueLength, const tU32 itemSize) const {
  tQueueHandle queueHandle =  NULL;
  queueHandle = (tQueueHandle)xQueueCreate(queueLength, itemSize);
  return queueHandle;                
}

/*******************************************************************************
* Function:  queueSend
* Description: Добавление элемента в очередь
******************************************************************************/
tRtosStatus cRTOS::queueSend(const tQueueHandle queue, const void * pItemToQueue, const tU32 ticksToWait) const {
  tRtosStatus status;
  status = this->queueSendToBack(queue,pItemToQueue, ticksToWait);
  return status;                
}


/*******************************************************************************
* Function:  queueSend
* Description: Добавление элемента в начало очереди
******************************************************************************/
tRtosStatus cRTOS::queueSendToFront(const tQueueHandle queue, const void * pItemToQueue, tU32 ticksToWait) const {
  tRtosStatus status;
  status = (tRtosStatus)xQueueSendToFront((QueueHandle_t)queue, pItemToQueue, (TickType_t)ticksToWait);
  return status;                
}

/*******************************************************************************
* Function:  queueSend
* Description: Добавление элемента в конец очереди
******************************************************************************/
tRtosStatus cRTOS::queueSendToBack(const tQueueHandle queue, const void * pItemToQueue, tU32 ticksToWait) const {
  tRtosStatus status;
  status = (tRtosStatus)xQueueSend((QueueHandle_t)queue, pItemToQueue, (TickType_t)ticksToWait);
  return status;                
}

/*******************************************************************************
* Function:  queueReceive
* Description: Прием из очереди
******************************************************************************/
tBoolean cRTOS::queueReceive(const tQueueHandle queue, void * pBuffer, const tU32 ticksToWait) const {
  tBoolean status;
  status = (tBoolean)xQueueReceive((QueueHandle_t)queue, pBuffer, 
                                   (TickType_t)ticksToWait);
  return status;                
}


/*******************************************************************************
* Function:  taskNotify
* Description: Нотификация нужной задачи
******************************************************************************/
tRtosStatus cRTOS::taskNotify(const tTaskHandle taskToNotify, const tU32 value, const tNotifyAction eAction) const {
  tRtosStatus status = RS_fail;
  const BaseType_t rowStatus = xTaskNotify(taskToNotify, value, eAction);
  if(rowStatus == pdTRUE) {
      status = RS_pass;
   }
   else {
      status = RS_fail;
   }
   return status;                
}

/*******************************************************************************
* Function:  taskNotifyWait
* Description: Ожидание нотификации от других задач
******************************************************************************/
tBoolean cRTOS::taskNotifyWait(const tU32 bitsToClearOnEntry, const tU32 BitsToClearOnExit, const tU32 *pValue, const tU32 ticksToWait) const {
  tBoolean status;
  status = xTaskNotifyWait(bitsToClearOnEntry, BitsToClearOnExit, (uint32_t *)pValue, (TickType_t)ticksToWait); 
  return status;                
}

/*******************************************************************************
* Function:  run
* Description: Функция являющаяся задачей в операционной системе, она статическая потому что при создании задачи передается указатель на конкретную
* функцию. Сама функция вызывает метод run объекта который наследует интерфейс iActiveObject. Любой объект, который хочет быть задачей должен унаследовать этот интерфейс и реализовать функцию run
*
******************************************************************************/
void cRTOS::run(const void *parameters) {
  ((iActiveObject*)parameters)->run();
}

