/*******************************************************************************
*  FILENAME: frtosWrapper.cpp
*
*  DESCRIPTION: ���������� ������ frtosWrapper
*
*  Created on: 21 ��� 2018 �.
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
* Description: ������ ������������
* Threading usage and Assumptions:  ��� ������ ������ ���� ������� � �������
*  taskCreate() �� ������ ������ �������
******************************************************************************/
void cRTOS::startScheduler(void) const {     // sheduler start
   vTaskStartScheduler();
}

/*******************************************************************************
* Function:  taskCreate
* Description: �������� ������, � �������� ������ ����� ��������������
* ����������� ����� run ������ cRTOS, � ������� ����� ���������� �������� �����
* ������� ���������� iActiveObject
******************************************************************************/
tRtosStatus cRTOS::taskCreate(iActiveObject *pActiveObject, const tU16 stackDepth, tU32 priority, const char * const name) const {
   tRtosStatus status;   
   const BaseType_t rowStatus = xTaskCreate((TaskFunction_t)cRTOS::run, name, (uint16_t)stackDepth, pActiveObject, (uint32_t)priority, &pActiveObject->taskHandle);  // tTaskFunction ��� ������������� �� ����������
   if(rowStatus == pdTRUE) status = RS_pass;
   else 				   status = RS_fail;
   return status;
}

/*******************************************************************************
* Function:  taskDelay
* Description: ������������ ������
******************************************************************************/
void cRTOS::taskDelay(const tU32 timeIncrement) const {    // // task pause
   vTaskDelay((TickType_t)timeIncrement);
}

/*******************************************************************************
* Function:  schedulerDisable
* Description: ��������� ������������
******************************************************************************/
void cRTOS::schedulerDisable(void) const {   // sheduler stop
   vTaskSuspendAll();
}

/*******************************************************************************
* Function:  schedulerEnable
* Description: �������������� ������ ������������
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
* Description: �������� event group
******************************************************************************/
tEventGroupHandle cRTOS::eventGroupCreate(void) const {
   tEventGroupHandle createdEventGroup;
   createdEventGroup = (tEventGroupHandle)xEventGroupCreate();
   return createdEventGroup;
}

/*******************************************************************************
* Function:  eventGroupSetBits
* Description: ������������� ���� � event group
******************************************************************************/
tU32 cRTOS::eventGroupSetBits(tEventGroupHandle eventGroup, const tU32 bitsToSet) const {
   tU32 bits;
   bits = (tU32)xEventGroupSetBits((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToSet);
   
   return bits;
}

/*******************************************************************************
* Function:  eventGroupSetBitsFromISR
* Description: ������ eventGroupSetBits, ���������� �� ����������� ����������
******************************************************************************/
tRtosStatus cRTOS::eventGroupSetBitsFromISR(tEventGroupHandle eventGroup, const tU32 bitsToSet) const {
   tRtosStatus status;
   BaseType_t result;
   BaseType_t xHigherPriorityTaskWoken;
   xHigherPriorityTaskWoken = (BaseType_t)FALSE;
   result = xEventGroupSetBitsFromISR((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToSet, &xHigherPriorityTaskWoken);        // ��� tEventGroupHandle ������������ ��� ������������� �� ����������
                                                                                                                                 // ������� xTimerPendFunctionCallFromISR (��. event_groups.h) ����� 4 ���������, ������ �� ������� ����� ��� 'PendedFunction_t
   if( result != pdFAIL ) {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);       // ���������� ���� unsigned long � ���������, ��������� ���������� � uint32 ���� � ���������� STM
      status = RS_pass;
   }
   else {
      status = RS_fail;       //timer service queue was full
   }
   return status;
}


/*******************************************************************************
* Function:  eventGroupClearBits
* Description: ������� ��������� ���� � event group. ���������� �������� event
*  group �� ���� ��� ��������� ���� ���� �������
******************************************************************************/
tU32 cRTOS::eventGroupClearBits(tEventGroupHandle eventGroup, const tU32 bitsToClear) const {
   tU32 bits;
   bits = (tU32)xEventGroupClearBits((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToClear);
   return bits;
}


/*******************************************************************************
* Function:  eventGroupClearBitsFromISR
* Description: ������ eventGroupClearBits, ���������� �� ����������� ����������.
*  ���������� �������� event group �� ���� ��� ��������� ���� ���� �������.
******************************************************************************/
tU32 cRTOS::eventGroupClearBitsFromISR(tEventGroupHandle eventGroup, const tU32 bitsToClear) const {
   tU32 bits;
   bits = (tU32)xEventGroupClearBitsFromISR((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToClear);   // 1. - ��� tEventGroupHandle ������������ ��� ������������� �� ����������
                                                                                                         // 2. - ������� xTimerPendFunctionCallFromISR (��. event_groups.h) ����� 4 ���������, ������ �� ������� ����� ��� 'PendedFunction_t'.
   return bits;
}


/*******************************************************************************
* Function:  eventGroupWaitBits
* Description: ������� ��������� ��������� ����� � event group � ������ ������ �
               ������������� ��������� � ������� ������� ticksToWait

******************************************************************************/
tU32 cRTOS::eventGroupWaitBits(const tEventGroupHandle eventGroup, const tU32 bitsToWaitFor, const tBoolean clearOnExit, const tBoolean waitForAllBits, tU32 ticksToWait) const {
   tU32 uxBits;
   uxBits = (tU32)xEventGroupWaitBits((EventGroupHandle_t)eventGroup, (EventBits_t)bitsToWaitFor,                    // ��� tEventGroupHandle ������������ ��� ������������� �� ����������
            (BaseType_t)clearOnExit, (BaseType_t)waitForAllBits, (TickType_t)ticksToWait);
   
   return uxBits;
}


/*******************************************************************************
* Function:  queueCreate
* Description: ������� �������, � ������ �������� ������ ���������� NULL
******************************************************************************/
tQueueHandle cRTOS::queueCreate( const tU32 queueLength, const tU32 itemSize) const {
  tQueueHandle queueHandle =  NULL;
  queueHandle = (tQueueHandle)xQueueCreate(queueLength, itemSize);
  return queueHandle;                
}

/*******************************************************************************
* Function:  queueSend
* Description: ���������� �������� � �������
******************************************************************************/
tRtosStatus cRTOS::queueSend(const tQueueHandle queue, const void * pItemToQueue, const tU32 ticksToWait) const {
  tRtosStatus status;
  status = this->queueSendToBack(queue,pItemToQueue, ticksToWait);
  return status;                
}


/*******************************************************************************
* Function:  queueSend
* Description: ���������� �������� � ������ �������
******************************************************************************/
tRtosStatus cRTOS::queueSendToFront(const tQueueHandle queue, const void * pItemToQueue, tU32 ticksToWait) const {
  tRtosStatus status;
  status = (tRtosStatus)xQueueSendToFront((QueueHandle_t)queue, pItemToQueue, (TickType_t)ticksToWait);
  return status;                
}

/*******************************************************************************
* Function:  queueSend
* Description: ���������� �������� � ����� �������
******************************************************************************/
tRtosStatus cRTOS::queueSendToBack(const tQueueHandle queue, const void * pItemToQueue, tU32 ticksToWait) const {
  tRtosStatus status;
  status = (tRtosStatus)xQueueSend((QueueHandle_t)queue, pItemToQueue, (TickType_t)ticksToWait);
  return status;                
}

/*******************************************************************************
* Function:  queueReceive
* Description: ����� �� �������
******************************************************************************/
tBoolean cRTOS::queueReceive(const tQueueHandle queue, void * pBuffer, const tU32 ticksToWait) const {
  tBoolean status;
  status = (tBoolean)xQueueReceive((QueueHandle_t)queue, pBuffer, 
                                   (TickType_t)ticksToWait);
  return status;                
}


/*******************************************************************************
* Function:  taskNotify
* Description: ����������� ������ ������
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
* Description: �������� ����������� �� ������ �����
******************************************************************************/
tBoolean cRTOS::taskNotifyWait(const tU32 bitsToClearOnEntry, const tU32 BitsToClearOnExit, const tU32 *pValue, const tU32 ticksToWait) const {
  tBoolean status;
  status = xTaskNotifyWait(bitsToClearOnEntry, BitsToClearOnExit, (uint32_t *)pValue, (TickType_t)ticksToWait); 
  return status;                
}

/*******************************************************************************
* Function:  run
* Description: ������� ���������� ������� � ������������ �������, ��� ����������� ������ ��� ��� �������� ������ ���������� ��������� �� ����������
* �������. ���� ������� �������� ����� run ������� ������� ��������� ��������� iActiveObject. ����� ������, ������� ����� ���� ������� ������ ������������ ���� ��������� � ����������� ������� run
*
******************************************************************************/
void cRTOS::run(const void *parameters) {
  ((iActiveObject*)parameters)->run();
}

