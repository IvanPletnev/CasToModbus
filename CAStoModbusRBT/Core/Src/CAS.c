/*
 * rs232.c
 *
 *  Created on: Oct 16, 2022
 *      Author: Ivan
 */

#include "CAS.h"
#include "Modbus.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>



CAS_Data_t casData = {0};
extern osMessageQueueId_t casRxHandle;
extern UART_HandleTypeDef huart1;

const char weightReqString[] = "D01KW\r\n";
const char zeroReqString[] = "D01KZ\r\n";
UART_Status_t uart1status = READY;


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1){
		casData.casRxData.casRxSize = Size;
		osMessageQueuePut(casRxHandle, &casData.casRxData, 0, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, casData.casRxData.casRxBuffer, 64);
	} else if (huart->Instance == USART2) {
		modbusData.rxData.modbusRxSize = Size;
		modbusData.rxData.modbusRxFlag = 1;
		HAL_UARTEx_ReceiveToIdle_DMA(huart, modbusData.rxData.modbusRxBuffer, 64);
	}
}

uint8_t CAS_Parcer (CAS_Data_t *data, casRxData_t *source){

	uint8_t i = 0;
	int8_t  j = 0;
	uint8_t weightBuffer[8] = {0};
	uint8_t tempWeightBuffer[8] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,};
	uint8_t weightSign = 0;

	if (source->casRxSize == CAS_WEIGHT_SIZE) {

		if (strncmp((const char*)source->casRxBuffer,"US", 2) == 0) {
			data->stability = UNSTABLE;
		} else if (strncmp((const char*)source->casRxBuffer,"ST", 2) == 0) {
			data->stability = STABLE;
		} else if (strncmp((const char*)source->casRxBuffer,"OL", 2) == 0) {
			data->stability = OVERLOAD;
		} else {
			//error handler
		}

		if (strncmp((const char*)source->casRxBuffer + 3,"GS", 2) == 0) {
			data->grossOrNet = GROSS;
		} else if (strncmp((const char*)source->casRxBuffer + 3,"NT", 2) == 0) {
			data->grossOrNet = NET;
		} else {
			//error handler
		}

		data->deviceId = data->casRxData.casRxBuffer[6] - 0x30;

		memcpy((uint8_t*)weightBuffer, (uint8_t*) source->casRxBuffer + 9, 8);
		i = j = 0;
		data->DP_position = 0;
		for (i = 0; i < 8; ) {
			if (weightBuffer[i] == '-') {
				weightSign = 1;
				weightBuffer[i] = 0x20;
			}
			if (weightBuffer[i] == '.'){
				data->DP_position = 7 - i;
				j--;
			} else {
				tempWeightBuffer[j] = weightBuffer[i];
			}
			i++;
			j++;
		}
		data->weightValue = atol((const char*) tempWeightBuffer);
		if (weightSign) {
			data->weightValue = - data->weightValue;
		}

		if (source->casRxBuffer[19] == 'g') {
			if (source->casRxBuffer[18] == 'k') {
				data->unit = KILOGRAM;
			}
		} else if (source->casRxBuffer[19] == 't') {
			data->unit = TON;
		} else {
			//error handler
		}

	} else if (source->casRxSize == CAS_RESPONSE_SIZE){

	} else {
		return 0;
	}

	return 1;
}

void casTask (void *argument) {

	casRxData_t casBuf;
	volatile CAS_Data_t *cas;
	cas = (CAS_Data_t *) argument;

	for(;;) {

		if (osMessageQueueGet(casRxHandle, (casRxData_t*)&casBuf, 0, osWaitForever) == osOK) {
			CAS_Parcer((CAS_Data_t *)cas, &casBuf);
			osThreadYield();
		}
	}
}

void uartTxTask (void *argument) {

	uint32_t flag = 0;

	for (;;) {
		if (uart1status == READY) {
			flag = osThreadFlagsWait(0x03, osFlagsWaitAny, osWaitForever);
			if (flag == 0x01) {
				uart1status = BUSY;
				HAL_UART_Transmit_DMA(&huart1, (uint8_t *) weightReqString, strlen (weightReqString));
			} else if (flag == 0x02){
				uart1status = BUSY;
				HAL_UART_Transmit_DMA(&huart1, (uint8_t *) zeroReqString, strlen (zeroReqString));
			}
		} else {
			osDelay(1);
		}
	}
}
