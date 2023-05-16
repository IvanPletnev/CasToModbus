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
#include <stdio.h>
#include "cmsis_os.h"
#include "eeprom.h"

CAS_Data_t casData = {0};
extern osMessageQueueId_t casRxHandle;
extern osMessageQueueId_t modbusRxHandle;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;
extern uint16_t ledStatusCounter;

const char weightReqString[] = "D01KW\r\n";
const char zeroReqString[] = "D01KZ\r\n";

const char errorString[] = "ERROR, please try again\r\n";

uint8_t settingsState = 0;

UART_Status_t uart1status = READY;
UART_Status_t uart2status = READY;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1){
		casData.casRxData.casRxSize = Size;
		osMessageQueuePut(casRxHandle, &casData.casRxData, 0, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, casData.casRxData.casRxBuffer, 64);
	} else if (huart->Instance == USART2) {
		modbusData.rxData.modbusRxSize = Size;
		osMessageQueuePut(modbusRxHandle, &modbusData.rxData, 0, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, modbusData.rxData.modbusRxBuffer, 64);
	} else 	if (huart->Instance == USART3){
		casData.casRxData.casRxSize = Size;
		osMessageQueuePut(casRxHandle, &casData.casRxData, 0, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, casData.casRxData.casRxBuffer, 64);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) ||
			(__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET) ||
					(__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET) ||
						(__HAL_UART_GET_FLAG(huart, UART_FLAG_PE) != RESET)){
		huart->Instance->DR;
	}
	if (huart->Instance == USART1){
		HAL_UART_AbortReceive(&huart1);
		HAL_UART_AbortTransmit(&huart1);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, casData.casRxData.casRxBuffer, 64);
		uart1status = READY;
	}

	if (huart->Instance == USART2){
		HAL_UART_AbortReceive(&huart2);
		HAL_UART_AbortTransmit(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, modbusData.rxData.modbusRxBuffer, 64);
		uart2status = READY;
	}
}

uint8_t CAS_Parcer (CAS_Data_t *data, casRxData_t *source){

	uint8_t i = 0;
	int8_t  j = 0;
	uint8_t weightBuffer[8] = {0};
	uint8_t tempWeightBuffer[8] = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,};
	uint8_t weightSign = 0;

	if (strncmp((const char*)source->casRxBuffer,"US", 2) == 0) {
		data->stability = UNSTABLE;
	} else if (strncmp((const char*)source->casRxBuffer,"ST", 2) == 0) {
		data->stability = STABLE;
	} else if (strncmp((const char*)source->casRxBuffer,"OL", 2) == 0) {
		data->stability = OVERLOAD;
	} else if (strncmp((const char*)source->casRxBuffer,"SET", 3) == 0){
		data->casMode = CAS_SETTINGS;
		HAL_UART_AbortTransmit(&huart1);
		HAL_TIM_Base_Stop_IT(&htim2);
		ledStatusCounter = 199;
		HAL_UART_Transmit(&huart1, (uint8_t*)"SETTINGS MODE\r\n", strlen("SETTINGS MODE\r\n"), 100);
		return 1;
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
	return 1;
}

void casSettings (casRxData_t *buffer, modbusData_t *modBus) {

	uint8_t len = 0;
	uint8_t tempBuf[64] = {0};
	uint32_t baudRate = 0;
	uint16_t deviceID = 0;

	if (!settingsState) {
		if (strncmp((const char*)buffer->casRxBuffer, "baud\r\n", 6) == 0) {
			len = sprintf((char*)tempBuf, "--Current BaudRate %lu\r\nEnter new:\r\n", huart2.Init.BaudRate);
			HAL_UART_Transmit(&huart1, tempBuf, len, 100);
			settingsState = 1;
			return;
		} else if (strncmp((const char*)buffer->casRxBuffer, "ID\r\n", 4) == 0) {
			len = sprintf((char*)tempBuf, "--Current ID %lu\r\nEnter new ID:\r\n", modBus->settings.deviceId);
			HAL_UART_Transmit(&huart1, tempBuf, len, 100);
			settingsState = 2;
			return;
		} else if (strncmp((const char*)buffer->casRxBuffer, "save\r\n", 6) == 0) {
			HAL_UART_Transmit(&huart1, (uint8_t*)"Save and reset?\r\n", strlen("Save and reset?\r\n"), 100);
			HAL_UART_Transmit(&huart1, (uint8_t*)"Press y or n\r\n", strlen("Press Y or N\r\n"), 100);
			settingsState = 3;
			return;
		} else {
			HAL_UART_Transmit(&huart1, (uint8_t*)errorString, strlen(errorString), 100);
		}
	}
	if (settingsState == 1) {
		if ((baudRate = atol ((const char *)buffer->casRxBuffer)) != 0){
			huart2.Init.BaudRate = baudRate;
			modBus->settings.baudRate = baudRate;
			len = sprintf((char*)tempBuf, "----OK. Current BaudRate %lu\r\n\r\n", huart2.Init.BaudRate);
			HAL_UART_Transmit(&huart1, tempBuf, len, 100);
			HAL_UART_Transmit(&huart1, (uint8_t*)"SETTINGS MODE\r\n", strlen("SETTINGS MODE\r\n"), 100);
			settingsState = 0;
			return;
		} else {
			HAL_UART_Transmit(&huart1, (uint8_t*)errorString, strlen(errorString), 100);
			return;
		}
	}
	if (settingsState == 2) {
		if ((deviceID = (uint8_t) atoi((const char *)buffer->casRxBuffer)) != 0){
			modBus->settings.deviceId = deviceID;
			len = sprintf((char*)tempBuf, "----OK. Current ID %d\r\n\r\n", deviceID);
			HAL_UART_Transmit(&huart1, tempBuf, len, 100);
			HAL_UART_Transmit(&huart1, (uint8_t*)"SETTINGS MODE\r\n", strlen("SETTINGS MODE\r\n"), 100);
			settingsState = 0;
			return;
		}
	}
	if (settingsState ==3) {
		if (strncmp ((const char*)buffer->casRxBuffer, "y\r\n", 3) == 0 || strncmp ((const char*)buffer->casRxBuffer, "Y\r\n", 3) == 0){
			HAL_UART_Transmit(&huart1, (uint8_t*)"Save...", strlen("Save..."), 100);
			eepromWrite((uint8_t*)&modBus->settings, sizeof(modbusSettings_t));
			osDelay(100);
			NVIC_SystemReset();
		} else if (strncmp ((const char*)buffer->casRxBuffer, "n\r\n", 3) == 0 || strncmp ((const char*)buffer->casRxBuffer, "N\r\n", 3) == 0){
			HAL_UART_Transmit(&huart1, (uint8_t*)"SETTINGS MODE\r\n", strlen("SETTINGS MODE\r\n"), 100);
			settingsState = 0;
			return;
		}
	}
}

void casTask (void *argument) {

	casRxData_t casBuf;
	volatile CAS_Data_t *cas;
	cas = (CAS_Data_t *) argument;
	ledStatusCounter = 999;
	osDelay(10);
	eepromRead((uint8_t*)&modbusData.settings, sizeof(modbusSettings_t));
	huart2.Init.BaudRate = modbusData.settings.baudRate;
	HAL_UART_Init(&huart2);
	osDelay(10);
	for(;;) {

		if (osMessageQueueGet(casRxHandle, (casRxData_t*)&casBuf, 0, osWaitForever) == osOK) {
			if (cas->casMode == CAS_WEIGHT) {
				CAS_Parcer((CAS_Data_t *)cas, &casBuf);
			} else if (cas->casMode == CAS_SETTINGS) {
				casSettings(&casBuf, &modbusData);
			}
			osThreadYield();
		}
	}
}

void uartTxTask (void *argument) {

	uint32_t flag1 = 0;

	for (;;) {

<<<<<<< HEAD
		flag1 = osThreadFlagsWait(0x07, osFlagsWaitAny, osWaitForever);
		if (flag1 == 0x01) {
			uart1status = BUSY;
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *) weightReqString, strlen (weightReqString));
		} else if (flag1 == 0x02){
			uart1status = BUSY;
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *) zeroReqString, strlen (zeroReqString));
			HAL_TIM_Base_Start_IT(&htim2);
		} else if (flag1 == 0x04) {
=======
		flag1 = osThreadFlagsWait(0x03, osFlagsWaitAny, 1);
		if (flag1 & 0x01) {
			uart1status = BUSY;
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *) weightReqString, strlen (weightReqString));
		} else if (flag1 & 0x02){
			uart1status = BUSY;
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *) zeroReqString, strlen (zeroReqString));
		}

		flag2 = osThreadFlagsWait(0x0C, osFlagsWaitAny, 1);
		if (flag2 & 0x04) {
>>>>>>> branch 'master' of git@github.com:IvanPletnev/CasToModbus.git
			uart2status = BUSY;
			setTxMode();
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&modbusData.modbusResp, modbusData.modbusResponseSize);
		}
	}
}
