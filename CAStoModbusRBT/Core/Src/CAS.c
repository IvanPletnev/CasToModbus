/*
 * rs232.c
 *
 *  Created on: Oct 16, 2022
 *      Author: Ivan
 */

#include "CAS.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>

extern uint8_t rs232_inputBuffer[64];
uint8_t CAS_inputSize = 0;
uint8_t MODBUS_InputSize = 0;
uint8_t CAS_RX_Flag = 0;
uint8_t MODBUS_RX_Flag = 0;

CAS_Data casData = {0};

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1){
		CAS_inputSize = Size;
		CAS_RX_Flag = 1;
	} else if (huart->Instance == USART2) {
		MODBUS_InputSize = Size;
		MODBUS_RX_Flag = 1;
	}
	HAL_UARTEx_ReceiveToIdle_DMA(huart, rs232_inputBuffer, 64);
}


uint8_t CAS_Parcer (uint8_t *buffer, uint8_t size, CAS_Data *data){

	uint8_t parseState = 0;
	uint8_t i = 0, j = 7;
	uint8_t weightBuffer[8] = {0};
	uint8_t tempWeightBuffer[8] = {0};
	uint8_t weightSign = 0;

	if (size == CAS_PACKET_SIZE) {

		switch (parseState) {
		case 0: {
			switch (buffer[0]){
			case 'S':
				if (buffer[1] == 'T') {
					data->stability = STABLE;
					parseState++;
				} else
					return 0;
				break;
			case 'U':
				if (buffer[1] == 'S') {
					data->stability = UNSTABLE;
					parseState++;
				} else
					return 0;
				break;
			case 'O':
				if (buffer[1] == 'L'){
					data->stability = OVERLOAD;
					parseState++;
				} else
					return 0;
				break;
			default:
				return 0;
			}
		}

		case 1: {
			switch(buffer[3]) {
			case 'G':
				if (buffer[4] == 'S'){
					data->grossOrNet = GROSS;
					parseState++;
				} else {
					return 0;
				}
				break;
			case 'N':
				if (buffer[4] == 'T'){
					data->grossOrNet = NET;
					parseState++;
				} else {
					return 0;
				}
				break;
			default:
				return 0;
			}
		}
		case 2: {
			data->deviceId = buffer[6];
			parseState++;
		}
		case 3: {
			memcpy((uint8_t*)weightBuffer, (uint8_t*) buffer+9, 8);
			i = j = 7;
			data->DP_position = 0;
			while (i) {
				if (weightBuffer[i] == '-') {
					weightSign = 1;
					weightBuffer[i] = 0x20;
				}
				if (weightBuffer[i] == '.'){
					data->DP_position = 7 - i;
					j++;
				} else {
					tempWeightBuffer[j] = weightBuffer[i];
				}
				i--;
				j--;
			}
			data->weightData = atol((const char*) tempWeightBuffer);
			if (weightSign) {
				data->weightData = - data->weightData;
			}
			parseState++;
		}
		case 4:
			switch (buffer[19]){
			case 'g':
				if (buffer[18] == 'k'){
					data->unit = KILOGRAM;
					parseState = 0;
				}
				break;
			case 't':
				data->unit = TON;
				break;
			default:
				return 0;
			}
			return 1;
		}

	} else {
		return 0;
	}

	return 1;
}
