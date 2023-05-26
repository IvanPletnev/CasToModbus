/*
 * rs232.h
 *
 *  Created on: Oct 16, 2022
 *      Author: Ivan
 */

#ifndef INC_CAS_H_
#define INC_CAS_H_

#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define CAS_WEIGHT_SIZE		22
#define CAS_RESPONSE_SIZE	7
#define CAS_BUFFER_SIZE		64


typedef enum {
	UNSTABLE,
	STABLE,
	OVERLOAD
}stabFactor_t;

typedef enum {
	GROSS,
	NET
}grossNet_t;

typedef enum {
	KILOGRAM,
	TON
}weightUnit_t;

typedef enum {
	READY,
	BUSY
}UART_Status_t;

typedef enum _CAS_Mode {
	CAS_WEIGHT,
	CAS_SETTINGS
}CAS_Mode_t;

typedef struct _casRxData {
	uint8_t casRxBuffer[32];
	uint8_t casRxSize;
}casRxData_t;

typedef struct _CAS_Data {
	uint8_t parsingIsOk;
	stabFactor_t stability;
	grossNet_t grossOrNet;
	uint8_t deviceId;
	uint8_t lampConditionByte;
	int32_t weightValue;
	uint8_t DP_position;
	weightUnit_t unit;
	casRxData_t casRxData;
	CAS_Mode_t casMode;
}CAS_Data_t;

extern volatile CAS_Data_t casData;
extern UART_Status_t uart1status;
extern UART_Status_t uart2status;

uint8_t CAS_Parcer (CAS_Data_t *data, casRxData_t *source);
void uartTxTask (void *argument);

#endif /* INC_CAS_H_ */
