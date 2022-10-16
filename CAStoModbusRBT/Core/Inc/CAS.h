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

#define CAS_PACKET_SIZE		22


typedef enum {
	UNSTABLE,
	STABLE,
	OVERLOAD
}stabFactor;

typedef enum {
	GROSS,
	NET
}grossNet;

typedef enum {
	KILOGRAM,
	TON
}weightUnit;

typedef struct _CAS_Data {

	uint8_t parsingIssOk;
	stabFactor stability;
	grossNet grossOrNet;
	uint8_t deviceId;
	uint8_t lampConditionByte;
	int32_t weightData;
	uint8_t DP_position;
	weightUnit unit;

}CAS_Data;



#endif /* INC_CAS_H_ */
