/*
 * Modbus.h
 *
 *  Created on: 17 окт. 2022 г.
 *      Author: Admin
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#include "main.h"
#include "CAS.h"


#define MODBUS_SLAVE_ADDRESS	0x01

#define READ_DO				0x01
#define READ_DI				0x02
#define	READ_AO				0x03
#define READ_AI				0x04
#define WRITE_DO			0x05
#define WRITE_AO			0x06
#define WRITE_MULTIPLE_DO	0x0F
#define	WRITE_MULTIPLE_AO	0x10

typedef struct _modbusRequest {
	uint8_t slave_ID;
	uint8_t functionalCode;
	uint16_t registerAddress;
	uint16_t registersNumber;
	uint16_t crc;
}modbusRequest_t;

typedef struct _modbusRxData {
	uint8_t modbusRxBuffer[64];
	uint8_t modbusRxSize;
	uint8_t modbusRxFlag;
}modbusRxData_t;

typedef struct _modbusData {
	modbusRequest_t request;
	uint8_t modbusResp[32];
	modbusRxData_t rxData;
}modbusData_t;

extern modbusData_t modbusData;


#endif /* INC_MODBUS_H_ */
