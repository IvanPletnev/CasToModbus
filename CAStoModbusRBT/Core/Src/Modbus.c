/*
 * Modbus.c
 *
 *  Created on: 17 окт. 2022 г.
 *      Author: Admin
 */

#include "Modbus.h"

modbusData_t modbusData;
uint8_t errorResponse[16] = {0};
extern CAS_Data_t casData;
extern UART_HandleTypeDef huart2;

uint16_t ModbusRTU_CRC(uint8_t * buf, uint8_t len) {
	uint16_t crc = 0xFFFF;

  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];          // XOR byte into least sig. byte of crc

    for (uint8_t i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}


void setTxMode (void) {
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, SET);
	HAL_GPIO_WritePin(RE_GPIO_Port, RE_Pin, SET);
}

void setRxMode (void) {
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, RESET);
	HAL_GPIO_WritePin(RE_GPIO_Port, RE_Pin, RESET);
}


uint8_t modbusParcer (uint8_t * buffer, uint8_t len, modbusData_t *modData){

	uint16_t hiTempWord = 0;
	uint16_t loTempWord = 0;
	uint16_t reqCrc = 0;
	uint16_t respCrc = 0;
	uint8_t errorCode = 0;

	if (buffer[0] != MODBUS_SLAVE_ADDRESS) {
		return 0;
	} else {
		modData->request.slave_ID = buffer[0];
		modData->request.functionalCode = buffer[1];
		modData->request.crc = ((uint16_t)buffer[len - 2]) << 8;
		modData->request.crc |= buffer[len - 1];

		reqCrc = ModbusRTU_CRC(buffer, len-2);
		if (reqCrc != modData->request.crc){
			return 0;
		} else {
			modData->request.registerAddress = ((uint16_t) buffer[2]) << 8;
			modData->request.registerAddress |= buffer[3];
			switch (modData->request.functionalCode){

			case 0x04:{

				switch (modData->request.registerAddress) {

				case 0: {
					modData->modbusResp[0] = modData->request.slave_ID;
					modData->modbusResp[1] = 0x04;
					modData->modbusResp[2] = 0x06;
					hiTempWord = (uint16_t)((casData.weightValue & 0xFFFF0000) >> 16);
					loTempWord = (uint16_t) (casData.weightValue & 0x0000FFFF);
					modData->modbusResp[3] = (uint8_t)((loTempWord & 0xFF00) >> 8);
					modData->modbusResp[4] = (uint8_t)(loTempWord & 0x00FF);
					modData->modbusResp[5] = (uint8_t)((hiTempWord & 0xFF00) >> 8);
					modData->modbusResp[6] = (uint8_t)(hiTempWord & 0x00FF);
					modData->modbusResp[7] = 0x00;
					modData->modbusResp[8] = (uint8_t)casData.stability;
					respCrc = ModbusRTU_CRC(modData->modbusResp, 9);
					modData->modbusResp[9] = (uint8_t)((respCrc & 0xFF00) >> 8);
					modData->modbusResp[10] = (uint8_t)(respCrc & 0x00FF);
					setTxMode();
//					HAL_UART_Transmit_DMA(&huart2, modData->modbusResp, 11);
					break;
				}

				default:
					errorCode = 0x02;
					break;
				}
				break;
			}
			case 0x05: {
				switch (modData->request.registerAddress) {
				case 0:

					modData->modbusResp[0] = modData->request.slave_ID;
					modData->modbusResp[1] = 0x05;
					modData->modbusResp[3] = buffer[3];
					if (buffer[4] == 0xFF) {
						modData->modbusResp[4] = 0xFF;
						//Сюда добавить посылку в CAS
					} else {
						modData->modbusResp[4] = 0;
					}
					modData->modbusResp[5] = 0;
					respCrc = ModbusRTU_CRC(modData->modbusResp, 8);
					modData->modbusResp[6] = (uint8_t)((respCrc & 0xFF00) >> 8);
					modData->modbusResp[7] = (uint8_t)(respCrc & 0x00FF);

					break;
				}
				break;
			}
			default:
				errorCode = 0x01;
				break;
			}
		}
	}
	return 1;
}
