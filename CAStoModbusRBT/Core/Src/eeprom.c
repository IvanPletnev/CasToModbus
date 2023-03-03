/*
 * eeprom.c
 *
 *  Created on: Apr 20, 2021
 *      Author: Ivan
 */

#include "main.h"
#include "eeprom.h"

extern I2C_HandleTypeDef hi2c1;

int eepromWrite (uint8_t *source, int size) {

	int remainingBytes;
	remainingBytes = size;
	uint16_t memoryAddress = 0;

	while (remainingBytes > 0) {
		if (HAL_I2C_Mem_Write(&hi2c1,(uint16_t) EEPROM_ADDRESS, memoryAddress, I2C_MEMADD_SIZE_16BIT, (uint8_t*) (source + memoryAddress),EEPROM_PAGE_SIZE, 1000) != HAL_OK) {
			Error_Handler();
		}

		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		}

		while (HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDRESS, EEPROM_MAX_TRIALS, I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT) {
		}

		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		}
		remainingBytes -= EEPROM_PAGE_SIZE;
		memoryAddress += EEPROM_PAGE_SIZE;
	}
	return 1;
}

int eepromRead (uint8_t *dest, int size) {

	if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, 0, I2C_MEMADD_SIZE_16BIT, dest, size, 1000)!=HAL_OK){
		Error_Handler();
	}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	}
	return 1;
}
