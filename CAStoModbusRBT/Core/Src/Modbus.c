/*
 * Modbus.c
 *
 *  Created on: 17 окт. 2022 г.
 *      Author: Admin
 */

#include "Modbus.h"

modbusRequest packet;

uint8_t modbusParcer (uint8_t * buffer, uint8_t len, modbusRequest *pack){
	if (buffer[0] != MODBUS_SLAVE_ADDRESS) {
		return 0;
	} else {
		switch (buffer[1]){
		case 0x04:
			pack->registerAddress = ((uint16_t) buffer[2]) << 8;
			pack->registerAddress |= buffer[3];
			pack->registersNumber = ((uint16_t) buffer[4]) << 8;
			pack->registersNumber |= buffer[5];

			break;
		}
	}
}

uint16_t ModRTU_CRC(uint8_t * buf, uint8_t len) {
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
