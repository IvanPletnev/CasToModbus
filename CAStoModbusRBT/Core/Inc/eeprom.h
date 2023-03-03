/*
 * eeprom.h
 *
 *  Created on: Feb 4, 2021
 *      Author: Ivan
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_ADDRESS	0x50 << 1
#define	EEPROM_PAGE_SIZE	64

#define I2Cx_TIMEOUT_MAX    300
#define EEPROM_MAX_TRIALS   300

int eepromWrite (uint8_t *source, int size);
int eepromRead (uint8_t *dest, int size);

#endif /* INC_EEPROM_H_ */
