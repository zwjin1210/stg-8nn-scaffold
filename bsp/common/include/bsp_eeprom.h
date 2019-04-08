#ifndef _BSP_EEPROM_H
#define _BSP_EEPROM_H

#include <stdint.h>

void BSP_EEPROM_initAO(void);
void BSP_EEPROM_startAO(const uint8_t priority);

void BSP_EEPROM_read(uint16_t startAddress, uint8_t nBytes);
void BSP_EEPROM_write(uint16_t startAddress, uint8_t nBytes, uint8_t* data);

#endif /* _BSP_EEPROM_H */
