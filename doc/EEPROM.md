# EEPROM management software

## Introduction
The STG-85n comes with one 24LC64, a 64K I2C serial EEPROM by Microchip, connected on the same bus that the I2C2 peripheral. Our main use case for the EEPROM is to store configuration parameters.

## Design goals
The EEPROM management software should implement a reliable non-volatile store for 32-bit variables:

- Wear leveling should be implemented, by means of an O-buffer, so that each variable may be updated up to 15M times and up to 16 variables may be stored.
- Variables shouldn't be used as high-frequency counters, but as configuration parameters.
- A command should be provided to erase all the EEPROM.

## Hardware considerations
- EEPROM pages are 32 bytes long.
- EEPROM physical page boundaries start at addresses that are "integer multiples of the page size" and end at addresses that are "integer multiples of the page size minus 1":
  - `PageFirstAddress(N) = N * PageSize; N >= 0`
  - `PageLastAddress(N) = (N+1) * PageSize - 1; N >= 0`
- EEPROM cycle endurance, of 1M, is specified per page, not per single byte; it is recommended to update variables "on bulk" as much as possible.
- uC's DMA capability should be used.
- uC's CRC capability should be used.

## Description of the O-buffer

The O-buffer implementation is page based; each page stores a 16-bit parameter variable group (7) and one status variable. Each variable is integrity protected by a 16-bit CRC: HD=7; 0x978A = `x^16 + x^13 + x^11 + x^10 + x^9 + x^8 + x^4 +  x^2 + 1` (see Koopman). The bytes follow a big-endian ordering.

|Page bytes|16-bit data| 16-bit CRC|
|:---: | :--- | :---: |
|0 - 3|status | CRC |
|4 - 7|parameter 0| CRC|
|8 - 11|parameter 1| CRC|
|12 - 15|parameter 2| CRC|
|16 - 19|parameter 3| CRC|
|20 - 23|parameter 4| CRC|
|24 - 27|parameter 5| CRC|
|28 - 31|parameter 6| CRC|

### Notes
- Given that cycle endurance is page based, one status variable is used for each parameter group. Preferably, parameter variables should be grouped by affinity and updated together.

## DMA configuration
The DMA1 device is used;
- channel 4 for TX,
- and channel 5 for RX.

## Notes
- Interesting alternative to EEPROM: [Cypress’s Serial Nonvolatile SRAM](https://www.cypress.com/products/nvsram-nonvolatile-sram)

## References
### EEPROM
- [Microchip's datasheet for the 24LC64 I2C Serial EEPROM (the chip used in the STG-85n)](http://ww1.microchip.com/downloads/en/devicedoc/21189t.pdf)
- [*"Avoiding EEPROM Wearout"*, Better Embedded System SW Blog. Koopman, P. (2015)](https://betterembsw.blogspot.com/2015/07/avoiding-eeprom-wearout.html)
- [*"Avoiding EEPROM corruption problems"*, Better Embedded System SW Blog. Koopman, P. (2011)](https://betterembsw.blogspot.com/2011/11/avoiding-eeprom-corruption-problems.html)
- [Microchip's AN2526 Application note - *"AVR101: High Endurance EEPROM Storage"*. (2016)](http://ww1.microchip.com/downloads/en/AppNotes/doc2526.pdf)
- [Microchip's AN1449 Application note - *"High-Reliability and High-Frequency EEPROM Counter"*.](http://ww1.microchip.com/downloads/en/AppNotes/01449A.pdf)
### I2C
- ST's RM0091 Reference Manual. Chapter 26, Inter-integrated circuit (I2C) interface. See above (STM32) for full reference.
- [*"Using the I2C Bus"*](https://robot-electronics.co.uk/i2c-tutorial)
### CRC
- [*"Tutorial:Checksum and CRC Data Integrity Techniques for Aviation"*. Koopman, P.; Driscoll, K.; Hall, B. (2012)](https://users.ece.cmu.edu/~koopman/pubs/KoopmanCRCWebinar9May2012.pdf)
- [*"CRC for Protecting A Single Value"*, Checksum and CRC Central Blog. Koopman, P. (2012)](http://checksumcrc.blogspot.com/2012/01/crc-for-protecting-single-value.html)
- [*"What's the best CRC polynomial to use?"*, Better Embedded System SW Blog. Koopman, P. (2010)](https://betterembsw.blogspot.com/2010/05/whats-best-crc-polynomial-to-use.html)
