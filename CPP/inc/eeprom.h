/*
 * eeprom.h
 *
 *  Created on: Aug 7, 2025
 *      Author: Gil Vargas
 */
/*EEPROM
 * xF800
 * Motor direction:1
 * Startup throttle:2
 * Turtle rampup:1
 * Protocol:1 = 0=notset, 1=300,2=300-bi,3=600,4=6000-bi.
 * CommutationDelay:1, 1=0.5us,2=1us,3=1.5us,4=2us,5=2.5us
 *
 */
#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_ID 			 0x47
#define BASE_ADDRESS		 0x08000000
#define APPLICATION_ADDRESS  0x1000

#define EEPROM_ADDRESS 		 0xF800

#define  WORD_SIZE  (0x04)
#define PAGE_SIZE  (0x800)



void read_memory(char *data, const uint16_t size, const uint32_t address);
void write_eeprom(char *data, uint16_t size, uint32_t address);

void write_memory(uint8_t* data,const uint16_t size,const uint32_t address) ;
#endif /* INC_EEPROM_H_ */
