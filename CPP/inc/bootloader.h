/*
 * bootloader.h
 *
 *  Created on: Aug 10, 2025
 *      Author: Gil Vargas
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#define BUFFER_SIZE (0x12C)

#define ACK           (0x30)
#define COMMAND_ERROR      0xC1
#define CRC_ERROR          (0xC2)

#define ACK_OK                  (0x00)

#define ACK_I_INVALID_CMD       (0x02)
#define ACK_I_INVALID_CRC       (0x03)
#define ACK_I_VERIFY_ERROR      (0x04)
#define ACK_I_INVALID_CHANNEL   (0x08)
#define ACK_I_INVALID_PARAM     (0x09)
#define ACK_D_GENERAL_ERROR     (0x0F)

#define SIGNAL_PORT GPIOB
#define SIGNAL_PIN LL_GPIO_PIN_4
#define SIGNAL_PIN_IDR GPIO_IDR_ID4_Pos

#define INCOMING_BUFFER (0x45)
#define RESTART_DEVICE (0x46)

#define CMD_RUN             (0x00)
#define CMD_PROG_FLASH      (0x01)
#define CMD_ERASE_FLASH     (0x02)
#define CMD_READ_FLASH_SIL  (0x03)
#define CMD_VERIFY_FLASH    (0x03)
#define CMD_VERIFY_FLASH_ARM (0x04)
#define CMD_READ_EEPROM     (0x04)
#define CMD_PROG_EEPROM     (0x05)
#define CMD_READ_SRAM       (0x06)
#define CMD_READ_FLASH_ATM  (0x07)
#define CMD_KEEP_ALIVE      (0xFD)
#define CMD_SET_ADDRESS     (0xFF)
#define CMD_SET_BUFFER      (0xFE)

#define BAUDRATE 19200
#define UART_BIT_TIME (1000000/BAUDRATE)//52us * 64 cper_us.
#define UART_BIT_TIME_HALF (UART_BIT_TIME>>1)

#define DEVICE_INFO_PACKET_SIZE (0x11)

#define DOUBLEWORD_SIZE (0x08)

#define SIGNAL_LINE_RISE_TIMEOUT (5000)

#define BYTE_TIMEOUT (500) // 500mils
#define ERROR (0xffff)
void wait_clock_cycles(uint32_t cc);
void set_output();
void set_input();
void jump_to_application();
uint16_t receive_buffer(volatile uint8_t* pbuffer , uint16_t buffer_size );
void transmit_packet(volatile uint8_t *buffer, uint16_t size);




#endif /* INC_BOOTLOADER_H_ */
