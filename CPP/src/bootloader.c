/*
 * main.cpp
 *
 *  Created on: Dec 29, 2024
 *      Author: Gil Vargas
 *      https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_BLHeli/blheli_4way_protocol.h
 *      https://www.youtube.com/watch?v=WIBSPhffOHI how to run a react app from vscode.
 *	npm start
 *	then debug.
 *      https://github.com/AlkaMotors/g071Bootloader/blob/main/Core/Src/main.c#L644
 *      https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_BLHeli/blheli_4way_protocol.h
 *      https://github.dev/ArduPilot/ardupilot/blob/master/libraries/AP_BLHeli/blheli_4way_protocol.h
 *
 *      https://github.dev/betaflight/betaflight/blob/master/src/main/msp/msp_protocol.h //
 */


#include "main.h"
#include "string.h"

#include "eeprom.h"
#include "bootloader.h"

volatile uint32_t cmd_counter = 0;
volatile uint8_t configurator_connected = 0;

volatile uint16_t flash_address = 0;
volatile uint16_t flash_buffer_len = 0;

volatile uint8_t write_command_expected = 0;


volatile uint8_t incoming_buffer[BUFFER_SIZE] = {0};
volatile uint8_t global_buffer[BUFFER_SIZE] = {0};
volatile uint8_t outgoing_buffer[BUFFER_SIZE] = {0};

uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x56,0x47,0x06,0x06,0x01, 0x30};
//uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x69,0x47,0x06,0x06,0x01, 0x30};

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;
const unsigned short  THROTTLE_DELAY = 300;
eeprom_settings_t eeprom_settings = {.name = "Gil32"
								,.description = "Feature description goes here."
								,.version=3
								,.sub_version = 5
								,.esc_config_layout = 1
								,.commutation_delay = 2
								,.startup_throttle = THROTTLE_DELAY
								,.turtle_rampup = 50
								,.rampup = 0
								,.motor_direction = 1
								,.motor_direction = 0xff
								,.feature1 = 0xff
								,.feature2 = 0xff
								,.feature3 = 0xff
								,.feature4 = 0xff
								,.feature5 = 0xff
								,.feature6 = 0xff
								,.feature7 = 0xff
								,.feature8 = 0xff
								,.feature9 = 0xff
								,.feature10 = 0xff
								,.feature11 = 0xff
								,.feature12 = 0xff
								,.feature13 = 0xff
								,.feature14 = 0xff
								,.feature15 = 0xff
								,.feature16 = 0xff

};


uint8_t calculated_crc_low_byte;
uint8_t calculated_crc_high_byte;
static uint8_16_u CRC_16;

void reset_timer()
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
//	TIM2->PSC = 63;
//	TIM2->ARR = 0xffffffff;
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
//	TIM2->EGR |= 1;
}

static void ByteCrc(uint8_t *bt)
{
    uint8_t xb = *bt;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 ) {
            CRC_16.word = CRC_16.word >> 1;
            CRC_16.word = CRC_16.word ^ 0xA001;
        } else {
            CRC_16.word = CRC_16.word >> 1;
        }
        xb = xb >> 1;
    }
}
void makeCrc(uint8_t* bytes, uint16_t size)
{
    int crc = 0;

    CRC_16.word = 0;
    uint8_t b = 0;
    for (int index = 0; index < size; index++)
    {
    	b = bytes[index];
    	ByteCrc((uint8_t*)&b);
    }
    calculated_crc_low_byte = (uint8_t)((crc >> 8) & 0xff);
    calculated_crc_high_byte = (uint8_t)((crc) & 0xff);

}

void set_input()
{

	SIGNAL_PORT->MODER &= ~(GPIO_MODER_MODE4);
	SIGNAL_PORT->PUPDR &= ~(GPIO_PUPDR_PUPD4_Msk);
//	SIGNAL_PORT->PUPDR |= (GPIO_PUPDR_PUPD4_0);
}
void set_output()
{
	SIGNAL_PORT->MODER |= (GPIO_MODER_MODE4_0);
//	SIGNAL_PORT->PUPDR |= (2<<8);
//	SIGNAL_PORT->PUPDR |= (GPIO_PUPDR_PUPD4_0);
}

uint16_t receive_byte( const uint16_t bytes_received)
{
	volatile uint8_t result = 0;

	TIM2->CNT = 0;
	while( !(SIGNAL_PORT->IDR & SIGNAL_PIN)   )
	{
		if( TIM2->CNT > SIGNAL_LINE_RISE_TIMEOUT)
		{
			return ERROR;
		}
	}

	TIM2->CNT = 0;

	while( (SIGNAL_PORT->IDR & SIGNAL_PIN)  )
	{
		if( TIM2->CNT > UART_BIT_TIME && bytes_received)
		{
			return ERROR;
		}
	}

	wait_clock_cycles(UART_BIT_TIME_HALF);

	for( uint8_t i = 0; i <= 7; i++)
	{
		wait_clock_cycles(UART_BIT_TIME);
		if( (SIGNAL_PORT->IDR & SIGNAL_PIN))
			result |= 1<<i;

	}

	wait_clock_cycles((UART_BIT_TIME_HALF+ 20) );

	return result;
}

uint16_t receive_buffer(volatile uint8_t* buffer , const uint16_t buffer_size )
{
	set_input();

	volatile uint16_t buffer_counter = 0;
	volatile uint16_t result = 0;

	while(1)
	{
		result = receive_byte(buffer_counter);
		if( result != ERROR)
		{
			buffer[buffer_counter] = (uint8_t)result;

			buffer_counter++;
			if( buffer_counter>= buffer_size)
			{
				jump_to_application();
				return ERROR;
			}

		}
		else if( buffer_counter > 0)
		{
			return buffer_counter;
		}
		else
		{
			return ERROR;
		}
	}

	return ERROR;
}

void transmit_byte(char charValue)
{
	char bits = 0;
	SIGNAL_PORT->BRR = SIGNAL_PIN;

	while( bits++ < 8)
	{
		wait_clock_cycles(UART_BIT_TIME);
		SIGNAL_PORT->BSRR = SIGNAL_PIN<<(!(charValue & 1)<<4);

		charValue >>= 1;
	}

	wait_clock_cycles(UART_BIT_TIME);
	SIGNAL_PORT->BSRR = SIGNAL_PIN;
}

void transmit_packet(volatile uint8_t *buffer, uint16_t size)
{
	set_output();

	for( int i = 0; i < size; i++)
	{
		 transmit_byte(buffer[i]);
		 wait_clock_cycles(UART_BIT_TIME );
	}
	set_input();
}

void send_keep_alive_ack()
{
	set_output();
	transmit_byte(0xC1);
	set_input();
}
void send_ack(void)
{
	set_output();
	transmit_byte(ACK);
	set_input();
}
void send_crc_error(void)
{
	set_output();
	transmit_byte(CRC_ERROR);
	set_input();
}
void send_command_error(void)
{
	set_output();
	transmit_byte(COMMAND_ERROR);
	set_input();
}

void send_requested_buffer(void)
{
	memset((uint8_t*) &outgoing_buffer[0], 0, sizeof(outgoing_buffer));

	uint16_t byte_len = incoming_buffer[1];

	read_memory((char*) &outgoing_buffer, byte_len, flash_address);

	makeCrc((uint8_t*)&outgoing_buffer, byte_len);

	outgoing_buffer[byte_len] = CRC_16.bytes[0];
	outgoing_buffer[byte_len + 1] = CRC_16.bytes[1];
	outgoing_buffer[byte_len + 2] = ACK;

	transmit_packet(outgoing_buffer, byte_len + 3);
}

uint16_t process_packet( uint16_t size)
{

	if( size < 3)
	{
		return ERROR;
	}
	uint8_t cmd = incoming_buffer[0];

	uint16_t data_sum = 0;

	makeCrc((uint8_t*)&incoming_buffer[0], size-2);

	if( size == DEVICE_INFO_PACKET_SIZE)
	{
		for( uint8_t index = 0; index < size; index++)
			data_sum += incoming_buffer[index];
	}
	if( data_sum == 910 || data_sum == 1030)
	{
		transmit_packet(deviceInfo, 9);
	}
	else
	{
		if( incoming_buffer[size-1] != CRC_16.bytes[1] || incoming_buffer[size-2] != CRC_16.bytes[0])
		{
		    send_crc_error();
		 	write_command_expected = 0;

		 	return ERROR;
		}

		cmd_counter++;

		if (write_command_expected == 1 && size >= flash_buffer_len) {
			cmd = INCOMING_BUFFER;
		}else if(write_command_expected == 1)
		{
			send_crc_error();
			return ERROR;
		}

		if ((cmd | incoming_buffer[1] | incoming_buffer[2]) == 0) {
			cmd = RESTART_DEVICE;
		}

		write_command_expected = 0;

		switch (cmd)
		{

		case INCOMING_BUFFER: // incoming data to write.

			memcpy((void*) &global_buffer, (const void*) &incoming_buffer, flash_buffer_len);
			send_ack();

			break;

		case RESTART_DEVICE:
			NVIC_SystemReset();
			break;

		case CMD_PROG_FLASH: // program eeprom.

			write_memory((uint8_t*) &global_buffer, flash_buffer_len, flash_address);
			send_ack();

			break;

		case CMD_READ_FLASH_SIL:

			send_requested_buffer();

			break;

		case CMD_KEEP_ALIVE:

			send_ack();
			break;
		case CMD_SET_BUFFER:

			flash_buffer_len = incoming_buffer[2] << 8 | incoming_buffer[3];
			write_command_expected = 1;

			break;
		case CMD_SET_ADDRESS:

			flash_address = ((uint16_t) incoming_buffer[2]) << 8 | (uint16_t) incoming_buffer[3];
			send_ack();

			break;
		default:
			// TODO: anything that lands here needs a handler.
			break;
		}


	}
	return 0;
}

void jump_to_application(){

	typedef void (*pFunction)(void);
	const uint32_t VALID_ENTRY = 0x2000;

	uint32_t jump_address;

	pFunction jump_to_application;

	uint32_t vtor_value = *(__IO uint32_t*) (BASE_ADDRESS + APPLICATION_ADDRESS);

	uint8_t save_id = *(uint8_t*)(BASE_ADDRESS +  EEPROM_ADDRESS);
	if (EEPROM_ID != save_id || (vtor_value>>16) != VALID_ENTRY){
		return;
	}

	__disable_irq();
	jump_address = *(__IO uint32_t*) ((BASE_ADDRESS + APPLICATION_ADDRESS) + 4);
	jump_to_application = (pFunction) jump_address;
    __set_MSP(*(__IO uint32_t*) (BASE_ADDRESS + APPLICATION_ADDRESS));

    jump_to_application();
}

void wait_clock_cycles(const uint32_t cc)
{
	TIM2->CNT = 0;
	while(TIM2->CNT < cc);
}

void init_eeprom(void)
{
	eeprom_settings_t readmem;
	memset((void*) &readmem, 0, sizeof(eeprom_settings_t));

	read_memory((char*) &readmem, sizeof(eeprom_settings_t), EEPROM_ADDRESS);

	if (readmem.name[0] != EEPROM_ID) {
		write_memory((uint8_t*) &eeprom_settings, sizeof(eeprom_settings_t), EEPROM_ADDRESS);
	}
}

void setup(void)
{
//	GPIOB->BRR = LL_GPIO_PIN_8;
//	GPIOB->MODER &= ~(GPIO_MODER_MODE8);
//	GPIOB->MODER |= (GPIO_MODER_MODE8_0);

	SIGNAL_PORT->PUPDR &= ~(GPIO_PUPDR_PUPD4_Msk);
//	SIGNAL_PORT->PUPDR |= (GPIO_PUPDR_PUPD4_1);

	LL_TIM_EnableCounter(TIM2);

	set_output();
	set_input();
}
void process_comms(void)
{
	static volatile uint16_t error_packets = 0;

	volatile uint16_t result = 0;

	memset((void*)&incoming_buffer,0,BUFFER_SIZE);
	result = receive_buffer(incoming_buffer, BUFFER_SIZE);

	if( result != ERROR)
	{
		configurator_connected = 1;
		if( ERROR != process_packet(result) )
		{
			error_packets = 0;
		}
	}

	if( error_packets++ >= 1000)
	{
		if( !configurator_connected)
			jump_to_application();
		error_packets = 0;
		return;
	}


}
void main_program(void)
{
	setup();
	init_eeprom();

	while(1)
	{
		process_comms();
	}

}
