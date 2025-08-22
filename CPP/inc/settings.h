/*
 * settings.h
 *
 *  Created on: Aug 20, 2025
 *      Author: gilv2
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_
typedef struct eeprom_settings_s{
	char name[12]; // 			= {"Gil32"};		//		= {"Gil32"};
	char description[64]; //	= {"Feature description goes here."};//
	uint8_t version; //	 			= 3;
	uint8_t sub_version; // 		= 5;
	uint8_t esc_config_layout; //	= 1;
	uint8_t commutation_delay; // 	= 64;
	uint16_t startup_throttle; // = THROTTLE_DELAY ;//0x2c01; // 300=0x12c reversed.
	uint8_t turtle_rampup; //  		= 50;
	uint16_t rampup; // 			= 4000
	uint8_t motor_direction_master; //  = 0x1;
	uint8_t motor_direction; // 	= 0xff;
	uint8_t crash_detection; //  	= 0xff;
	uint8_t feature2; //  			= 0xff;
	uint8_t feature3; // 			= 0xff;
	uint8_t feature4; //  			= 0xff;
	uint8_t feature5; //  			= 0xff;
	uint8_t feature6; //  			= 0xff;
	uint8_t feature7; //  			= 0xff;
	uint8_t feature8; //  			= 0xff;
	uint8_t feature9; //  			= 0xff;
	uint8_t feature10; //  			= 0xff;
	uint8_t feature11; //  			= 0xff;
	uint8_t feature12; //  			= 0xff;
	uint8_t feature13; //  			= 0xff;
	uint8_t feature14; //  			= 0xff;
	uint8_t feature15; //  			= 0xff;
	uint8_t feature16; //  			= 0xff;

}__attribute__((packed)) eeprom_settings_t;



#endif /* INC_SETTINGS_H_ */
