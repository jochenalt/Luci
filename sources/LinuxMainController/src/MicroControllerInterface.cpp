/*
 * MicroControllerInterface.cpp
 *
 *  Created on: 02.03.2015
 *      Author: JochenAlt
 */

#include "MicroControllerInterface.h"

#include "LEDController.h"
#include "unistd.h"
#include "setup.h"


#define DIY_CONTROLLER_BOARD

#ifdef DIY_CONTROLLER_BOARD
#include "I2CInterface.h"

#define SEND_PWM_DATA_REQUEST 10
#define SEND_POWER_REQUEST 11
#define MICROCONTROLLER_I2C_ADDRESSS 4
#endif

void MicroControllerInterface::setup() {
	I2CInterface::getInstance().setup(I2C_PATH, MICROCONTROLLER_I2C_ADDRESSS);
}

void MicroControllerInterface::setServoData(int servo1_us, int servo2_us, int servo3_us,int servo4_us,int servo5_us) {
	servoPWMValue_us[0] = servo1_us;
	servoPWMValue_us[1] = servo2_us;
	servoPWMValue_us[2] = servo3_us;
	servoPWMValue_us[3] = servo4_us;
	servoPWMValue_us[4] = servo5_us;
	dataIsPending = true;
}


void MicroControllerInterface::setLampDuty(int pLampDuty /* 0..100% */) {
	if (pLampDuty != lampDuty) {
		lampDuty  = pLampDuty;
		dataIsPending = true;
	}
}

void MicroControllerInterface::send() {
	uint8_t sendBlock[SERVO_NUMBER*2+2+1]; // 5 servos, 1 lamp, one byte for the checksum
	int sendBlockByte = 0;
	for (int i = 0;i<SERVO_NUMBER;i++) {
		sendBlock[sendBlockByte++] = servoPWMValue_us[i] >> 8;
		sendBlock[sendBlockByte++] = servoPWMValue_us[i] & 0xFF;
	}

	sendBlock[sendBlockByte++] = lampDuty >> 8;
	sendBlock[sendBlockByte++] = lampDuty & 0xFF;

	// compute checksum
	int address = SEND_PWM_DATA_REQUEST;
	int checksum = address;
	for (int i = 0;i<sendBlockByte;i++) {
			checksum ^= sendBlock[i];
	}
	sendBlock[sendBlockByte] = checksum;
	I2CInterface::getInstance().open( I2CInterface::ReadWrite );
	I2CInterface::getInstance().writeBlock(address,sendBlock, sizeof(sendBlock));

	// uint8_t replyBlock[64];
	// myI2c->writeByte(1,0); // request status
	// myI2c->readLine(replyBlock, sizeof(replyBlock));
	I2CInterface::getInstance().close();
	dataIsPending = false;
}

void MicroControllerInterface::power(bool pPowerOn) {
	powerOn = pPowerOn;

	uint8_t sendBlock[2];
	int sendBlockByte = 0;
	sendBlock[sendBlockByte++] = pPowerOn;

	// compute checksum
	int address = SEND_POWER_REQUEST;
	int checksum = address;
	for (int i = 0;i<sendBlockByte;i++) {
		checksum ^= sendBlock[i];
	}
	sendBlock[sendBlockByte] = checksum;
	I2CInterface::getInstance().open( I2CInterface::ReadWrite );
	I2CInterface::getInstance().writeBlock(address,sendBlock, sizeof(sendBlock));

	// uint8_t replyBlock[64];
	// myI2c->writeByte(1,0); // request status
	// myI2c->readLine(replyBlock, sizeof(replyBlock));
	I2CInterface::getInstance().close();

	// send initial PWM data, but wait 1ms to give I2C the time to communicate
	usleep(1000);
	if (!pPowerOn) {
		LEDController::getInstance().setPattern(LEDPattern());
		MicroControllerInterface::getInstance().setLampDuty(0);
	}
	send();
}

