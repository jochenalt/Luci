/*
 * main.cpp
 *
 * Created: 21.11.2014 22:32:08
 *  Author: JochenAlt
 */ 

#include "Arduino.h"
#include "HardwareSerial.h"
#include "Servo.h"
#include <Wire.h>

#include <avr/wdt.h>

#include "TimePassedBy.h"

#define LAMP_PIN PIN_D3
#define SERVO_POWER_STARTUP_PIN PIN_D6
#define SERVO_POWER_ON_PIN PIN_D7
#define SETUP_WEBCAM_BYPASS_PIN PIN_D4

#define INITIAL_BAUD_RATE 115200
#define SERVO_DUE_TIME (REFRESH_INTERVAL/1000)
#define SERVO_NO 5
#define SET_PWM_CMD_ADDRESS 10
#define SET_POWER_ADDRESS 11
#define INIT_WEBCAM_ADDRESS 12

#define I2C_BUS_ADDRESS 4

uint8_t request[32];
uint8_t requestLen = 0;
volatile boolean requestPending= false;
volatile boolean responsePending = false;
uint8_t response[32];
uint8_t responseLen = 0;
bool debug = false;

Servo servo[SERVO_NO];
int servoDuty[SERVO_NO] = {0,0,0,0,0};
int lastServoDuty[SERVO_NO] = {0,0,0,0,0};
bool isPowerOn = false;
bool startupProcedure = false;
uint8_t startupPhase = 0;

uint32_t startUpTime;
volatile boolean newServoDutyReceived = false;
uint8_t servoPin[SERVO_NO] = {PIN_C1,  PIN_C2, PIN_C3, PIN_C0, PIN_B2   }; // , PIN_D6, PIN_B1, PIN_B2/*, PIN_B3*/  };
TimePassedBy servoTimer;

#define SERVO_START_TIME 500
#define SERVO_START_FULL_THROTTLE_TIME 2800
#define SERVO_STARTUP_TIME 3000

int lampDuty = 0; // in percent

void writeMultiMicroseconds(Servo pServo[MAX_SERVOS], int pValue[MAX_SERVOS], uint8_t pServoNo);

void setError(const __FlashStringHelper* msg) {
	// ErrorLEDBlinker.setOneTime(ErrorBlinkPattern,sizeof(ErrorBlinkPattern));
	Serial.print("§");
	Serial.println(msg);
}

void setStartUp() {
	pinMode(SERVO_POWER_STARTUP_PIN, OUTPUT);
	digitalWrite(SERVO_POWER_STARTUP_PIN, LOW);
}

void unsetStartUp() {
	pinMode(SERVO_POWER_STARTUP_PIN, INPUT_PULLUP);
}
String pinName(uint8_t pin) {
	switch (pin) {
		case PIN_D0: return "D0";
		case PIN_D1: return "D1";
		case PIN_D2: return "D2";
		case PIN_D3: return "D3";
		case PIN_D4: return "D4";
		case PIN_D5: return "D5";
		case PIN_D6: return "D6";
		case PIN_D7: return "D7";

		case PIN_B0: return "B0";
		case PIN_B1: return "B1";
		case PIN_B2: return "B2";
		case PIN_B3: return "B3";
		case PIN_B4: return "B4";
		case PIN_B5: return "B5";

		case PIN_C0: return "C0";
		case PIN_C1: return "C1";
		case PIN_C2: return "C2";
		case PIN_C3: return "C3";
		case PIN_C4: return "C4";
		case PIN_C5: return "C5";
		default:
		return "";
	}
}


void setPwmLampDuty(int pDuty) {
	if (pDuty != lampDuty) {
		int duty = 255-pDuty;
		/*
		Serial.print("led=");
		Serial.print(pDuty);	
		Serial.print("%");
		Serial.print("1=");
		Serial.print((pDuty*256)/100);
		Serial.print("==");
		Serial.println(duty);
		*/
		analogWrite(LAMP_PIN,duty);
	}
	lampDuty = pDuty;
}



// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveI2CEvent(int howMany)
{
	requestLen = 0;
	while(0 < Wire.available()) // loop through all but the last
	{
		char c = Wire.read(); // receive byte as a character
		request[requestLen++] = c;
	}
	requestLen--;
	requestPending = true;	
}

void requestI2CReply(){
	if (responsePending) {
		Wire.write(response, responseLen);
		responsePending = false;
	}
}

void setWatchDog() {
	wdt_enable(WDTO_250MS);	
}

void setup()
{
	setWatchDog();
	Serial.begin(INITIAL_BAUD_RATE);           // start serial for output

	for (int i = 0;i<SERVO_NO;i++) {				// initialize but with no signal yet
		servo[i].attach(servoPin[i]);
		servo[i].writeMicroseconds(0);
	}
					
	Wire.begin(I2C_BUS_ADDRESS);                // join i2c bus with address #4
	Wire.onReceive(receiveI2CEvent);			// register function for receiving messages
	Wire.onRequest(requestI2CReply);			// register function for replying to messages

	unsetStartUp();
	pinMode(SERVO_POWER_ON_PIN, OUTPUT);
	digitalWrite(SERVO_POWER_ON_PIN, HIGH);
	setPwmLampDuty(0); // initially switch lamp off 

	// switch webcam off
	pinMode(SETUP_WEBCAM_BYPASS_PIN, INPUT);
	
	Serial.print("luci's servo pins(");
	for (int i = 0;i<SERVO_NO;i++) {
		if (i>0)
			Serial.print(",");
		switch (i) {
			case 0: Serial.print("base-turn"); break;
			case 1: Serial.print("leg"); break;
			case 2: Serial.print("arm"); break;
			case 3: Serial.print("head-nick"); break;
			case 4: Serial.print("head-turn"); break;
		}
		Serial.print("="); 
		Serial.print(pinName(servoPin[i]));
	}
	Serial.print(") ");
	Serial.print(" servo on[low]=");
	Serial.print(pinName(SERVO_POWER_STARTUP_PIN));
	Serial.print(" servo low power[low]=");
	Serial.print(pinName(SERVO_POWER_ON_PIN));
	Serial.print(" lamp[low]=");
	Serial.print(pinName(LAMP_PIN));
	Serial.print(" webcam[low]=");
	Serial.print(pinName(SETUP_WEBCAM_BYPASS_PIN));
	Serial.println();
	

}

void switchPower(bool on) {
	if (on) {
		if (!isPowerOn) {
			unsetStartUp();
			startupProcedure = true;
			startUpTime = millis();
			isPowerOn = true;
			startupPhase = 0;
			// dont send servo impulses in startup procedure
			for (int i = 0;i<SERVO_NO;i++) {
				servo[i].writeMicroseconds(0);
			}
		}
	} else {
		if (isPowerOn) {
			setStartUp();
			digitalWrite(SERVO_POWER_ON_PIN, HIGH);
			startupProcedure = false;
			isPowerOn = false;
			setPwmLampDuty(0);
		}
	} 
}
void startupLoop() {
	if (startupProcedure) {

		uint32_t now = millis();
		if (now-startUpTime <SERVO_START_TIME) {
			if (startupPhase == 0) {
				// switch off everything until voltage is stable
				setStartUp();
				
				digitalWrite(SERVO_POWER_ON_PIN, LOW);
				setPwmLampDuty(now); // lamps goes up within 100 ms
				Serial.print("waiting ... ");
				startupPhase = 1;
			}
		} else if (now-startUpTime < SERVO_START_FULL_THROTTLE_TIME) {
			setPwmLampDuty((255*((now-startUpTime)-SERVO_START_TIME))/(SERVO_START_FULL_THROTTLE_TIME-SERVO_START_TIME)); // lamps goes slowly on within that phase
			if (startupPhase == 1) {
				// switch on the servo relay with low voltage for startup
				setStartUp();
				digitalWrite(SERVO_POWER_ON_PIN, LOW);

				Serial.print("low power ... ");
				startupPhase = 2;
			}
		}
		else if (now -startUpTime< SERVO_STARTUP_TIME) {
			setPwmLampDuty(255-(((now-startUpTime)-SERVO_START_FULL_THROTTLE_TIME)*100)/(SERVO_STARTUP_TIME-SERVO_START_FULL_THROTTLE_TIME)); // lamps goes off within that phase
			if (startupPhase == 2) {
				// relay remains on, go to full power supply
				unsetStartUp();
				digitalWrite(SERVO_POWER_ON_PIN, LOW);
				startupPhase = 3;
				Serial.println(" full power.");

			}
		} else {
			startupProcedure = false;
			setPwmLampDuty(0);
		}
	}
}


extern bool pulseOn; // set in servo library
void loop()
{
	wdt_reset();

	// process startup phase when necessary
	startupLoop();
		
	// process communication requests 
	if (requestPending) {
		uint8_t checksum = 0;
		for (int j = 0;j<requestLen;j++) {
			checksum ^= request[j];
		}

		if (checksum != request[requestLen]) {
			Serial.print("request[");
			Serial.println(requestLen);
			Serial.print("]:invalid checksum ");
			Serial.println(checksum);
			Serial.print("!=");
			Serial.println(request[requestLen]);
		}
		
		uint8_t address = request[0];
		if (address == SET_PWM_CMD_ADDRESS) {
			uint8_t i = 0;
			for (;i<SERVO_NO;i++) {
				int duty =  (request[1+i*2] << 8)+ request[1+i*2+1];
				servoDuty[i] = duty;
			}
			// grab lamp out of request
			uint16_t lampDuty =  (request[1+i*2] << 8)+ request[1+i*2+1];
			setPwmLampDuty(lampDuty);
			i++;
			
			newServoDutyReceived = true;
			if (debug) {
				Serial.print("set pwm");
				Serial.print("(");
				for (uint8_t i = 0;i<SERVO_NO;i++) {
					Serial.print(servoDuty[i]);
					Serial.print(",");
				} 
				Serial.print(lampDuty);
				Serial.println(")");
			}
		} else if (address == SET_POWER_ADDRESS) {
			bool switchOn = request[1];
				if (!switchOn)
					Serial.println("set power off");
				else
					Serial.print("prepare power on ...");
			switchPower(switchOn);
		} else if (address == INIT_WEBCAM_ADDRESS) {
			Serial.println("init webcam ...");
			// switch webcam off
			pinMode(SETUP_WEBCAM_BYPASS_PIN, OUTPUT);
			digitalWrite(SETUP_WEBCAM_BYPASS_PIN,HIGH);
			wdt_enable(WDTO_1S);	
			delay(150); // time for the relay to switch off webcam
			digitalWrite(SETUP_WEBCAM_BYPASS_PIN,LOW);
			pinMode(SETUP_WEBCAM_BYPASS_PIN, INPUT);
			setWatchDog();
		}
		else {
			Serial.print("unknown request type");
			Serial.println(address);
			Serial.print("# ");
			for (uint8_t i = 0;i<requestLen;i++) {
				Serial.print(i);
				Serial.print("=");
				Serial.print((int)request[i]);
				Serial.print(" ");
			}
			Serial.println();
		}
		
		requestPending = false;
	}

	// wait until current pulse is over before setting the new duty (which produces an interrupt)
	if (!pulseOn && newServoDutyReceived) {
		if (servoTimer.isDue_ms(SERVO_DUE_TIME)) {
		/*
		for (uint8_t i = 0;i<SERVO_NO;i++) {
			int motor_us = constrain(servoDuty[i],MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
			if (servoDuty[i] != 0 && (abs(lastServoDuty[i] - servoDuty[i]))>1) {
				servo[i].writeMicroseconds(motor_us);
				lastServoDuty[i] = servoDuty[i];
			}
		}
		*/
		// to avoid too many cli(), do all write operations in one critical block
		writeMultiMicroseconds(servo,servoDuty,SERVO_NO);
		newServoDutyReceived = false;
		}
	}
}

		
