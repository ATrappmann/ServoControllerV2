/**
 * Servo-Controller 2.0
 * Copyright (c) 2017-18 by Andreas Trappmann. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * 
 * Arduino Nano based Servo-Controller used in my robotic projects after 
 * installation of new servos. Before starting the designated control software
 * for the whole robot, this sketch on an Arduino Nano with its related Shield
 * allows testing each servo one-by-one for its allowed movement radius. Up to
 * 5 servos are supported. 
 * The status is displayed on a small 20x2 character LCD, showing the current
 * settings with its lower and upper limits of action radius. If the onboard
 * LED is on, servos are ready to move and follow any change in position. By
 * pressing the button ModeControl at any time, all servos stop immediatly and
 * are released from control (no PWM signal any more). This does protect the 
 * robot mechanics, if the servo turns in the wrong direction or is going to
 * move beyond the mechanical boundaries of a joint.
 *
 * Dependencies to 3rd-party libraries:
 * 	- VarSpeedServo - library from https://github.com/netlabtoolkit/VarSpeedServo
 *	- LiquidCrystal - library from http://www.arduino.cc/en/Tutorial/LiquidCrystal
 * 
 * LiquidCrystal 16x2 LCD display, compatible with the Hitachi HD44780 driver
 * The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 8
 * LCD D5 pin to digital pin 7
 * LCD D6 pin to digital pin 4
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor: ends to +5V and ground, wiper to LCD VO pin (pin 3)
 */

// enable/disable features 
//
#define DEBUG	1
//#define HAS_LED 1

#define ON    HIGH
#define OFF   LOW

// library includes
//
#include <LiquidCrystal.h>
#include <VarSpeedServo.h>
#include "Poti.h"

// global constants
//
const int NUM_SERVOS = 4;

// arduino pin definitions
//
#ifdef HAS_LED
const int LedPin = 13;
#endif
const int ModeControlPin = 10;
const int ModeSelectPin = 13;
const int PotiPin[NUM_SERVOS] = { A0, A1, A2, A3 };
const int ServoPin[NUM_SERVOS] = { 3, 5, 6, 9 };
VarSpeedServo servo[NUM_SERVOS];
Poti poti[NUM_SERVOS];

// definition of data types
//
enum OpMode {
	SELECT_MODE = 0,  // on startup only
  DIRECT_MODE,
	LIMITED_MODE,
	NUM_MODES
};

// global variables
//
unsigned long currentMillis;
const unsigned long buttonInterval = 50;
unsigned long previousButtonMillis = 0;
const unsigned long readingInterval = 40;
unsigned long previousReadingMillis = 0;
const unsigned long displayInterval = 10;
unsigned long previousDisplayMillis = 0;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 8, 7, 4, 2);

// state variables of last state
int prevUA = 0;
int prevOA = 180;
int prevVel = 0;
int prevPos = 0;
OpMode prevMode = SELECT_MODE;

/*
** arduino init function at startup
*/
void setup() {
  // setup display
  //
  lcd.begin(16, 2);

	// initialize digital pin modes
	//
	pinMode(ModeControlPin, INPUT);
	pinMode(ModeSelectPin, INPUT);
#ifdef HAS_LED
  pinMode(LedPin, OUTPUT);
#endif

  for (int i=0; i<NUM_SERVOS; i++) {
//    pinMode(PotiPin[i], INPUT);
    poti[i].attach(PotiPin[i]);
  }
  
	// show welcome message
	//
#ifdef HAS_LED
	digitalWrite(LedPin, ON);	// test LED
#endif

#ifdef DEBUG
	Serial.begin(115200);
  Serial.println();
	Serial.println("Servo-Controller 2.0");
	Serial.println("Copyright (c) 2017-18 by Andreas Trappmann");
#endif
	
	lcd.setCursor(0, 0);
	//        "0123456789012345"
	lcd.print("Servo-Control v2");
	lcd.setCursor(0, 1);
	lcd.print("(c) A. Trappmann");
	delay(1000);	

#ifdef HAS_LED
	digitalWrite(LedPin, OFF);
#endif
}

/*
** arduino loop function for operation
*/
void loop() {
	// check buttons
	//
	currentMillis = millis();
	if (currentMillis - previousButtonMillis >= buttonInterval) {		
    previousButtonMillis = currentMillis;
    
		// check for mode change
		//
		int mode = selectMode();
		if (-1 != mode) {
			OpMode selectedMode = (OpMode)mode;
			if (selectedMode != prevMode) {
        initMode(selectedMode);
			  prevMode = selectedMode;
			}
      else {
        initMode(prevMode);
      }
		}
	}
	
	// process selected operation mode
	//
	switch (prevMode) {
		case DIRECT_MODE:
			directMode();
			break;
		case LIMITED_MODE:
			limitedMode();
			break;
	}
}

/*
** main functions of Servo-Controller
*/

void initMode(const OpMode mode) {
  switch (mode) {
    case DIRECT_MODE:
      initDirectMode();
      break;
    case LIMITED_MODE:
      initLimitedMode();
      break;
  }
}

/*
 * selectMode - check if button ModeControl is pressed and allow change of operation mode
 *
 * @return OpMode -1 or new operation mode
 *			-1, if button ModeControl was not pressed
 *			mode, which was selected with button ModeSelect 
 */
int selectMode() {
  if (SELECT_MODE == prevMode) { // on startup
    lcd.setCursor(0, 0);
    //        "0123456789012345"
    lcd.print("Press ModeCtrl  ");
    lcd.setCursor(0, 1);
    lcd.print("to continue...  ");
  }
  
	if (HIGH == digitalRead(ModeControlPin)) {	// Button ModeControl is pressed
		// stop running mode, stop servos!
		//
		switch (prevMode) {
			case DIRECT_MODE:
				stopDirectMode();
				break;
			case LIMITED_MODE:
				stopLimitedMode();
				break;
		}
		
		while (HIGH == digitalRead(ModeControlPin)); // wait until button ModeControl is released
    
		OpMode mode = prevMode;
		do {
			// show current mode
			//
			lcd.setCursor(0, 0);
			//        "0123456789012345"
			lcd.print("Mode Select:    ");
			lcd.setCursor(0, 1);
			switch (mode) {
				case SELECT_MODE:
					//        "0123456789012345"
					lcd.print("direct   limited");
					break;
				case DIRECT_MODE:
					lcd.print("DIRECT   limited");
					break;
				case LIMITED_MODE:
					lcd.print("direct   LIMITED");
					break;
			}

			// check for mode change
			//
			if (HIGH == digitalRead(ModeSelectPin)) {
        
        int m = (int)mode;
				m++;
				if (m >= NUM_MODES) m = 1;
				mode = (OpMode)m;
       
				while (HIGH == digitalRead(ModeSelectPin)); // wait until button ModeSelect is released
			}
		} while (LOW == digitalRead(ModeControlPin)); // continue until button ModeControl is pressed
    while (HIGH == digitalRead(ModeControlPin)); // wait until button ModeControl is released

		return mode;
	}
	else return -1;
}

/*
** Funtions for 'DirectMode' of operation
*/
void initDirectMode() {
#ifdef DEBUG	
	Serial.println("Initializing: DirectMode");
#endif
}

void stopDirectMode() {
#ifdef DEBUG	
	Serial.println("Leaving: DirectMode");
#endif
	
	for (int i=0; i<NUM_SERVOS; i++) {
		servo[i].stop();
		servo[i].detach();
	}
#ifdef HAS_LED
	digitalWrite(LedPin, OFF);
#endif
}

void directMode() {
  // read values and operate servo
  //
  if (currentMillis - previousReadingMillis >= readingInterval) {
    previousReadingMillis = currentMillis;

    if (HIGH == digitalRead(ModeSelectPin)) {
      if (servo[0].attached()) { // stop servos
        for (int i=0; i<NUM_SERVOS; i++) {
          servo[i].stop();
          servo[i].detach();
        }
#ifdef HAS_LED
        digitalWrite(LedPin, OFF);
#endif
      }
      else { // start servos
        for (int i=0; i<NUM_SERVOS; i++) {
          servo[i].attach(ServoPin[i]);
        }
#ifdef HAS_LED
        digitalWrite(LedPin, ON);
#endif
      }
      
      while (HIGH == digitalRead(ModeSelectPin)); // wait for button release
    }
    
    lcd.setCursor(0, 0);
    //        "0123456789012345"
    lcd.print("S1  S2  S3  S4  ");
    lcd.setCursor(0, 1);  
    for (int i=0; i<NUM_SERVOS; i++) {
      int potiValue = poti[i].analogRead(); //analogRead(PotiPin[i]);
      if (-1 == potiValue) continue;
      
      int grad = map(potiValue, 0, 1023, 0, 180);
      
      int len = lcd.print(grad);
      for (int j=len; j<4; j++) lcd.print(" ");

      if (servo[i].attached()) {
          servo[i].write(grad);
      }
    }
    lcd.setCursor(15, 1);
    if (servo[0].attached()) {
      lcd.print("*");
    } else {
      lcd.print(" ");
    }
  }
}

/*
** Functions for 'LimitedMode' of operation
*/
void initLimitedMode() {
#ifdef DEBUG	
	Serial.println("Initializing: LimitedMode");
#endif
}

void stopLimitedMode() {
#ifdef DEBUG
	Serial.println("Leaving: LimitedMode");
#endif
	
	servo[0].stop();
	servo[0].detach();
#ifdef HAS_LED
	digitalWrite(LedPin, OFF);
#endif
}

void limitedMode() {
	// read values and operate servo
	//
	if (currentMillis - previousReadingMillis >= readingInterval) {
		previousReadingMillis = currentMillis;

    if (HIGH == digitalRead(ModeSelectPin)) {
      if (servo[0].attached()) { // stop servo #0
        servo[0].stop();
        servo[0].detach();
#ifdef HAS_LED
        digitalWrite(LedPin, OFF);
#endif
      }
      else { // start servo #0
        servo[0].attach(ServoPin[0]);
        servo[0].write(prevPos, prevVel);
#ifdef HAS_LED
        digitalWrite(LedPin, ON);
#endif
      }
      
      while (HIGH == digitalRead(ModeSelectPin)); // wait for button release
    }

		// read current values of all potentiometers and map them to their value ranges
		//
		int uaValue = poti[0].analogRead(); //analogRead(PotiPin[0]);		// unterer Anschlag
		uaValue = constrain(uaValue, 0, 1023);	// ADC range
		int uA = map(uaValue, 0, 1023, 0, 180);	// map value to servo range
		
		int oaValue = poti[1].analogRead(); //analogRead(PotiPin[1]);		// oberer Anschlag
		oaValue = constrain(oaValue, 0, 1023);	// ADC range
		int oA = map(oaValue, 0, 1023, 0, 180);	// map value to servo range
		
		int velValue = poti[2].analogRead(); //analogRead(PotiPin[2]);		// servo velocity
		velValue = constrain(velValue, 0, 1023);	// ADC range
		int vel = map(velValue, 0, 1023, 0, 255);	// map value to velocity range
		
		int posValue = poti[3].analogRead(); //analogRead(PotiPin[3]);		// servo position
		posValue = constrain(posValue, 0, 1023);	// ADC range
		int pos = map(posValue, 0, 1023, 0, 180);	// map value to servo range
		
#ifdef DEBUG		
		Serial.print("Readings: ua="); Serial.print(uA);
		Serial.print(", oa="); Serial.print(oA);
		Serial.print(", vel="); Serial.print(vel);
		Serial.print(", pos="); Serial.println(pos);
#endif
		
		// check constraints between potentiometer values
		//
		if (uA > prevOA) { // value is to big, ignore reading use previous value
			uA = prevUA;
		}
		if (oA < prevUA) { // value is to low, ignore reading use previous value
			oA = prevOA;
		}
		
		if (uA > oA) { // value is to big, limit value to oA
			uA = oA;
		}
		if (oA < uA) { // value is to low, limit value to uA
			oA = uA;
		}
		
		if (pos < uA) { // value is to low, ignore reading, limit to uA
			pos = uA;
		}
		if (pos > oA) { // value is to big, ignore reading, limit to oA
			pos = oA;
		}
		
#ifdef DEBUG		
		Serial.print("Constrained: ua="); Serial.print(uA);
		Serial.print(", oa="); Serial.print(oA);
		Serial.print(", vel="); Serial.print(vel);
		Serial.print(", pos="); Serial.println(pos);
			
		if ((uA > oA)||(oA < uA)||(pos < uA)||(pos > oA)) {
			Serial.println("ERROR: Constrain Mismatch!");
		} 
#endif
		
		// check velocity
		if (vel > 0) { // do movement
#ifdef DEBUG
			Serial.print("Move servo to pos="); Serial.print(pos);
			Serial.print(", with vel="); Serial.println(vel);
#endif
			
			// move servo to new position
			servo[0].write(pos, vel);	// move with velocity
		}
		else { // stop movement 
#ifdef DEBUG
			Serial.println("Stop movement and detach servo");
#endif			
			servo[0].stop();
		}
		
		// remember the new values
		//
		prevUA = uA;
		prevOA = oA;
		prevVel = vel;
		prevPos = pos;
	}
	
	// display status
	//
	if (currentMillis - previousDisplayMillis >= displayInterval) {
		previousDisplayMillis = currentMillis;
		
		// display settings
		//
		lcd.setCursor(0, 0);
		//        "0123456789012345"
		lcd.print(" uA  oA vel pos ");
		
		lcd.setCursor(0, 1);
		lcdPrintNum(prevUA, 3);
		lcdPrintNum(prevOA, 4);
		lcdPrintNum(prevVel, 4);
		lcdPrintNum(prevPos, 4);
		if (servo[0].attached()) {
			lcd.print("*");
		}
		else lcd.print(" ");
	}
}

/*
** Helper functions
*/

#ifdef DEBUG
void debug(const char *str) {
  Serial.println(str);
}
#else
void debug(const char *str) {}
#endif

void lcdPrintNum(const int value, const int digits) {
	if (value < 10) {
		for (int i=1; i<digits; i++) {
			lcd.print(" ");
		}
	}
	else if (value < 100) {
		for (int i=2; i<digits; i++) {
			lcd.print(" ");
		}
	}
	else if (value < 1000) {
		for (int i=3; i<digits; i++) {
			lcd.print(" ");
		}
	}
	lcd.print(value);
}
