// Poti.cxx

#include "Poti.h"
#include <Arduino.h>

Poti::Poti() {
  AnalogPin = -1;
}

void Poti::attach(const int anAnalogPin) {
	AnalogPin = anAnalogPin;
	pinMode(AnalogPin, INPUT);
	
	NumReadings = 10;
	readings = new int[NumReadings];
	for (int i=0; i<NumReadings; i++) {
		readings[i] = 0;
	}
  total = 0;
  readIndex = 0;
}
		
int Poti::analogRead() {
  if (-1 == AnalogPin) return -1;
  
	int newValue = ::analogRead(AnalogPin);	// read new value
  newValue = constrain(newValue, 0, 1023);
  
  total = total - readings[readIndex];
	readings[readIndex] = newValue;
	total = total + readings[readIndex];
	readIndex = readIndex + 1;

	if (readIndex >= NumReadings) {
		readIndex = 0; // wrap around to the beginning
	}
	
	double average = total / NumReadings;
	return average;
}
