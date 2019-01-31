// Poti.h

#ifndef POTI_H
#define POTI_H

class Poti {
private:
	int AnalogPin;
  
	int NumReadings;
	int *readings;
  
	int readIndex = 0;
	double total = 0;

public:
	Poti();

  void attach(const int anAnalogPin);
	int analogRead();
};

#endif /* POTI_H */
