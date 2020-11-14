#include <Arduino.h>
#include <Servo.h>
#include <DMXSerial.h>

const int DmxTimeoutMs = 		5000;
const int DmxDipPins[9] = 		{ 3,4,5,6,7,8,9,10,11 };

Servo Servo1; 
const int Servo1CtrlPin = 		9;
const int Servo1DefaultPos =	0;
const int Servo1MinPos = 		0;
const int Servo1MaxPos = 		200;

void setup() {
  DMXSerial.init(DMXReceiver);
  DMXSerial.write(Servo1CtrlPin, Servo1DefaultPos);

  pinMode(Servo1CtrlPin, OUTPUT);	
  Servo1.attach(Servo1CtrlPin);

  for(int i = 0; i<9; i++){
    pinMode(DmxDipPins[i], INPUT);      // set the digital pins (defined above) as input
    digitalWrite(DmxDipPins[i], HIGH);  // set internal pullup resistor on
  }
}

void loop() {
  // Calculate how long no data backet was received
  unsigned long lastPacket = DMXSerial.noDataSince();
  uint16_t DmxAddress = getDipAddress();
  
  if (lastPacket < DmxTimeoutMs) {
	uint8_t value = DMXSerial.read(DmxAddress);
	// option1: map DMX value range to range  Servo1MinPos Servo1MaxPos
	// value = map(value, 0, 255, Servo1MinPos, Servo1MaxPos);
	// option2: don't map to range, just jump between Min and Max Pos
	value = (value>=128) ? Servo1MaxPos : Servo1MinPos;
  	Servo1.write(value);
  } else {
    // Send default value if no package has been received for 5+ seconds
  	Servo1.write(Servo1DefaultPos);
  }
}

// cannot use uint8 as DMX uses 9 bits
uint16_t getDipAddress(){
  int i,address=0;
  for(i=0; i<9; i++){
	  // build address number by left shifting by 1, then merge next bit by OR operation
    address = (address << 1) | digitalRead(DmxDipPins[i]);
  }
  return address;
}