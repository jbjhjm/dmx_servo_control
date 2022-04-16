#include <Arduino.h>
#include <Servo.h>
#include <DMXSerial.h>

const int DmxTimeoutMs = 		5000;
const int DmxDipPins[9] = 		{ 2,3,4,5,6,7,8,9,10 };

Servo Servo1; 
const int Servo1CtrlPin = 		12; // 12 so that 13 is free for using onboard debug LED hardwired to 13
const int Servo1DefaultPos =	0;
const uint8_t Servo1MinPos = 	0;
const uint8_t Servo1MaxPos = 	180;

const unsigned long MOVING_TIME = 3000; // moving time is 3 seconds
unsigned long moveStartTime;
unsigned long lastServoPosition;
unsigned long targetServoPosition;
int targetInputValue;

const int HelperLedPin = 11; // was only connected during breadboard development !?


//  Can only use either Serial or DMXSerial!

void setup() {
//   Serial.begin(9600);
  DMXSerial.init(DMXReceiver);
  DMXSerial.write(Servo1CtrlPin, Servo1DefaultPos);

  pinMode(Servo1CtrlPin, OUTPUT);	
  Servo1.attach(Servo1CtrlPin,500,2500);

  pinMode(HelperLedPin, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  for(int i = 0; i<9; i++){
    pinMode(DmxDipPins[i], INPUT_PULLUP); // set the digital pins (defined above) as input
  }
}

// cannot use uint8 as DMX uses 9 bits
uint16_t getDipAddress(){
  int i,address=0;
  for(i=1; i<=9; i++){
	  // build address number by left shifting by 1, then merge next bit by OR operation
	  // start from last input which will be the first bit in built int!
    address = (address << 1) | !digitalRead(DmxDipPins[9-i]);
  }
  return address;
}

// Caution: bitOffset starts from 0!
bool getBit(byte data, int bitOffset = 0) {
	// Create a mask by shifting (<<) "1" bit to the left.
	// then use AND (&) to check if the bit exists in both values.
    return (data & (1<<bitOffset)) != 0;
}

void printDipValues() {
//   Serial.print(digitalRead(DmxDipPins[0]));
//   Serial.print(digitalRead(DmxDipPins[1]));
//   Serial.print(digitalRead(DmxDipPins[2]));
//   Serial.print(digitalRead(DmxDipPins[3]));
//   Serial.print(digitalRead(DmxDipPins[4]));
//   Serial.print(digitalRead(DmxDipPins[5]));
//   Serial.print(digitalRead(DmxDipPins[6]));
//   Serial.print(digitalRead(DmxDipPins[7]));
//   Serial.print(digitalRead(DmxDipPins[8]));
//   Serial.print(" (");
//   Serial.print(getDipAddress());
//   Serial.print(")");
//   Serial.print("\r\n");
}

void loop() {
  // // Calculate how long no data packet was received
//   digitalWrite(HelperLedPin,HIGH);
//   delay(500);
//   digitalWrite(HelperLedPin,LOW);
//   delay(500);
  unsigned long lastPacket = DMXSerial.noDataSince();
  uint16_t DmxAddress = getDipAddress();
// //   printDipValues();
  
//   Servo1.write(Servo1MaxPos);  

/**
 *  HelperLedPin will only light up if DMX signal fails.
 *  LED_BUILTIN will light up if DMX signal is > 126 
 */
  
	uint8_t inputValue;
	if (lastPacket < DmxTimeoutMs) { 
		inputValue = DMXSerial.read(DmxAddress);
		digitalWrite(HelperLedPin,LOW);
	} else {
		// Send default value if no package has been received for 5+ seconds
		inputValue = Servo1DefaultPos;
		digitalWrite(HelperLedPin,HIGH);
		// digitalWrite(LED_BUILTIN, LOW);
	}

	if(inputValue != targetInputValue) {
		// changed target value
		moveStartTime = millis();
		targetInputValue = inputValue;
		lastServoPosition = targetServoPosition;
		targetServoPosition = (targetInputValue>=128) ? Servo1MaxPos : Servo1MinPos;
	}
	
	// interp to target value
	// option1: map DMX value range to range  Servo1MinPos Servo1MaxPos
	// value = map(value, 0, 255, Servo1MinPos, Servo1MaxPos);
	// option2: don't map to range, just jump between Min and Max Pos
	// uint8_t newServoPos = (value>=128) ? Servo1MaxPos : Servo1MinPos;
	unsigned long progress = millis() - moveStartTime;
	if (progress <= MOVING_TIME) {
		// move has not finished yet
		long interpPos = map(progress, 0, MOVING_TIME, lastServoPosition, targetServoPosition);
		Servo1.write(interpPos); 
	}

	if(targetInputValue>=128) digitalWrite(LED_BUILTIN,HIGH);
	else digitalWrite(LED_BUILTIN,LOW);

	// digitalWrite(LED_BUILTIN, LOW);



}
