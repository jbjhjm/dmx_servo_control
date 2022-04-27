#include <Arduino.h>
#include <Servo.h>
#include <DMXSerial.h>

const int DmxTimeoutMs = 		5000;
const int DmxDipPins[9] = 		{ 2,3,4,5,6,7,8,9,10 };

Servo Servo1; 
const int Servo1CtrlPin = 		12; // 12 so that 13 is free for using onboard debug LED hardwired to 13
const int Servo1DefaultPos =	0;
const uint8_t Servo1MinPos = 	106;
const uint8_t Servo1MaxPos = 	0;

const unsigned long MOVING_TIME = 1000; // moving time is 3 seconds
unsigned long moveStartTime;
unsigned long moveEndTime;
unsigned long prevServoPosition;
unsigned long targetServoPosition;
int targetInputValue;
float relativeTargetPosition;

const int HelperLedPin = 11; // was only connected during breadboard development !?


//  Can only use either Serial or DMXSerial!

void setup() {
//   Serial.begin(9600);
  DMXSerial.init(DMXReceiver);
  DMXSerial.write(Servo1CtrlPin, Servo1DefaultPos);

  pinMode(Servo1CtrlPin, OUTPUT);	
  Servo1.attach(Servo1CtrlPin,500,2500);

  targetInputValue = -1; // make sure on first data receive targetInputValue and inputValue is different

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

float floatmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return ((float)x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void beginMovement(uint8_t inputValue) {
	// changed target value
	unsigned long now = millis();
	targetInputValue = inputValue;

	if (now <= moveEndTime) {
		// target is being updated while still interpolating
		// need to calculate the current servo position
		prevServoPosition = map(now, moveStartTime, moveEndTime, prevServoPosition, targetServoPosition);
	} else {
		// no interpolation running, we can assume targetServoPosition is current position
		prevServoPosition = targetServoPosition;
	}


	// A/B position
	// targetServoPosition = (targetInputValue>=128) ? Servo1MaxPos : Servo1MinPos;
	// moveEndTime = now + MOVING_TIME;

	// interpolated position
	// force float types, else relativeTargetPosition will always become 0.0!
	float prevRelativeTargetPosition = relativeTargetPosition;
	relativeTargetPosition = (float)targetInputValue / (float)255.0; 
	// map relativeTargetPosition between Servo1 Min & Max Position
	targetServoPosition =  Servo1MinPos + (relativeTargetPosition * (Servo1MaxPos-Servo1MinPos));

	// calculate interpolation time / speed
	// percentage of movement distance compared to Min/Max position range
	float movementDistanceRelative = abs(relativeTargetPosition - prevRelativeTargetPosition);
	moveStartTime = now;
	moveEndTime = moveStartTime + (movementDistanceRelative * MOVING_TIME);
}


void loop() {
	// Calculate how long no data packet was received
	// digitalWrite(HelperLedPin,HIGH);
	// delay(500);
	// digitalWrite(HelperLedPin,LOW);
	// delay(500);
	// printDipValues();

	/**
	 * Receive DMX or use default value
	 *  HelperLedPin will only light up if DMX signal fails.
	 *  LED_BUILTIN will light up if DMX signal is > 126 
	 */
  
	uint8_t inputValue;
	unsigned long lastPacket = DMXSerial.noDataSince();
	uint16_t DmxAddress = getDipAddress();

	if (lastPacket < DmxTimeoutMs) { 
		inputValue = DMXSerial.read(DmxAddress);
		digitalWrite(HelperLedPin,LOW);
	} else {
		// Send default value if no package has been received for 5+ seconds
		inputValue = Servo1DefaultPos;
		digitalWrite(HelperLedPin,HIGH);
		// digitalWrite(LED_BUILTIN, LOW);
	}

	// calculate movement if change was detected
	if(inputValue != targetInputValue) {
		beginMovement(inputValue);
	}
	
	// interpolate to target value
	if (millis() <= moveEndTime) {
		// move has not finished yet
		long interpPos = map(millis(), moveStartTime, moveEndTime, prevServoPosition, targetServoPosition);
		Servo1.write(interpPos); 
		digitalWrite(LED_BUILTIN,HIGH);
	} else {
		// commented out to be 100% sure servo will never receive immedate target value that was not interpolated
		// Servo1.write(targetServoPosition); 
		digitalWrite(LED_BUILTIN,LOW);
	}

	// if(targetServoPosition>0.0) digitalWrite(LED_BUILTIN,HIGH);
	// else digitalWrite(LED_BUILTIN,LOW);

}
