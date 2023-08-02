/**
    Vending Machine for Wall Marble Run

  In this puzzle, players must insert appropriate coins into the machine in order to release a ball
  into the interactive marble run/maze.
*/

// Includes

#include <SPI.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Stepper.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>


// Constants

/* Put the proper code in here for wifi connection and MQTT connection */

// Attach the coin counter to pin 22 and define other things related to the coin counter
const byte coinCounter = 22;

// Number of impulses detected
volatile int impulseCount=0;

// Number of pulses for a coin detected
const int targetPulses = 2;

// Define pins for the Nema17 Stepper motor used for the dispenser
const byte Dispenser1 = 14;
const byte Dispenser2 = 27;
const byte Dispenser3 = 26;
const byte Dispenser4 = 25;

// define pins for the motor used for the escalator
const byte MotorA = 4;
const byte MotorIN1 = 18;
const byte MotorIN2 = 19;

// Initiate the instance of the stepper motor
const int stepsPerRevolution = 200;
Stepper dispenserStepper (stepsPerRevolution, 14, 27, 26, 25);

//Stepper escalatorStepper (stepsPerRevolution, 4, 18, 19, 21);

int IRentrance;
int IRexit;

// DEFINES

// Setting PWM properties

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

#define DEBUG ;

// define IR sensor pins
const int IRentrancePin = 23;
const int IRexitPin = 22;

// Strip of LEDs includes two segments; one for static lighting and one for motion lighting
const int Strip1_Pin = 32;
const int Strip1_NUM_LEDS = 60;

const int Strip1AStart = 0;
const int Strip1BStart = 30;

const int Strip1ALEN = 30;
const int Strip1BLEN = 30;

// Create instance of NeoPixel for the strip of LEDs
Adafruit_NeoPixel Strip1leds(Strip1_NUM_LEDS, Strip1_Pin, NEO_GRB + NEO_KHZ800);

void setup() {
  dispenserStepper.setSpeed(60);
  // put your setup code here, to run once
#ifdef DEBUG
  Serial.begin(9600);
#endif
  // Set all the control pins to outputs and inputs as appropriate.
  pinMode(coinCounter, INPUT);

  // Interrupt connected to pin 22 executing IncomingImpuls function when signal goes from High to Low
  attachInterrupt(digitalPinToInterrupt(coinCounter), incomingImpuls, RISING);

  pinMode(MotorA, OUTPUT);
  pinMode(MotorIN1, OUTPUT);
  pinMode(MotorIN2, OUTPUT);

  pinMode(Dispenser1, OUTPUT);
  pinMode(Dispenser2, OUTPUT);
  pinMode(Dispenser3, OUTPUT);
  pinMode(Dispenser4, OUTPUT);

  // Turn off motors - Initial State
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);

  // configure LED PWM functionalities
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MotorA, pwmChannel);

  // Initialize the NeoPixel strip objects
  Strip1leds.begin();
  Strip1leds.show();
  Strip1leds.setBrightness(75);

  for (int y = 0; y < Strip1_NUM_LEDS; y++) {
    // Cycle the lights blue
    Strip1leds.setPixelColor(y, Strip1leds.Color(0, 0, 255));
  }
  Strip1leds.show();
  delay(500);

  for (int y = 0; y < Strip1_NUM_LEDS; y++) {
    // Cycle the lights off
    Strip1leds.setPixelColor(y, Strip1leds.Color(0, 0, 0));
  }
  Strip1leds.show();
  delay(500);


}

void incomingImpuls()
  {
    impulseCount=impulseCount+1;
  }

void loop() {

  //If we have received a coin, activate stuff
  if (impulseCount >= targetPulses){
    dispenserStepper.step(100);   // Turn the stepper 180 degrees
    impulseCount = 0; // Reset the impulse counter
  }

  // If we haven't received a coin, keep waiting
  else{
  }

  IRentrance=digitalRead(IRentrancePin); 
  if (IRentrance==LOW){
    digitalWrite(MotorIN1, LOW);
    digitalWrite(MotorIN2, HIGH);
  }

  IRexit=digitalRead(IRexitPin); 
  if (IRexit==LOW){
    digitalWrite(MotorIN1, LOW);
    digitalWrite(MotorIN2, LOW);
  }

  
  }

