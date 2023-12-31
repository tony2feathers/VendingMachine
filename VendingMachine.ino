/**
    Vending Machine for Wall Marble Run

  In this puzzle, players must insert appropriate coins into the machine in order to release a ball
  into the interactive marble run/maze.


// Includes
#include <SPI.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Stepper.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "esp_secrets.h"

// Constants
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;  // the WiFi radio's status

const char* mqtt_server = "10.1.10.55";

WiFiClient espClient;
PubSubClient client(espClient);

// I think these are remnants of old code??
long lastMsg = 0;
char msg[50];
int value = 0;

const long interval = 1000;
unsigned long previousMillis = 0;

// constants for MQTT
const char topic[] = "ToDevice/VendingMachine";
const char hostTopic[] = "ToHost/VendingMachine";
const char* deviceID = "VendingMachine";

int count = 0;

// Variable to count the balls currently in the escalator
int ballReturn = 0;

// Attach the coin counter to pin 34 and define other things related to the coin counter
const byte coinCounter = 34;

// Number of impulses detected
volatile int impulseCount = 0;

// Number of pulses for a coin detected
const int targetPulses = 1;

// Define pins for the Nema17 Stepper motor used for the dispenser
const byte Dispenser1 = 14;
const byte Dispenser2 = 27;
const byte Dispenser3 = 26;
const byte Dispenser4 = 25;

// define pins for the motor used for the escalator
const byte MotorA = 4;
const byte MotorIN1 = 19;
const byte MotorIN2 = 18;

// define pins for the linear actuator
const byte MotorB = 16;
const byte MotorIN3 = 5;
const byte MotorIN4 = 17;

// Initiate the instance of the stepper motor
const int stepsPerRevolution = 200;
Stepper dispenserStepper(stepsPerRevolution, 14, 27, 26, 25);

// DEFINES

// Setting PWM properties to control motor speed
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

#define DEBUG;

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

//************************INTERRUPT FUNCTIONS************************
void incomingImpuls() {
  impulseCount = impulseCount + 1;
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Setup the wifi and MQTT connections
  wifiSetup();
  MQTTsetup();
  Serial.print("Connected to WIFI and MQTT!");

  // Set the speed of the dispenser stepper motor
  dispenserStepper.setSpeed(60);

  // Set all the control pins to outputs and inputs as appropriate.
  pinMode(coinCounter, INPUT);
  pinMode(IRentrancePin, INPUT);
  pinMode(IRexitPin, INPUT);

  // Interrupt connected to pin 22 executing IncomingImpuls function when signal goes from High to Low
  attachInterrupt(digitalPinToInterrupt(coinCounter), incomingImpuls, FALLING);

  pinMode(MotorA, OUTPUT);
  pinMode(MotorIN1, OUTPUT);
  pinMode(MotorIN2, OUTPUT);

  pinMode(MotorB, OUTPUT);
  pinMode(MotorIN3, OUTPUT);
  pinMode(MotorIN4, OUTPUT);

  pinMode(Dispenser1, OUTPUT);
  pinMode(Dispenser2, OUTPUT);
  pinMode(Dispenser3, OUTPUT);
  pinMode(Dispenser4, OUTPUT);

  // Turn off motors - Initial State
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);

  // configure LED PWM functionalities
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MotorA, pwmChannel);

  // Initialize the NeoPixel strip objects
  Strip1leds.begin();
  Strip1leds.show();
  Strip1leds.setBrightness(75);

  for (int x = 0; x < Strip1_NUM_LEDS; x++) {
    // Cycle the lights blue
    Strip1leds.setPixelColor(x, Strip1leds.Color(0, 0, 255));
  }
  Strip1leds.show();
  delay(500);

  for (int y = 0; y < Strip1_NUM_LEDS; y++) {
    // Cycle the lights off
    Strip1leds.setPixelColor(y, Strip1leds.Color(0, 0, 0));
  }
  Strip1leds.show();
  delay(500);
  Serial.println("LED's tested, turning on ambient lights!");

int z = 0;
for (int z = Strip1BStart; z < (Strip1BStart + Strip1BLEN); z++) {
  // Activate the top strip of lights for "ambient lighting"
  Strip1leds.setPixelColor(z, Strip1leds.Color(204, 85, 0));
}
}

void loop() {
  // Check if there is any data available in the Serial Monitor
  /*if (Serial.available()) {
    String serialData;
    // Read the incoming data and append it to the serialData variable
    char c = Serial.read();
    serialData += c;

    // Check if the incoming data ends with a newline character, indicating the end of a message
    if (c == '\n') {
      // Publish the data to the MQTT broker
      serialData.trim(); // Remove leading and trailing whitespace
      Serial.print("Publishing message to topic: ");
      Serial.println(topic);
      Serial.print("Message: ");
      Serial.println(serialData);
      client.publish(topic, serialData.c_str());

      // Clear the serialData variable to prepare for the next message
      serialData = "";
    }
  }

  //If we have received a coin, activate stuff
  if (impulseCount >= targetPulses) {
    onDispense();
  }

  // If we haven't received a coin, keep waiting
  else {
  }

  int IRentrance = digitalRead(IRentrancePin);
  if (IRentrance == HIGH) {
    ballReturn++;
    cylonStrip1ATrail(Strip1leds.Color(0, 119, 178), 50, 10);
    digitalWrite(MotorIN1, LOW);
    digitalWrite(MotorIN2, HIGH);
    client.publish(hostTopic, "Ball entered escalator");
  }
  int IRexit = digitalRead(IRexitPin);
  if (IRexit == HIGH) {
    ballReturn--;
    client.publish(hostTopic, "Ball exited escalator");
  }
  if (ballReturn == 0) {
    digitalWrite(MotorIN1, LOW);
    digitalWrite(MotorIN2, LOW);
  }

  delay(500);
  client.loop();
}

//************************WIFI & MQTT FUNCTIONS************************
void wifiSetup() {
  Serial.println();
  Serial.println("****************************");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP Address: ");
  Serial.print(WiFi.localIP());
}

void MQTTsetup() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(topic);
  while (!client.connected()) {

    // Debug info
    Serial.print("Attempting to connect to the MQTT broker at ");
    Serial.println(mqtt_server);

    // Attempt to connect
    if (client.connect(deviceID)) {

      // Debug info
      Serial.println("Connected to MQTT broker");

      // Once connected, publish an announcement to the host
      client.publish(hostTopic, "Vending Machine Connected!");
      // Subscribe to topics meant for this device
      client.subscribe(topic);
      Serial.println("Subscribed to topic: ");
      Serial.println(topic);
    } else {
      // Debug info
      Serial.print("Failed to connect to MQTT broker, rc =");
      Serial.print(client.state());
      Serial.println("Retrying in 5 seconds...");
      // Wait 5 seconds before retrying
      delay(500);
    }
  }
}

void callback(char* thisTopic, byte* message, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(thisTopic);
  Serial.print("] ");
  Serial.println("Message: ");
  
  // Convert byte array to C-style string
  char messageArrived[length + 1];
  memcpy(messageArrived, message, length);
  messageArrived[length] = '\0'; // Null-terminate the string
 
  for (unsigned int i = 0; i < length; i++) {
    messageArrived[i] = tolower(messageArrived[i]);
  }

  // Use strcmp to look for certain messages
  if (strcmp(messageArrived, "dispense") == 0){
    onDispense();
  } else if (strcmp(messageArrived, "reverse") == 0){
    onReverse();
  } else if (strcmp(messageArrived, "stop") == 0){
    onStop();
  } else if (strcmp(messageArrived, "open") == 0){
    onOpen();
  } else if (strcmp(messageArrived, "close") == 0){
    onClose();
  }
  Serial.println(messageArrived);
  Serial.println();
}

//************************MQTT MESSAGE FUNCTIONS************************
void onDispense() {
  dispenserStepper.step(100);  // Turn the stepper 180 degrees
  impulseCount = 0;            // Reset the impulse counter
  Serial.println("Ball dispense activated. Players should now have a ball in the marble run!");
  client.publish(hostTopic, "Ball dispense activated. Players should now have a ball in the marble run!");
}

void onStop() {
  // Force all motors to come to a stop immediately
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  Serial.print("All motor stop command received!");
  client.publish(hostTopic, "All motor stop command received!");
}

void onOpen() {
  // Activate the linear actuator for 5 seconds
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, HIGH);
  delay(500);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  Serial.print("Coin OPEN command received");
  client.publish(hostTopic, "Coin OPEN command received");
}

void onClose() {
  // Activate the linear actuator in the other direction for 5 seconds
  digitalWrite(MotorIN3, HIGH);
  digitalWrite(MotorIN4, LOW);
  delay(500);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  Serial.print("Coin CLOSE command received");
  client.publish(hostTopic, "Coin CLOSE command received");
}

void onReverse() {
  digitalWrite(MotorIN1, HIGH);
  digitalWrite(MotorIN2, LOW);
  Serial.print("Reverse motor command received");
  client.publish(hostTopic, "Reverse motor command received");
  delay(500);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, HIGH);
}

//************************LIGHTING FUNCTIONS************************
void cylonStrip1ATrail(uint32_t c, uint8_t wait, uint8_t reps) {
  for (int j = reps; reps > 0; reps--) {
    int i = 0;
    for (i = Strip1AStart; i <= Strip1AStart + Strip1ALEN; i++) {
      Strip1leds.setPixelColor(i, c);
      faderToBlackStrip1A();
      Strip1leds.show();
      delay(wait);
    }
    int x = 0;
    for (x = Strip1AStart; x <= Strip1AStart + Strip1ALEN; x++) {
      Strip1leds.setPixelColor(x, Strip1leds.Color(0, 0, 0));
      Strip1leds.show();
    }
  }
}
void faderToBlackStrip1A() {
  for (int j = Strip1AStart; j < Strip1AStart + Strip1ALEN; j++) {
    fadeToBlackStrip1A(j, 64);
  }
}

void fadeToBlackStrip1A(int ledNo, byte fadeValue) {
  // NeoPixel
  uint32_t oldColor;
  uint8_t r, g, b;
  int value;

  oldColor = Strip1leds.getPixelColor(ledNo);
  r = (oldColor & 0x00ff0000UL) >> 16;
  g = (oldColor & 0x0000ff00UL) >> 8;
  b = (oldColor & 0x000000ffUL);

  r = (g <= 10) ? 0 : (int)r - (g * fadeValue / 256);
  g = (r <= 10) ? 0 : (int)g - (r * fadeValue / 256);
  b = (b <= 10) ? 0 : (int)b - (b * fadeValue / 256);

  Strip1leds.setPixelColor(ledNo, r, g, b);
}*/