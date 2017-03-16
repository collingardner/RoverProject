
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/
/*                SETTING THE PINS FOR SENSOR AND ROVOR SHIELD    */
//Pins from the Ultrasonic Sensor
int triggerPin = 2; //triggering on pin 2
int echoPin = 3;    //echo on pin 3

//Pins for the Rover shield
int rightMotorPin = 5;
int leftMotorPin = 6;
int rightDirectionPin = 7;
int leftDirectionPin = 8;
int leftspeed = 255; //setmaximum speed, goes constant speed
int rightspeed = 255;

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/*=====================*/
void setup(void)
{
  /* SETUP for SENSOR and ROVER SHEILD */
  pinMode(triggerPin, OUTPUT); //defining pins
  pinMode(echoPin, INPUT);

  //Shield Code
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightDirectionPin, OUTPUT);
  pinMode(leftDirectionPin, OUTPUT);

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  /* Print Bluefruit information */
  ble.info();
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }
  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

void loop(void)
{ 
  //thiis for calculating the distance objects in cm from the Ultrasonic Sensor
  int duration, distance;    //Adding duration and distance

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  // This if statement waits for the button presses for the controls
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    if (buttnum == 3 ) { //if User presses this button, vehicle goes in self driving mode
      bool selfDrive = true;
      
      while (selfDrive) { //THIS IS IF IT HAS TO TURN AROUND MORE THAN 3 TIMES THE VEHICLE STOPS AND THE USER HAS TO CONTROL IT FROM HERE
        digitalWrite(triggerPin, HIGH); //triggering the wave(like blinking an LED)
        delay(10);
        digitalWrite(triggerPin, LOW);
        duration = pulseIn(echoPin, HIGH); //a special function for listening and waiting for the wave
        distance = (duration / 2) / 29.1; //transforming the number to cm(if you want inches, you have to change the 29.1 with a suitable number
        if (distance < 10 && distance > 0) { //If vehicle within 10 cm of something it stops,
          collisionStop();
          break; // to break out of the big while lop
        }
        else {
          if (distance > 10  || distance < 0) { 
            forward(leftspeed, rightspeed);
          }
          else {
            collisionStop();
          }
        }
      }
    }
    else if (buttnum == 1 ) { //Sets Rover's speed to low
      leftspeed = 200;
      rightspeed = 200;
    }
    else if (buttnum == 2 ) { //Sets the Rover's speed to high
      leftspeed = 255;
      rightspeed = 255;
    }
    else if (buttnum == 5 && pressed == true) {
      forward (leftspeed, rightspeed);
    }
    else if (buttnum == 7 && pressed == true) {
      turnLeft (leftspeed, rightspeed);
    }
    else if (buttnum == 8 && pressed == true ) {
      turnRight (leftspeed, rightspeed);
    }
    else if (buttnum == 6 && pressed == true) {
      reverse (leftspeed, rightspeed);
    }
    else {
      stop();
    }
  }

}
//METHODS FOR TRAVELING
void stop(void) //Stop
{
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
}
void collisionStop(void) //Stop
{
  
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
  delay(10000);
  ble.println("Collision Avoided");
  ble.println(" Rover is stopped, use directional keys to drive, and button 3 for self driving");
  digitalWrite(leftMotorPin, LOW);
  digitalWrite(rightMotorPin, LOW);
}
void forward(char a, char b)
{
  analogWrite (leftMotorPin, a);
  digitalWrite(leftDirectionPin, LOW);
  analogWrite (rightMotorPin, b);
  digitalWrite(rightDirectionPin, LOW);
}

void reverse (char a, char b)
{
  analogWrite (leftMotorPin, a);
  digitalWrite(leftDirectionPin, HIGH);
  analogWrite (rightMotorPin, b);
  digitalWrite(rightDirectionPin, HIGH);
}

void turnLeft(char a, char b)
{
  analogWrite (leftMotorPin, a);
  digitalWrite(leftDirectionPin, HIGH);
  analogWrite (rightMotorPin, b);
  digitalWrite(rightDirectionPin, LOW);
}

void turnRight(char a, char b)
{
  analogWrite (leftMotorPin, a);
  digitalWrite(leftDirectionPin, LOW);
  analogWrite (rightMotorPin, b);
  digitalWrite(rightDirectionPin, HIGH);
}

