
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

//#define BLUEFRUIT_SWUART_RXD_PIN 10
//#define BLUEFRUIT_SWUART_TXD_PIN 11
//#define BLUEFRUIT_UART_CTS_PIN 12
//#define BLUEFRUIT_UART_RTS_PIN 4
//
//#define BLUEFRUIT_UART_MODE_PIN 13

    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/
/*                SETTING THE PINS FOR SENSOR AND ROVOR SHIELD    */
//Pins from the Ultrasonic Sensor
int triggerPin = 2; //triggering on pin 2
int echoPin = 3;    //echo on pin 3

//Pins for the shield
int lightPin = A0;
int thermoPin = A1;
int rightMotorPin = 5;
int leftMotorPin = 6;
int rightDirectionPin = 7;
int leftDirectionPin = 8;
int leftspeed = 255; //setmaximum speed, goes constant speed
int rightspeed = 255;

int altLeftSpeed = 126;// this will be to put the rover into low speed mode
int altRightSpeed = 126;

//Also will add option of pressing 1, being controlled mode, and pressing 2, being self drving mode

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


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
  pinMode(lightPin, INPUT);
  pinMode(thermoPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightDirectionPin, OUTPUT);
  pinMode(leftDirectionPin, OUTPUT);


  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  //thiis for calculating the distance objects in cm from the Ultrasonic Sensor
 int duration, distance;    //Adding duration and distance
 int turnCount;
 int breakCount = 0;
  digitalWrite(triggerPin, HIGH); //triggering the wave(like blinking an LED)
 
  digitalWrite(triggerPin, LOW);

  
  duration = pulseIn(echoPin, HIGH); //a special function for listening and waiting for the wave
  distance = (duration/2) / 29.1; //transforming the number to cm(if you want inches, you have to change the 29.1 with a suitable number
 
 Serial.print(distance);    //printing the numbers
  Serial.print("cm");       //and the unit
  Serial.println(" ");      //just printing to a new line



//this is where it self drive

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  bool selfDrive;
  bool userDrive;

  
  
 

  //this will stop the vehicle if the distance sensor thinks it will run into something
  //regardless if the buttton on controller is pressed or not
//    if(distance < 10 && distance > 0 ){
//      stop();
//      delay(1000);//delay 1000 ms
//      left (leftspeed, rightspeed);
//      delay(2500);
//    }

  // Buttons
 if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
   // Serial.print ("Button "); Serial.print(buttnum);
   
    //This is to change the speeds of the vehicle

     if(buttnum == 3 ){//dont know if i need if pressed == true
//breakCount = 0; think this needs to be done as well

 while(breakCount < 2){//THIS IS IF IT HAS TO TURN AROUND MORE THAN 3 TIMES THE VEHICLE STOPS AND THE USER HAS TO CONTROL IT FROM HERE

  digitalWrite(triggerPin, HIGH); //triggering the wave(like blinking an LED)
 //  delay(10);
  digitalWrite(triggerPin, LOW);

  
  duration = pulseIn(echoPin, HIGH); //a special function for listening and waiting for the wave
  distance = (duration/2) / 29.1; //transforming the number to cm(if you want inches, you have to change the 29.1 with a suitable number
  
 Serial.print(distance);    //printing the numbers
  Serial.print("cm");       //and the unit
  Serial.println(" ");      //just printing to a new line

//forward(leftspeed,rightspeed);
    if(distance < 10  && distance > 0){
    turnLeft(leftspeed,rightspeed);
    delay(1500); // how ever many milliseconds it takes to turn 90 degrees
    breakCount = breakCount + 1;  //this will just stop it and make the user have to take back control of it
    }
    else if(breakCount > 2 ){
      break;
    }
    else{
      if(distance > 10)
      forward(leftspeed,rightspeed);
    }




  
 }

//  digitalWrite(triggerPin, HIGH); //triggering the wave(like blinking an LED)
// //  delay(10);
//  digitalWrite(triggerPin, LOW);
//
//  
//  duration = pulseIn(echoPin, HIGH); //a special function for listening and waiting for the wave
//  distance = (duration/2) / 29.1; //transforming the number to cm(if you want inches, you have to change the 29.1 with a suitable number
//  
// Serial.print(distance);    //printing the numbers
//  Serial.print("cm");       //and the unit
//  Serial.println(" ");      //just printing to a new line
//
////forward(leftspeed,rightspeed);
//    if(distance < 10  && distance > 0){
//    turnLeft(leftspeed,rightspeed);
//    delay(2500); // how ever many milliseconds it takes to turn 90 degrees
//    }
//    else{
//      if(distance > 10)
//      forward(leftspeed,rightspeed);
//      else
//      stop();
//    }
  }

   else if(buttnum == 1 ){
        leftspeed = 200;
        rightspeed = 200;
    } 
    else if(buttnum ==2 ){
      leftspeed = 255;
      rightspeed = 255;
    }
//    else{
//      //DO NOTHING
//    }

//    if(buttnum == 3){
//      selfDrive = true;
//      userDrive = false;
//    } 
//    else if(buttnum == 4){
//      userDrive = true;
//      selfDrive = false;
//    }
//    else{
//      //selfDrive = false;
//      //userDrive = true;
//    }

//Make Big if controlled do the below code, elseif button = 1,pressed == true, and released == true, for selfdriving buttong pressed, else = stop around this
//Serial.print("selfdrive");
//Serial.print(selfDrive);
//Serial.print("userDrive");
//Serial.print(userDrive); 
  else if (buttnum == 5 && pressed == true) {
     forward (leftspeed, rightspeed);
     turnCount = 0;
    }
   else if (buttnum == 7 && pressed == true) {
     turnLeft (leftspeed, rightspeed);
     turnCount = 0;
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

