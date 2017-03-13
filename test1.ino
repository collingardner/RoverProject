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
int leftspeed = 255; //255 is maximum speed
int rightspeed = 255;



void setup() {
   Serial.begin(9600);
  
  pinMode(triggerPin, OUTPUT); //defining pins
  pinMode(echoPin, INPUT);

  //Shield Code
  pinMode(lightPin, INPUT);
  pinMode(thermoPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightDirectionPin, OUTPUT);
  pinMode(leftDirectionPin, OUTPUT);

}

void loop() {
  //Ultrasonic Sensor Code
  int duration, distance;    //Adding duration and distance
  
  digitalWrite(triggerPin, HIGH); //triggering the wave(like blinking an LED)
  delay(10);
  digitalWrite(triggerPin, LOW);
  
  duration = pulseIn(echoPin, HIGH); //a special function for listening and waiting for the wave
  distance = (duration/2) / 29.1; //transforming the number to cm(if you want inches, you have to change the 29.1 with a suitable number

  //Rover Shield

  if(distance < 10){
    left (leftspeed,rightspeed);
    delay(2500); // how ever many milliseconds it takes to turn 90 degrees
    }
  else{
    forward (leftspeed, rightspeed);
  }
  
 // forward (leftspeed, rightspeed); //Go Forward
 // delay(2500);
//  left (leftspeed, rightspeed); // Turn Left
//  delay(2500);
//  reverse (leftspeed, rightspeed); // Go Backward
//  delay(2500);
//  right (leftspeed, rightspeed); // Turn Right
//  delay(2500);
//  stop(); // Full Stop
//  delay(2500);

  Serial.print(distance);    //printing the numbers
  Serial.print("cm");       //and the unit
  Serial.println(" ");      //just printing to a new line

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

void left (char a, char b)
{
  analogWrite (leftMotorPin, a);
  digitalWrite(leftDirectionPin, HIGH);
  analogWrite (rightMotorPin, b);
  digitalWrite(rightDirectionPin, LOW);
}

void right (char a, char b)
{
  analogWrite (leftMotorPin, a);
  digitalWrite(leftDirectionPin, LOW);
  analogWrite (rightMotorPin, b);
  digitalWrite(rightDirectionPin, HIGH);
}


