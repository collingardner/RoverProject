//This was code borrwed from the DfRobotShop
// Rover Shield user Guide.  It is basic rover- full speed 
// The code below will cause both motors connected to the onboard H­bridge to rotate at full speed.


int E1 = 6; //M1 Speed Control
int E2 = 5; //M2 Speed Control
int M1 = 8; //M1 Direction Control
int M2 = 7; //M2 Direction Control
void setup()
{
int i;
for(i=5;i<=8;i++)
pinMode(i, OUTPUT);
Serial.begin(9600);
}
void loop()
{
int leftspeed = 255; //255 is maximum speed
int rightspeed = 255;
analogWrite (E1,255);
digitalWrite(M1,LOW);
analogWrite (E2,255);
digitalWrite(M2,LOW);
delay(100);
}

