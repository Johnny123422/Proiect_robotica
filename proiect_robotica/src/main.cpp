#include <Arduino.h>
#include <Servo.h>
#include <AFMotor.h>

#define Echo 14 // A0 
#define Trig 15 // A1
#define motor 10
#define Speed 120
#define Speed_Cot 200
#define spoint 85

int distance;
int Left;
int Right;
int L = 0;
int R = 0;
int L1 = 0;
int R1 = 0;

Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);
AF_DCMotor M5(1);
AF_DCMotor M6(2);
AF_DCMotor M7(3);
AF_DCMotor M8(4);

int ultrasonic()
{

  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  long t = pulseIn(Echo, HIGH);

  long cm = t / 29 / 2; // convertim timpul în distanță (cm)
  return cm;
}
void forward()
{
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void backward()
{
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void right()
{
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M7.run(BACKWARD);
  M8.run(BACKWARD);
}
void left()
{
  M5.run(FORWARD);
  M6.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void Stop()
{
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

int rightsee()
{
  servo.write(0);
  delay(800);
  Left = ultrasonic();
  return Left;
}
int leftsee()
{
  servo.write(180);
  delay(800);
  Right = ultrasonic();
  return Right;
}

void avoid_obstacle()
{
  distance = ultrasonic();
 

  if (distance <= 12)
  {
    Stop();
    backward();
    delay(500);
    Stop();
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R)
    {
      right();
      delay(1000); 
      Stop();
      delay(200);
    }
    else
    {
      left();
      delay(1000); 
      Stop();
      delay(200);
    }
  }
  else
  {
    forward();
  }
}
 
void setup()
{


   Serial.begin(9600);

  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);

  servo.attach(motor);
  servo.write(spoint); 

  Stop(); 

  
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
  M5.setSpeed(Speed_Cot);
  M6.setSpeed(Speed_Cot);
  M7.setSpeed(Speed_Cot);
  M8.setSpeed(Speed_Cot);
}


void loop()
{
   avoid_obstacle();

   Serial.print("Distanță măsurată: ");
  Serial.println(distance);  
}