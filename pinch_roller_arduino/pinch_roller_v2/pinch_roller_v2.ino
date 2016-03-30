//Pinch roller motor driver by N. Seymour-Smith
//This version offloads feedback calculations to the microscope computer

#include <PWM.h>

//Digital Pins
int motor_spoolerPin = 9;     //PWM pin for spooler motor

//Variables for puller speed control
int puller_speed = 125;


void setup ()
{
  Serial.begin (115200);
  while(!Serial);
  Serial.setTimeout(1);
  pinMode(motor_spoolerPin, OUTPUT);
  InitTimersSafe();
  SetPinFrequencySafe(9, 1000);
  analogWrite(motor_spoolerPin,puller_speed);
}

void loop ()
{
  if(Serial.available())
  {
   puller_speed = Serial.parseInt();  
   Serial.flush(); 
  }
  analogWrite(motor_spoolerPin,puller_speed);   //Set the spool speed to the PID result
}


