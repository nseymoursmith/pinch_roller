#include <PID_v1.h>

//Pinch roller feedback by N. Seymour-Smith
//Modified from:
//Filament Winder by Ian Johnson

#include <EEPROM.h>

//Spooler PID Setup
#include <PID_v1.h>
int Setpoint, Input, Output;                            //Define PID Variables
//PID pullPID(&Input, &Output, &Setpoint, 1, 0, 0, DIRECT);    //Specify the links and initial tuning parameters

float kp = 1.0;
int error = 0;

//Digital Pins
int motor_spoolerPin = 5;     //PWM pin for spooler motor

//Variables for puller speed control
int line_position;
int puller_speed = 125;
int speed_offset = 125;

//Serial communication
const int buff = 20;
char command[buff]; // Command string (time in sec, to record)
char inChar; // incoming character
byte index = 0; // string index

void setup ()
{
  Serial.begin (9600);
  while(!Serial);
  Serial.setTimeout(10);
  pinMode(motor_spoolerPin, OUTPUT);

  //Pull PID
    //initialize the variables
  Setpoint = 190;                       //The value PID trys to maintain.  The number controls the amount of tension on the spool.
  line_position = Setpoint;
//  Input = line_position; 
 // pullPID.SetMode(AUTOMATIC);          //turn the PID on
 // pullPID.SetControllerDirection(REVERSE);
}

void loop ()
{
//serial_output();       //Go to the function that outputs messages to the serial monitor.  Comment this out to turn them all off.
serial_check();
pull_control();       //Go to the spool_control function for auto control

}
  
   
void serial_check()
{
  if(Serial.available())
  {
   line_position = Serial.parseInt();   
  }
}
   
void pull_control()
{
//  Serial.print(line_position);
//  Serial.print(", ");
//  Serial.print(error);
//  Serial.print(", ");
//  Serial.println(Output);
  Input = line_position;                 //Get line position from sensors
//  pullPID.Compute();  //Run the PID 
 // Output = (Output / 1.5);             //Scale the Output from 0-300 to 0-255)   
  error = Input - Setpoint;
  Output = int(kp*error); 
  if (Output <= 0) {Output = 1;}         //Limit the output to the range of 0-255) 
  if (Output >= 222){Output = 255;}
  puller_speed = - Output + 125 + speed_offset;
  if (puller_speed <= 0) { puller_speed = 1;}
  if (puller_speed >= 255) {puller_speed = 255;}
  analogWrite(motor_spoolerPin,puller_speed);   //Set the spool speed to the PID result



}


