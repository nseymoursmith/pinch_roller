#include <QTRSensors.h>

#include <PID_v1.h>




//Filament Winder by Ian Johnson

// Using QTR Sensor library from Pololu for line following 

#include <EEPROM.h>

//Guide Servo

#include <Servo.h> 
Servo servo;  
int angle = 0;   // servo position in degrees 

//Filament Sensor
#include <QTRSensors.h>
#define NUM_SENSORS   4     // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     

// sensors 0 through 3 are connected to analog pins pins 3 through 0 , respectively
QTRSensorsAnalog qtra((unsigned char[]) {3, 2, 1, 0}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


//Spooler PID Setup
#include <PID_v1.h>
double Setpoint, Input, Output;                            //Define PID Variables
PID pullPID(&Input, &Output, &Setpoint, .17, 0, 0, REVERSE);    //Specify the links and initial tuning parameters

//Digital Pins
int hall_a_Pin = 7;           // Hall sensor A
int sensor_setPin = 4;           //Button for setting the Pull PID setpoint
int motor_spoolerPin = 5;     //PWM pin for spooler motor
int guidePin = 6;             //Guide servo
int guide_maxPin = 3;        //Button for setting the Max Guide limits
int guide_minPin = 8;        //Button for setting the Min Guide limit
int toggle = 2;              //Auto / Manual toggle switch

//Analog Pins
int knob_Pin = 6;   // Pot that controls the fast puller speed to raise the loop
int sensor_1 = 0;         //Top filament sensor
int sensor_2 = 1;      //MIddle filament sensor
int sensor_3 = 2;     //Bottom filament sensor
int sensor_4 = 3;

//Variables for spool rotation
int hall_a_status = HIGH;    //The last reading from the Hall sensor
int hall_b_status = HIGH;    //The last reading from the other Hall sensor
int hall_a_mode = 0;          //Has Hall A been triggered?
int hall_b_mode = 0;          //Has Hall B been triggered?
int Revolutions = 0;          //Rotation counter
int Last_Revolution = 0;      //For checking if a new rotation hasbeen counted

//Variables for moving the guide
float guide_min = EEPROM.read(2);           //Right limit for filament guide
float guide_max = EEPROM.read(1);          //Left Limit for Filamnet guide
int guide_direction = 0;        //Direction the guide is moving
float guide_angle = 90;         //Start out in the middle
float last_position;

//Variables for smoothing the potentiometer reading
const int numReadings = 5;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int inputPin = knob_Pin;





//Variables for puller speed control via the photo sensors

unsigned int line_position; 
int spooler_speed = 200;
int puller_speed = 100;
int rotation_status = 0;

//Var MIE

int counter = 0;

void setup ()
{
  Serial.begin (9600);
  Serial.println("START");
  
  /*
  Serial.print("GUIDE MIN ");
  Serial.println(guide_min); 
  Serial.print("GUIDE MAX ");
  Serial.println(guide_max); 
  */
  
  pinMode(motor_spoolerPin, OUTPUT);
  pinMode(hall_a_Pin, INPUT);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  pinMode(guidePin, OUTPUT);
  pinMode(sensor_setPin, INPUT);
  pinMode(toggle, INPUT);
  pinMode(sensor_1, INPUT);
  pinMode(sensor_2, INPUT);
  pinMode(sensor_3, INPUT);
  pinMode(sensor_4, INPUT);
 
  
  digitalWrite(hall_a_Pin, HIGH);        //Pullup resistor for Hall A
  digitalWrite(8, HIGH);      //Pullup resistor for guide setup switch
  digitalWrite(3, HIGH);      //Pullup resistor for guide setup switch
  digitalWrite(guidePin, HIGH);
  digitalWrite(sensor_setPin, HIGH);        //Pullup resistor for pid setup switch
  digitalWrite(toggle, HIGH);            //Pullup resistor for toggle
//  digitalWrite(sensor_1, HIGH);
//  digitalWrite(sensor_2, HIGH);
//  digitalWrite(sensor_3, HIGH);
//  digitalWrite(sensor_4, HIGH);

  //
  
  
  servo.attach(guidePin); 
  servo.write(90 );

  //Pull PID
    //initialize the variables
  Input = line_position; 
  Setpoint = 1500;                       //The value PID trys to maintain.  The number controls the amount of tension on the spool.
  pullPID.SetMode(AUTOMATIC);          //turn the PID on
  pullPID.SetControllerDirection(REVERSE);
  
    // initialize all the knob readings to 0: 
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;   
  
  
}

void loop ()
{
//serial_output();       //Go to the function that outputs messages to the serial monitor.  Comment this out to turn them all off.



//Guide setup switch

      
while (digitalRead(4) == 0 && digitalRead(toggle) == 0) {    //If middle button is pressed while in auto mode, set the current guide position
float last;
last = guide_position(last_position);
last_position = last;


}

if (digitalRead(3) == 0 && digitalRead(8) == 1) {     //If guide max nutton is pressed go to calibrate_max
                                                      //calibrate_max will loop while the button is held down.
calibrate_max();                                      //When the button is let up it will return here and write the
EEPROM.write(1, guide_max);                           //new value to the EEPROM
}

if (digitalRead(8) == 0 && digitalRead(3) ==1) {      //If guide min button is pressed go to guide min

calibrate_min();
EEPROM.write(2, guide_min); 
}

// Calibrate   
if (digitalRead(4) == 0 && digitalRead(toggle) == 1 ){
 set_sensor();
}


guide_control();       //Go to the guide control function


if (digitalRead(toggle) == 1) {
manual_control();      //Go to the manual_control function for manual pull control
}

if (digitalRead(toggle) == 0) {
  
pull_control();       //Go to the spool_control function for auto control
}

}
   
   
   
   
void serial_output()
{

Serial.print("Min ");
Serial.print(guide_min);

Serial.print(" Max ");
Serial.print(guide_max);
Serial.print(" Guide Angle ");
Serial.print(guide_angle);

Serial.print(" Guide Direction ");
Serial.println(guide_direction);


}
   
   
void manual_control()
{  

  int knob_reading = analogRead(knob_Pin);  // Get value from Puller Max Speed Knob
  int puller_speed = knob_reading / 4.011;           //convert reading from pot to 0-255
  analogWrite(motor_spoolerPin, puller_speed);          //Set motor to the speed

  }

void pull_control()
{
  qtra.readCalibrated(sensorValues);
  unsigned int line_position = qtra.readLine(sensorValues, QTR_EMITTERS_OFF, 1);  
  
 // STAMPA IN SERIALE VELOCITA e POSIZIONE LINEA
 /*
    if (counter==100)
     {
     Serial.print("LINE ");
     Serial.println(line_position); 
     Serial.print("VEL ");
     Serial.println(puller_speed); 
     counter=0;
    }
    else
    {
      counter=counter + 1;
      
    }
*/
  

  Input = line_position;                 //Get line position from sensors
  pullPID.Compute();  //Run the PID 
 // Output = (Output / 1.5);             //Scale the Output from 0-300 to 0-255)       
  if (Output <= 0) {Output = 1;}         //Limit the output to the range of 0-255) 
 // if (Output >= 222){Output = 255;}
  puller_speed = Output;
  analogWrite(motor_spoolerPin,puller_speed);   //Set the spool speed to the PID result


  //Serial.print("VEL ");
  //Serial.println(puller_speed);

}

   
void guide_control()
{
  //The hall sensor might get checked several times while the magnet is in range, but we don't want a rotation logged with every check
  //while the magnet passes by.  When a rotation is logged, rotation status gets set to 1 so it doesn't get logged again until after the hall has
  //switched off.
  
    if (digitalRead(hall_a_Pin) == 0) {         //Keep rotation status at 0 as long as hall isn't triggered
    rotation_status = 0;
    }

   

   // If Hall A has been triggered and rotation status is 0, log a rotation
  if (digitalRead(hall_a_Pin) == 1 && rotation_status == 0){    
   
   if (guide_angle < guide_min){              //If the guide angle passes minimum set direction to forward
     guide_angle = guide_min;  
     guide_direction = 0;}
 
    if (guide_angle > guide_max) {            //If the guide angle has reached maximum change direction to back
    guide_angle = guide_max;  
    guide_direction = 1;}
 
    if (guide_direction == 0) {               //If the current direction of the guide is forward
    guide_angle = (guide_angle + 1.17);       //Move the guide +1 degree
    servo.write(guide_angle);}
    
   if (guide_direction == 1) {                //If the current direction of the guide is back
   guide_angle = (guide_angle - 1.17);        //Move the guide -1 degree
     servo.write(guide_angle);}
   
     rotation_status = 1;                    //Remember that a rotation was counted 
     Serial.print("GUIDE ");
     Serial.println(guide_angle); 
  float last_position = guide_angle;   
   
  }
 
  // If Hall A is still being triggered after the rotation has been counted (rotation_status = 0) nothing will happen.
    
}

float guide_position(float last_pos){

smoothing();
  int reading = average;               // 0 to 1023
 float  cal_position = reading / 5;                        // 0 to 180-ish
  servo.write(cal_position);  //Move the guide to the knob position
  guide_angle = cal_position; 
    if (guide_angle > last_pos) {            //If the guide angle has reached maximum change direction to back  
    guide_direction = 0;} 
  
  if (guide_angle < last_pos){              //If the guide angle passes minimum set direction to forward
     guide_direction = 1;}
last_pos = guide_angle;
return last_pos;
Serial.print("Reading ");
Serial.print(average);
Serial.print(" Angle ");
Serial.println(guide_angle);
}



void calibrate_max()  // Use the puller speed knob to set the guide limits
{
while (digitalRead(3) == 0 && digitalRead(8) == 1) {
smoothing();
  int reading = average;              // 0 to 1023
 int  cal_position = reading / 5;                        // 0 to 180-ish
 Serial.println(cal_position);
  servo.write(cal_position);                        //Move the guide to the knob position
    guide_max = cal_position;                       //Make the current position the max limit
}
 
  }


void calibrate_min()  // Use the puller speed knob to set the guide limits
{
while (digitalRead(8) == 0 && digitalRead(3) ==1) {
smoothing();
  int reading = average;                 // 0 to 1023
 int  cal_position = reading / 6;    // 0 to 180-ish
 //Serial.println(cal_position);
  servo.write(cal_position);                        //Move the guide to the knob position
    guide_min = cal_position;                       //Make the current position the max limit
  }
}


void set_sensor()                                    //Calibrate the reflectance sensors  Add LED indication that calibration is happening
{
  delay(500);
 
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  Serial.print("Minimum Values: ");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("Maximum Values: ");
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}
 
 int smoothing(){
     // subtract the last reading:
  total= total - readings[index];         
  // read from the sensor:  
  readings[index] = analogRead(inputPin); 
  // add the reading to the total:
  total= total + readings[index];       
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning: 
    index = 0;                           

  // calculate the average:
  average = total / numReadings;         
  // send it to the computer as ASCII digits
  Serial.println(average);   
  delay(1);        // delay in between reads for stability  
  //return average;
  
 }

