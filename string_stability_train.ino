//
// Train_2016C_Week1_Tests
// 2016-11-01 : BDK 
//
#define deltaT_ms 20 // digital control frame rate
#define debug_Mode 1 // Serial debug messages (1=on) (0=off) 
//
// SPI & SD libraries used for uSD card interface
//
#include <SPI.h>
#include <SD.h>
//
// Note:
//   Arduino = Master (controls the communication)
//   SD Card = Slave
//
// define pin usage
//
// (11-13 are fixed so no variables needed)
// Pin 11 = SPI MOSI (Master Out / Slave In)
// Pin 12 = SPI MISO (Master In / Slave Out)
// Pin 13 = SPI CLK (Clock)
//
#define chipSelectSPIPin 10 // SPI chip select (must use Pin 10 for SD?)
#define chipDetectPin 9  // NOT USED -- pulled low by SD breakout board when card present
#define pingTrigPin 4 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 5 // ping sensor echo pin (input to Arduino)
#define motorPin 6  // PWM from motor
double error_old = 0;
float integralErrorSum = 0;

//
// SD data file needs to be global (sadly)
//
File dataFile;
//
void setup(void)
{
  //
  // set pin states
  //
  pinMode(pingTrigPin, OUTPUT);
  pinMode(pingEchoPin, INPUT);
  setTrainMotorPWM(0);
  //
  // Initialize serial communication
  // Use high baud rate to allow rapid execution
  //
  Serial.begin(38400);
  Serial.println("Train Control Running");
  //
  // initialize the SD card
  //
  manageSD(0,"Train Control Data Log");  
}

////////////////////////////////////////////////////////////
// start main loop
////////////////////////////////////////////////////////////
void loop(void)
{
  //
  // wait for fixed frame rate
  //
  static unsigned long millisLast = 0;
  word deltaT_ms_actual;
  while (millis() - millisLast < deltaT_ms) {  }
  deltaT_ms_actual = millis() - millisLast;
  millisLast = millis();
  manageSD(1,String(millisLast));
  if (debug_Mode) {Serial.print(deltaT_ms_actual); Serial.print(" ");}
  //
  // Get the ping distance to the train ahead
  //
  double d_Ping_cm = 0.0;
  d_Ping_cm = getPingDistance_cm();
  manageSD(1,String(d_Ping_cm));
  if (debug_Mode) {Serial.print(d_Ping_cm); Serial.print(" ");}
  //
  // do the control law calculations -- MUST BE IN SUBROUTINE!
  //
  byte train_control_pwm=0;
  train_control_pwm = train_control_law(d_Ping_cm);
  manageSD(1,String(train_control_pwm));
  if (debug_Mode) {Serial.print(train_control_pwm); Serial.print(" ");}
  //
  // send the PWM command to the motor
  //
  setTrainMotorPWM(train_control_pwm);
  //
  // new line for serial terminal & for data log
  //
  if (debug_Mode) {Serial.println();}
  manageSD(2," ");
}
////////////////////////////////////////////////////////////
// end main loop
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// manage the SD card stuff
////////////////////////////////////////////////////////////
void manageSD(int ActionCode, String s)
{
  static int line_count = 0;
  int line_count_max = 100;

  if (ActionCode == 0) { // initialize
    if (!SD.begin(chipSelectSPIPin)) {
      if (debug_Mode){Serial.println("Card failed, or not present");}
    }
    else {
      if (debug_Mode) {Serial.println("card initialized.");}
    }
    dataFile = SD.open("data.txt", FILE_WRITE);
    if (debug_Mode) {Serial.println("SD file opened");}
    dataFile.println(s);
  }
  else if (ActionCode == 1) {
    s+= ",";
    dataFile.print(s);
  }
  else if (ActionCode == 2) {
    dataFile.println(s);
    line_count++;
    if (line_count > line_count_max) {     // prevent loss of data due to buffering by close & re-open data file
      dataFile.close();
      dataFile = SD.open("data.txt", FILE_WRITE);    
      if (debug_Mode) {Serial.println("SD file opened");}
      line_count = 0;
    }
  }
  return;
}

////////////////////////////////////////////////////////////
// Ping Sensor Input
////////////////////////////////////////////////////////////
double getPingDistance_cm()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  return 0.017 * echo_time;
}

////////////////////////////////////////////////////////////
// Train Motor Output
////////////////////////////////////////////////////////////
void setTrainMotorPWM(byte pwm_command)
{
  analogWrite(motorPin, pwm_command);
  delay(2);
  return;
}
////////////////////////////////////////////////////////////
// end of Train Motor Output
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// train control law
// this is the code that must be modified
////////////////////////////////////////////////////////////
byte train_control_law(double y)
{
  byte train_control_pwm;
  static word omega_count = 0;
  static double omega_krps = 0.0005;  //0.5 rad/s
  omega_count++;
  if (omega_count > max(500,(1.0/omega_krps))) {
    omega_count = 0;
    omega_krps = omega_krps + 0.0005; //increment rad/s until it reaches 1 rad/s
    if (omega_krps > 0.010) {
      omega_krps = 0.0005;
    }
  }

  static double train_control_percent = 0;
  float integralControl = 0;
  
  double yd = 15 + 5*sin(omega_krps*millis());
  manageSD(1,String(yd));
  if (debug_Mode) {Serial.print(yd); Serial.print(" ");}

  double error = y - yd; //[cm] /if you're closer to train than you want to be, y-yd < 0 then you need to slow down so control will be smaller

  //Proportional Control
  const float kp = 1.65;
  
  //Derivative Control
  const float kd = 0;
  double derivative = kd*(error - error_old)*1000/deltaT_ms;
  float derivativeControl = derivative;

  //Integral Control
  if (train_control_percent < 15 || train_control_percent > 240)
  {
  }
  else
  {
    const float ki = 0.8;
    integralErrorSum += error*deltaT_ms/1000;
    integralControl = ki*integralErrorSum;
  }

  
  
  train_control_percent = 75.0 + kp * error + derivativeControl + integralControl; //proportional feedback, 75% of control authrority is nominal operating conditions
  error_old = error;
  
  //train_control_pwm = constrain(2.55 * train_control_percent,0,255); // scale percent into PWM byte  
  
  train_control_pwm = 80.0*2.55;
  return train_control_pwm;
}
////////////////////////////////////////////////////////////
// end of train control law
////////////////////////////////////////////////////////////

//file output
//second, max time is 51 seconds means no echo, desired distance, train control in pwm units
//when it doesn't see naything (51), it's running at max 255 pwm
//sd card has a lot of data, look for your data
