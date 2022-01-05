/*
Control a robot arm (using Servos) from an Input arm (using Potentiometers)

by SrAmo, July 3, 2021

USE SERIAL PLOTTER TO REVIEW ONE VARIABLE AT A TIME

*/

#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#include "RunningAverage.h"
// Running Average library at: https://www.arduino.cc/reference/en/libraries/runningaverage/

// Define a running average object for each potentiometer
// This is used to smooth the readings and to slow down
//   the arm accelerations, for safety.
// The integer is the size of the running average list
//  25 is too slow.  15 is still a bit slow
#define RA_SIZE 5 // size of the running average list
RunningAverage potA_RA(RA_SIZE);
RunningAverage potB_RA(RA_SIZE);
RunningAverage potC_RA(RA_SIZE);
RunningAverage potD_RA(RA_SIZE);
RunningAverage potT_RA(RA_SIZE);

// Symbolic Constants:
#define JNTA_POT_PIN 5  // Joint A Potentiometer on analog pin A5
#define JNTB_POT_PIN 4  // Joint B Potentiometer on analog pin A4
#define JNTC_POT_PIN 3  // Joint C Potentiometer on analog pin A3
#define JNTD_POT_PIN 2  // Joint D (claw) Potentiometer on analog pin A2
#define JNTT_POT_PIN 1  // Joint T (table) Potentiometer on analog pin A1
// A0 not used at this  time

// Use these constants to tune the potetiometer RANGE
// number that ranges from 0 (0 Volts) to 1023 (5 Volts)
#define JNTA_POT_MIN 60 
#define JNTA_POT_MAX 650 

#define JNTB_POT_MIN 160 
#define JNTB_POT_MAX 950 

#define JNTC_POT_MIN 750 // reverse 720
#define JNTC_POT_MAX 400  // reverse  280

#define JNTD_POT_MIN 720 // reverse 
#define JNTD_POT_MAX 400  // reverse

#define JNTT_POT_MIN 200 
#define JNTT_POT_MAX 600

// Use these constants to control the range of the servo outputs
// Values are DEGREES (typically 0 to 180)  
//    ** Start small and increase with tuning **
//    ** to avoid current overloads           **
#define JNTA_SVO_MIN 30   // up/back
#define JNTA_SVO_MAX 130  // down/out

#define JNTB_SVO_MIN 0
#define JNTB_SVO_MAX 160

#define B_SVO_MIN 0
#define B_SVO_MAX 180

#define JNTC_SVO_MIN 30
#define JNTC_SVO_MAX 130

#define JNTD_SVO_MIN 160
#define JNTD_SVO_MAX 20

#define JNTT_SVO_MIN 160
#define JNTT_SVO_MAX 20

#define SVO_MIN 0 // used with constrain to keep servo value acceptable
#define SVO_MAX 180 // used with constrain to keep servo value acceptable

// Declare global variables

// Create the servo objects, one for each servo.
// You can control a maximum of twelve servos on the Uno 

Servo servoA;  // servo for the shoulder
Servo servoB;  // servo for the elbow
Servo servoC;  // servo for the wrist
Servo servoD;  // servo for the Claw
Servo servoT;  // servo for the Turntable

void setup() // this function runs once when the sketch starts up
{
  // No need to setup "analog in" pin. 

  /* Initialize running average: Doesn't seem to work very well.
  potA_RA.clear();
  potA_RA.fillValue(20,RA_SIZE);  // fillValue(value, nr) adds nr elements of value.
  potB_RA.clear();
  potB_RA.fillValue(20,RA_SIZE);
  potC_RA.clear();
  potC_RA.fillValue(20,RA_SIZE);
  potD_RA.clear();
  potD_RA.fillValue(20,RA_SIZE);
  */

  // Attach tells the Arduino to begin sending control signals
  // to the servo. Servos require a continuous stream of control
  // signals, even if you're not currently moving them.

  servoA.attach(9); // A shoulder
  servoB.attach(5); // B elbow
  servoC.attach(6); // C wrist
  servoD.attach(10); // Claw (D)
  servoT.attach(11); // Turntable

  Serial.begin(250000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 

  // prints title with ending line break
  Serial.println("MULTI SERVO CONTROL");
}

void loop() // this function runs repeatedly after setup() finishes
{
  // declare integer variable to store the value of potentiometers:

  int potA,potB,potC,potD,potT;
  int smooth_potA,smooth_potB,smooth_potC,smooth_potD,smooth_potT;
  int jntA, jntB, jntC, jntD,jntT;
  
  int jntB_Mod, jntB_Mod_temp;
  int B_map_min,B_map_max;
  
  // The potentiometer is set up as a voltage divider, so that
  // when you turn it, the voltage on the center pin will vary
  // from 0V to 5V. 
  
  // The Arduino can read external voltages on the analog input
  // pins using a built-in function called analogRead(). This
  // function takes one input value, the analog pin. It returns an integer
  // number that ranges from 0 (0 Volts) to 1023 (5 Volts).
 
  potA = analogRead(JNTA_POT_PIN);    // A pot
  potB = analogRead(JNTB_POT_PIN);    // B pot
  potC = analogRead(JNTC_POT_PIN);    // C pot
  potD = analogRead(JNTD_POT_PIN);    // D pot
  potT = analogRead(JNTT_POT_PIN);    // T pot

  // add the value to the Running Average list
  potA_RA.addValue(potA);
  potB_RA.addValue(potB);
  potC_RA.addValue(potC);
  potD_RA.addValue(potD);
  potT_RA.addValue(potT);

  // get smoothed value
  smooth_potA = potA_RA.getAverage();
  smooth_potB = potB_RA.getAverage();
  smooth_potC = potC_RA.getAverage();
  smooth_potD = potD_RA.getAverage();
  smooth_potT = potT_RA.getAverage();

  // Both map the pot value to degrees and constrain the angle, to prevent current overloads.
  // map(value, fromLow, fromHigh, toLow, toHigh)
  jntA = constrain(map(smooth_potA,JNTA_POT_MIN,JNTA_POT_MAX,JNTA_SVO_MIN,JNTA_SVO_MAX),SVO_MIN,SVO_MAX);    
  jntB = constrain(map(smooth_potB,JNTB_POT_MIN,JNTB_POT_MAX,JNTB_SVO_MIN,JNTB_SVO_MAX),SVO_MIN,SVO_MAX);
  jntC = constrain(map(smooth_potC,JNTC_POT_MIN,JNTC_POT_MAX,JNTC_SVO_MIN,JNTC_SVO_MAX),JNTC_SVO_MIN,SVO_MAX);    
  jntD = constrain(map(smooth_potD,JNTD_POT_MIN,JNTD_POT_MAX,JNTD_SVO_MIN,JNTD_SVO_MAX),SVO_MIN,SVO_MAX);    
  jntT = constrain(map(smooth_potT,JNTT_POT_MIN,JNTT_POT_MAX,JNTT_SVO_MIN,JNTT_SVO_MAX),SVO_MIN,SVO_MAX);    

  // Servo B is on joint A, so need to combine A and B
  jntB_Mod_temp = jntA+(160-jntB);
  //B_map_min = JNTA_POT_MIN + JNTB_POT_MIN+40; // pysically can't get to the min, add 40
  //B_map_max = JNTA_POT_MAX + JNTB_POT_MAX;
  //  JOINT B DOES NOT MOVE LIKE THE INPUT ARM.  ALSO HAVE CONSTANTS HERE.
  jntB_Mod = constrain(map(jntB_Mod_temp,70,220,B_SVO_MIN,B_SVO_MAX),SVO_MIN,SVO_MAX);  
  //jntB_Mod = constrain(map(jntB_Mod_temp,B_map_min,B_map_max,B_SVO_MIN,B_SVO_MAX),B_SVO_MAX,B_SVO_MIN);  

// use one of these lines with serial plotter to tune the constants
  //Serial.println(potC);  // change to potB, potC, etc. to see others
  //Serial.println(jntC); // change to jntA, jntB, jntC, etc to see others
  //Serial.println(jntB_Mod);

  // Change servo positions using write command (degrees):
  // NOTE: DRIVING PAST ENDPOINTS IS A HIGH CURRENT STATE.
  //
    servoA.write(jntA);
    servoB.write(jntB);
    servoC.write(jntC);
    servoD.write(jntD);
    servoT.write(jntT);
    //

//
  // output for debugging
  // Serial.print(val,digits)
  Serial.print("millis,");
  Serial.print(millis());
  Serial.print(",potA,");
  Serial.print(potA,1);  
  Serial.print(",smooth_potA,");
  Serial.print(smooth_potA);  
  Serial.print(",jntA,");
  Serial.print(jntA,1);
  Serial.print(",potB,");
  Serial.print(potB,1);
  Serial.print(",smooth_potB,");
  Serial.print(smooth_potB);
  Serial.print(",jntB,");
  Serial.print(jntB,1);
  Serial.print(",jntB_Mod_temp,");
  Serial.print(jntB_Mod_temp,1);
  Serial.print(",jntB_Mod,");
  Serial.print(jntB_Mod,1);
  Serial.print(",potC,");
  Serial.print(potC,1);
  Serial.print(",jntC,");
  Serial.print(jntC,1);
  Serial.print(",potD,");
  Serial.print(potD,1);
  Serial.print(",jntD,");
  Serial.println(jntD,1);
//
}
