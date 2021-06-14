/*
Control a robot arm (using Servos) from an Input arm (using Potentiometers)

by SrAmo, May 15, 2021

JOINT C SERVO NOT INSTALLED--NEED ARM REDESIGN

USE SERIAL PLOTTER TO REVIEW ONE VARIABLE AT A TIME

*/

#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#include "RunningAverage.h"

// Define a running average object for each potentiometer
// The integer is the size of the running average list
#define RA_SIZE 25 // size of the running average list
RunningAverage potA_RA(RA_SIZE);
RunningAverage potB_RA(RA_SIZE);
RunningAverage potC_RA(RA_SIZE);
RunningAverage potD_RA(RA_SIZE);

// Symbolic Constants:
#define SERVOS FALSE

#define JNTA_POT_PIN 0  // Potentiometer on analog pin 0
#define JNTB_POT_PIN 1  // Potentiometer on analog pin 1
#define JNTC_POT_PIN 2  // Potentiometer on analog pin 2
#define JNTD_POT_PIN 3  // Potentiometer on analog pin 3

// Use these constants to tune the potetiometer sensitivity
// number that ranges from 0 (0 Volts) to 1023 (5 Volts)
#define JNTA_POT_MIN 0 
#define JNTB_POT_MIN 0 
#define JNTC_POT_MIN 60 
#define JNTD_POT_MIN 630  //   BACKWARD

#define JNTA_POT_MAX 680 
#define JNTB_POT_MAX 690 
#define JNTC_POT_MAX 900 
#define JNTD_POT_MAX 300  // BACKWARD

// Use these constants to control the range of the servo outputs
// Values are DEGREES (typically 0 to 180)  
//    Start small and increase with tuning
#define JNTA_SVO_MIN 30   // up/back
#define JNTB_SVO_MIN 30
#define B_SVO_MIN 160  // BACKWARD
#define JNTC_SVO_MIN 40
#define JNTD_SVO_MIN 20

#define JNTA_SVO_MAX 130  // down/out
#define JNTB_SVO_MAX 160
#define B_SVO_MAX 0   // BACKWARD
#define JNTC_SVO_MAX 120
#define JNTD_SVO_MAX 160

// Declare global variables

// Create the servo objects, one for each servo.
// You can control a maximum of twelve servos on the Uno 

Servo servoA;  // servo control object
Servo servoB;  // servo control object
Servo servoC;  // servo control object
Servo servoD;  // servo control object

void setup() // this function runs once when the sketch starts up
{
  // No need to setup "analog in" pin. 

  // Initialize running average
  potA_RA.clear();
  potA_RA.fillValue(90,RA_SIZE);  // fillValue(value, nr) adds nr elements of value.
  potB_RA.clear();
  potB_RA.fillValue(90,RA_SIZE);
  potC_RA.clear();
  potC_RA.fillValue(90,RA_SIZE);
  potD_RA.clear();
  potD_RA.fillValue(90,RA_SIZE);

  // Attach tells the Arduino to begin sending control signals
  // to the servo. Servos require a continuous stream of control
  // signals, even if you're not currently moving them.

  servoA.attach(9); // A
  servoB.attach(5); // B
  servoC.attach(6); // C
  servoD.attach(10); // D

  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 

  // prints title with ending line break
  Serial.println("MULTI SERVO CONTROL");
}

void loop() // this function runs repeatedly after setup() finishes
{
  // declare integer variable to store the value of potentiometers:

  int potA,potB,potC,potD;
  int smooth_potA,smooth_potB,smooth_potC,smooth_potD;
  int jntA, jntB, jntC, jntD;
  int jntB_Mod;
  
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

  // add the value to the Running Average list
  potA_RA.addValue(potA);
  potB_RA.addValue(potB);
  potC_RA.addValue(potC);
  potD_RA.addValue(potD);

  // get smoothed value
  smooth_potA = potA_RA.getAverage();
  smooth_potB = potB_RA.getAverage();
  smooth_potC = potC_RA.getAverage();
  smooth_potD = potD_RA.getAverage();

  // Both map the pot value to degrees and constrain the angle, if something maps wrong
  jntA = constrain(map(smooth_potA,JNTA_POT_MIN,JNTA_POT_MAX,JNTA_SVO_MIN,JNTA_SVO_MAX),JNTA_SVO_MIN,JNTA_SVO_MAX);    
  jntB = constrain(map(smooth_potB,JNTB_POT_MIN,JNTB_POT_MAX,JNTB_SVO_MIN,JNTB_SVO_MAX),JNTB_SVO_MIN,JNTB_SVO_MAX);    
  jntC = constrain(map(smooth_potC,JNTC_POT_MIN,JNTC_POT_MAX,JNTC_SVO_MIN,JNTC_SVO_MAX),JNTC_SVO_MIN,JNTC_SVO_MAX);    
  jntD = constrain(map(smooth_potD,JNTD_POT_MIN,JNTD_POT_MAX,JNTD_SVO_MIN,JNTD_SVO_MAX),JNTD_SVO_MIN,JNTD_SVO_MAX);    

  // Servo B is on joint A, so need to combine A and B
  jntB_Mod = jntA+jntB;
  jntB_Mod = map(jntB_Mod,30,300,B_SVO_MIN,B_SVO_MAX);  

// use one of these lines with serial plotter to tune the constants
  //Serial.println(jntA);
  //Serial.println(jntB_Mod);
  Serial.println(jntC);
  //Serial.println(jntD);

  // To control a servo, you give it the angle you'd like it
  // to turn to. 
  // Change servo positions:
    servoA.write(jntA);
    servoB.write(jntB_Mod);
    servoC.write(jntC);
    servoD.write(jntD);

/*
  Serial.print("potA , ");
  Serial.print(potA);
  Serial.print(":  potB, ");
  Serial.print(potB);
  Serial.print(":  jntB, ");
  Serial.print(jntB);
  Serial.print(": potC , ");
  Serial.print(potC);
  Serial.print(": jntC , ");
  Serial.print(jntC);
  Serial.print(": JOINT D, ");
  Serial.println(potD);
*/
}
