#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

//#define SERIALOUT TRUE  // COMMENT-OUT TO TURN OFF SERIAL AND MAKE SMOOTH!

// CONSTANTS:
// Use PIN_ constants to identify the analog pin (0-5) that the Potentiometer is on
// Use POT_ constants to set potetiometer ranges from 0 (0 Volts) to 1023 (5 Volts)
// Use INPUTARM_ constants to map the Input Angle (deg)for a given potentiometer value
// Compute Output Arm Angles from INPUTS and map() to compute value for servo
// Use SVODIG_ to identify the digital pin that the servo is attached to
// Use SVO_ constants to set servo range in microseconds - 
// Values are microseconds (~700 to ~2300--SERVOS MUST BE TESTED), REGARDLESS OF SERVO RANGE
// 270 deg servo, gets 260 deg motion with 500 to 2800 microsecond range
// 180 deg servo, gets 160 deg motion with 500 to 2800 microsecond range
// 180 deg SAVOX, gets 170 deg motion with 500 to 2800 microsecond range
//    ** Start small and increase with tuning **
//    ** to avoid current overloads           **

// TEST VALUES
#define PIN 0    // TEST PIN IS ZERO
#define POT_MIN 0   // 0 TO 1023
#define POT_MAX 1023   // 0 TO 1023
#define INPUTARM_MIN 0  // DEG
#define INPUTARM_MAX 180  // DEG
#define SVODIG 11
#define OUTPUTARM_MIN 0  // DEG
#define OUTPUTARM_MAX 180  // DEG
#define SVO_MIN 500   // microseconds
#define SVO_MAX 2400  // microseconds

// ROBOT ARM DATA
// TURNTABLE, A0 pin
#define PIN_T 0
#define POT_MIN_T 0
#define POT_MAX_T 1023
#define INPUTARM_MIN_T 0  // DEG
#define INPUTARM_MAX_T 180  // DEG
#define SVODIG_T 11
// straight out is at 30 DEG, 1870 microseconds
#define OUTPUTARM_MIN_T 0  // DEG
#define OUTPUTARM_MAX_T 180  // DEG
#define SVO_MIN_T 500   // microseconds
#define SVO_MAX_T 2400  // microseconds

// JOINT D (CLAW), A2 pin, Pot range = 407 800,   Input Arm angles = 0 160
#define PIN_D 2
#define POT_MIN_D  407
#define POT_MAX_D  800
#define INPUTARM_MIN_D 0  // DEG
#define INPUTARM_MAX_D 160  // DEG
#define SVODIG_D 10
#define OUTPUTARM_MIN_D 0  // DEG
#define OUTPUTARM_MAX_D 160  // DEG
#define SVO_MIN_D 500   // microseconds
#define SVO_MAX_D 2300  // microseconds

// JOINT A, A5 pin, Pot range = 70 685,  Input Arm angles = -10  130
#define PIN_A 5
#define POT_MIN_A 70
#define POT_MAX_A 685
#define INPUTARM_MIN_A  -10
#define INPUTARM_MAX_A  130
#define SVODIG_A 9
#define OUTPUTARM_MIN_A 0  // DEG
#define OUTPUTARM_MAX_A 153  // DEG
#define SVO_MIN_A 810   // microseconds
#define SVO_MAX_A 2430  // microseconds

// JOINT C, 
//#define PIN_C  COMPUTED INPUT.  MIGHT HAVE A TUNER POT IN THE FUTURE
#define SVODIG_C 6
#define OUTPUTARM_MIN_C -30  // NEED TO UPGRADE TO 270 DEG SERVO
#define OUTPUTARM_MAX_C 120
#define SVO_MIN_C 2330   // microseconds
#define SVO_MAX_C 350  // microseconds

// JOINT B, A4 pin, Pot range = 235 964,   Input Arm angles = -150 20
#define PIN_B 4
#define POT_MIN_B  235
#define POT_MAX_B  964
#define INPUTARM_MIN_B  -150
#define INPUTARM_MAX_B  20
#define SVODIG_B 5
#define OUTPUTARM_MIN_A 130  // DEG
#define OUTPUTARM_MAX_A -55  // DEG
#define SVO_MIN_A 830   // microseconds
#define SVO_MAX_A 2210  // microseconds

#ifdef SERIALOUT
  #define SVO_RATE_LIMIT 50.0  // UNITS OF servo microseconds per loop. 50 is good for turntable with serial on.
#else
  #define SVO_RATE_LIMIT 0.2  // UNITS OF servo microseconds per loop.  0.2 is good for turntable with serial off.
#endif  

Servo servoA;  // servo for testing

// Use these constants to control the range of the servo outputs

// #define SVO_RATE_LIMIT 100 // milliseconds per degree

int pot;
int potA,minA,maxA;  // stores the potentiometer reading
double inputJoint;
double priorA,deltaA;  // stores the computed value for the servo
int potB;
int inputA,inputB,outputB,outputC;
double outputJoint;
int outputServo;
double rate;

unsigned long millisTime;

void setup() {
  servoA.attach(SVODIG); 

  // SERIAL OUTPUT AFFECTS SMOOTHNESS OF SERVO PERFORMANCE 
  //  WITH SERIAL ON AND LOW BAUD RATE = JERKY PERFORMANCE
  //   WITH OFF OR HIGH 500000 BAUD RATE = SMOOTH PERFORMANCE
#ifdef SERIALOUT
  Serial.begin(9600); // baud rate, slower is easier to read
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 
#endif
  millisTime = millis();

  priorA = 1800;
  
  pot = 500;  // VALUE IN MID RANGE SO THE MIN AND MAX WORK  
  minA = 10000;  // USE FOR DETERMINING RANGE OF POT
  maxA = 0;
}

void loop() {
  millisTime = millis();

  pot = analogRead(PIN);
  //potA = analogRead(PIN_A);    // read the pot.
  //potB = analogRead(PIN_B); 
  
  minA = min(minA,pot); // check for min value
  maxA = max(maxA,pot); // check for max value

  // Both map the pot value to degrees and constrain the angle, to prevent current overloads.
  // map(value, fromLow, fromHigh, toLow, toHigh)

  inputJoint = map(pot,POT_MIN,POT_MAX,INPUTARM_MIN*10,INPUTARM_MAX*10)/10.0;    // MAP POTENTIOMETER TO INPUT ANGLE
  outputJoint = inputJoint;       // CALCULATE OUTPUT ANGLE
  outputServo = map(outputJoint,OUTPUTARM_MIN,OUTPUTARM_MAX,SVO_MIN,SVO_MAX);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  
  //outputB = inputA + inputB;
  //outputC = constrain((-90 - outputB),OUTPUTARM_MIN_C,OUTPUTARM_MAX_C);  // want the hand to be always pointed down, within range of -120 to 120
  
  //inputA = map(potA,POT_MIN_A,POT_MAX_A,INPUTARM_MIN_A,INPUTARM_MAX_A);    
  //inputB = map(potB,POT_MIN_B,POT_MAX_B,INPUTARM_MIN_B,INPUTARM_MAX_B);    
  
  deltaA = priorA-outputServo;
  if (deltaA > SVO_RATE_LIMIT) {
    // Change servo positions using write command (degrees):
    priorA = priorA - SVO_RATE_LIMIT;
    servoA.writeMicroseconds(priorA);    
  } else if (deltaA < SVO_RATE_LIMIT) {
    // Change servo positions using write command (degrees):
    priorA = priorA + SVO_RATE_LIMIT;
    servoA.writeMicroseconds(priorA);    
    
  } else {
    // Change servo positions using write command (degrees):
    priorA = outputServo;
    servoA.writeMicroseconds(priorA);    
  }

#ifdef SERIALOUT
  // output for debugging
  // Serial.print(val,digits)
  Serial.print("millis,");
  Serial.print(millisTime);
  
  Serial.print(",pot,");
  Serial.print(pot);  
  Serial.print(",min_potA,");
  Serial.print(minA);
  Serial.print(",max_potA,");
  Serial.print(maxA);
  Serial.print(",inputJoint,");
  Serial.print(inputJoint,1);
  Serial.print(",outputJoint,");
  Serial.print(outputJoint,1);
  Serial.print(",outputServo,");
  Serial.print(outputServo);
  
  //Serial.print(",inputA,");
  //Serial.print(inputA);
  //Serial.print(",inputB,");
  //Serial.print(inputB);
  //Serial.print(",outputB,");
  //Serial.print(outputB);
 // Serial.print(",outputC,");
  //Serial.print(outputC);
  //Serial.print(",deltaA,");  // getting delta in the 200 when moving fast
  //Serial.print(deltaA);
  
  Serial.println("     END");
#endif
}
