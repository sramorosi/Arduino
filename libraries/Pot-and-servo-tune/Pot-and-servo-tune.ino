#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

// Symbolic Constants:
#define POT_PIN 2  // Potentiometer on analog pin (0-5)
#define POT_A 5
#define POT_B 4

// Use these constants to tune the potetiometer RANGE
// number that ranges from 0 (0 Volts) to 1023 (5 Volts)
#define POT_MIN 407
#define POT_MAX 800

// DATA
// A0 PORT (TURNTABLE)  0 1023
#define POT_MIN_T -150
#define POT_MAX_T 20

// A5 PORT (joint A)  Pot range = 70 685,  Input Arm angles = -10  130
#define POT_MIN_A 70
#define POT_MAX_A 685
#define inputARM_MIN_A  -10
#define inputARM_MAX_A  130

// A4 Port (Joint B)  Pot range = 235 964,   Input Arm angles = -150 20
#define POT_MIN_B  235
#define POT_MAX_B  964
#define inputARM_MIN_B  -150
#define inputARM_MAX_B  20

// A2 Port (Joint D-CLAW)  Pot range = 407 800,   Input Arm angles = 0 180
#define POT_MIN_D  407
#define POT_MAX_D  800


// JOINTS
#define JNT_MIN 0
#define JNT_MAX 180

Servo servoA;  // servo for testing

// Use these constants to control the range of the servo outputs
// Values are microseconds (~700 to ~2300--SERVOS MUST BE TESTED), REGARDLESS OF SERVO RANGE
// 270 deg servo, could get 260 deg motion with 500 to 2800 microsecond range
// 180 deg servo, could get 160 deg motion with 500 to 2800 microsecond range
// 180 deg SAVOX, could get 170 deg motion with 500 to 2800 microsecond range
//    ** Start small and increase with tuning **
//    ** to avoid current overloads           **
#define JNT_SVO_MIN 500   // microseconds
#define JNT_SVO_MAX 2400  // microseconds

#define SVO_MIN 0 // used with constrain to keep servo value acceptable
#define SVO_MAX 180 // used with constrain to keep servo value acceptable

#define SVO_RATE_LIMIT 100 // milliseconds per degree

int pot;
int potA,minA,maxA;  // stores the potentiometer reading
int inputJoint,priorA,deltaA;  // stores the computed value for the servo
int potB;
int inputA,inputB,outputB,outputC;

unsigned long millisTime,priorTime,deltaTime;

void setup() {
  servoA.attach(9); // Servo is attached to Digital pin D9

  // SERIAL OUTPUT AFFECTS SMOOTHNESS OF SERVO PERFORMANCE 
  //  WITH SERIAL ON AND LOW BAUD RATE = JERKY PERFORMANCE
  //   WITH OFF OR HIGH 500000 BAUD RATE = SMOOTH PERFORMANCE
  Serial.begin(9600); // baud rate, slower is easier to read
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 
  priorTime = millis();
  millisTime = priorTime;

  priorA = 0;
  deltaA = 0;
  pot = 500;  // VALUE IN MID RANGE SO THE MIN AND MAX WORK
  minA = 10000;
  maxA = 0;
}

void loop() {
  priorTime = millisTime;
  millisTime = millis();
  deltaTime = millisTime-priorTime;

  pot = analogRead(POT_PIN);
  potA = analogRead(POT_A);    // read the pot.
  potB = analogRead(POT_B); 
  minA = min(minA,pot); // check for min value
  maxA = max(maxA,pot); // check for max value

  // Both map the pot value to degrees and constrain the angle, to prevent current overloads.
  // map(value, fromLow, fromHigh, toLow, toHigh)
  priorA = inputJoint;
  //inputJoint = constrain(map(potA,POT_MIN,POT_MAX,JNT_SVO_MIN,JNT_SVO_MAX),SVO_MIN,SVO_MAX);    
  inputJoint = map(pot,POT_MIN,POT_MAX,JNT_MIN,JNT_MAX);    
  inputA = map(potA,POT_MIN_A,POT_MAX_A,inputARM_MIN_A,inputARM_MAX_A);    
  inputB = map(potB,POT_MIN_B,POT_MAX_B,inputARM_MIN_B,inputARM_MAX_B);    
   deltaA = priorA-inputJoint;

  outputB = inputA + inputB;
  outputC = constrain((-90 - outputB),-120,120);  // want the hand to be always pointed down, within range of -120 to 120
  
  // Change servo positions using write command (degrees):
  servoA.writeMicroseconds(inputJoint);

  // output for debugging
  // Serial.print(val,digits)
  //Serial.print("millis,");
  //Serial.print(millisTime);
  //Serial.print(",deltaTime,");
  //Serial.print(deltaTime);
  Serial.print(",pot,");
  Serial.print(pot);  
  Serial.print(",min_potA,");
  Serial.print(minA);
  Serial.print(",max_potA,");
  Serial.print(maxA);
  Serial.print(",inputJoint,");
  Serial.print(inputJoint);
  Serial.print(",inputA,");
  Serial.print(inputA);
  Serial.print(",inputB,");
  Serial.print(inputB);
  Serial.print(",outputB,");
  Serial.print(outputB);
  Serial.print(",outputC,");
  Serial.print(outputC);
  //Serial.print(",deltaA,");  // getting delta in the 200 when moving fast
  //Serial.print(deltaA);
  
  Serial.println("     END");

}
