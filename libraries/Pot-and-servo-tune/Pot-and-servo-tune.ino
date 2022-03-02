/* ROBOT ARM CONTROL SOFTWARE
 *  By, SrAmo, February 2022
 *  Lesson Learned: Short loop times equals smooth arm performance.
 *  Turning off serial output makes loops much faster.
 *  Using int rather than float or double makes loops faster.
 *  Turn off any unnesessary code.
 *  Trying to manage the servo rate made things worse.
 */
#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#define SERIALOUT false  // Controlls SERIAL output. Turn off when not debugging. 

#define SCLR 100  // scaler used to improve accuracy of int variables

// TRUE WHEN TUNING ONE POT/SERVO,  FALSE WHEN DOING ALL SERVOS
#define A_ON true
#define B_ON true
#define C_ON true
#define D_ON true
#define T_ON true

//#define lenAB 50.0     // Length of Input AB arm in mm
//#define lenBC 60.0     // Length of Input BC arm in mm
//#define RADIAN 57.2957795  // number of degrees in one radian

// CONSTANTS:
// Use PIN_ constants to identify the analog pin (0-5) for the Potentiometer.
// Use POT_ constants to set potetiometer ranges from 0 (0 Volts) to 1023 (5 Volts)
// Use IA_ constants to map the Input Angle (deg)for a given potentiometer value
// Compute Output Arm Angles from INPUTS and map() to compute value for servos.
// Use SVODIG_ to identify the digital pin that the servo is attached to.
// Use SVO_ constants to set servo range in microseconds 
// Values are microseconds (~700 to ~2300--SERVOS MUST BE TESTED), REGARDLESS OF SERVO RANGE
// 270 deg servo, gets 260 deg motion with 500 to 2800 microsecond range
// 180 deg servo, gets 160 deg motion with 500 to 2800 microsecond range
// 180 deg SAVOX, gets 170 deg motion with 500 to 2800 microsecond range
//   **        avoid current overloads           **

// JOINT A, Analog 4 input pin, Digital 9 output
#define PIN_A 4
#define POT_MIN_A 134
#define POT_MAX_A 870
const int IA_MIN_A = 2*SCLR;
const int IA_MAX_A = 175*SCLR;
#define SVODIG_A 9
const int OA_MIN_A = 0*SCLR;  // DEG
const int OA_MAX_A = 170*SCLR;  // DEG SHOULD BE same as input limits?
#define SVO_MIN_A 960   // microseconds
#define SVO_MAX_A 2200  // microseconds

// JOINT B, Analog 1 input pin, Digital 5 output pin
#define PIN_B 1
#define POT_MIN_B  1
#define POT_MAX_B 500
const int IA_MIN_B = -130*SCLR;
const int IA_MAX_B = 1*SCLR;
#define SVODIG_B 5
const int OA_MIN_B = -50*SCLR;  // IA_MIN + 90??
const int OA_MAX_B = 90*SCLR;  // IA_MAX - 90??
#define SVO_MIN_B 2300   // microseconds
#define SVO_MAX_B 1000  // microseconds

// JOINT C, COMPUTED INPUT.
//  TUNER POT, Analog 5 input pin, Digital 6 output pin
#define PIN_C 5    // The C potentiometer is for tuning the Claw position.
#define POT_MIN_C 0   // 0 TO 1023
#define POT_MAX_C 1000   // 0 TO 1023
const int IA_MIN_C = -100*SCLR;  // DEG CW (-110) Smaller than actual angle
const int IA_MAX_C = 40*SCLR;  // DEG CCW (40) Smaller than actual angle
#define SVODIG_C 6   // Currently 180 deg servo. Could TO UPGRADE TO 270 DEG SERVO
const int OA_MIN_C = -100*SCLR;
const int OA_MAX_C = 40*SCLR;
#define SVO_MIN_C 2200   // microseconds
#define SVO_MAX_C 500  // microseconds

// JOINT D (CLAW), Analog 2 input pin, Digital 10 output pin
#define PIN_D 2
#define POT_MIN_D  370
#define POT_MAX_D  700
const int IA_MIN_D = 0*SCLR;  // DEG
const int IA_MAX_D = 160*SCLR;  // DEG
#define SVODIG_D 10
const int OA_MIN_D = 0*SCLR;  // DEG
const int OA_MAX_D = 160*SCLR;  // DEG
#define SVO_MIN_D 500   // microseconds
#define SVO_MAX_D 2300  // microseconds

// TURNTABLE, Analog 0 input pin, Digital 11 output pin
#define PIN_T 0
#define POT_MIN_T 130
#define POT_MAX_T 930
const int IA_MIN_T = -90*SCLR;  // DEG
const int IA_MAX_T = 100*SCLR;  // DEG
#define SVODIG_T 11
// straight out is at 0 DEG, XXX microseconds
const int OA_MIN_T = -25*SCLR;  // DEG, was -45
const int OA_MAX_T = 50*SCLR;  // DEG, was 70
#define SVO_MIN_T 500   // microseconds
#define SVO_MAX_T 2300  // microseconds

// ##### GLOBAL VARIABLES #####
Servo servoA,servoB,servoC,servoD,servoT;  // servos for robot arm

struct joint {
  int pot_value;  // analog pin value, int (0 to 1023)
  int pot_min;  // used for tuning the pot
  int pot_max;  // used for tuning the pot
  int pot_angle;  // input device angle
  int arm_angle;  // robot arm angle
  int servo_ms;   // servo command in microseconds
  int servo_ms_prior; // used to look at servo rate of change
};
struct joint jA,jB,jC,jD,jT;

unsigned long millisTime;
//float cx,cy;

joint setup_joint() {
  // Initialize the variables in a joint struct
  struct joint jt;
  jt.pot_angle = 500;
  jt.pot_min = 10000;
  jt.pot_max = 0;
  jt.arm_angle = 0;
  jt.servo_ms = 1000;
  jt.servo_ms_prior = 1000;
  return jt;
}

void setup() {
  #if A_ON
    servoA.attach(SVODIG_A);
  #endif
  #if B_ON
    servoB.attach(SVODIG_B);
  #endif
  #if C_ON
    servoC.attach(SVODIG_C);
  #endif
  #if D_ON
    servoD.attach(SVODIG_D);
  #endif
  #if T_ON
    servoT.attach(SVODIG_T);
  #endif

  // SERIAL OUTPUT AFFECTS SMOOTHNESS OF SERVO PERFORMANCE 
  //  WITH SERIAL ON AND LOW BAUD RATE = JERKY PERFORMANCE
  //   WITH OFF OR HIGH 500000 BAUD RATE = SMOOTH PERFORMANCE
  #if SERIALOUT
    Serial.begin(9600); // baud rate, slower is easier to read
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    } 
  #endif
  
  jA = setup_joint();
  jB = setup_joint();
  jC = setup_joint();
  jD = setup_joint();
  jT = setup_joint(); 
}

void pot_min_max(joint & jt) {
  // Save min and max pot values. Sweep pots, read values.
  // Not needed when Serial is off.
  jt.pot_min = min(jt.pot_value,jt.pot_min); // update min
  jt.pot_max = max(jt.pot_value,jt.pot_max); // update max
}

void pot_map(joint & jt,int fromLow, int fromHigh, int toLow,int toHigh,boolean constr) {
  // map(value, fromLow, fromHigh, toLow, toHigh), uses integer math
  jt.pot_angle = map(jt.pot_value,fromLow,fromHigh,toLow,toHigh); 
  if (constr) {
    jt.pot_angle = constrain(jt.pot_angle,toLow,toHigh);
  }
  jt.arm_angle = jt.pot_angle;  // assume that the two are equal for now
}

void servo_map(joint & jt,int fromLow, int fromHigh, int toLow,int toHigh) {
  // Maps joint angle to the servo microsecond value
  jt.servo_ms = map(jt.arm_angle,fromLow,fromHigh,toLow,toHigh);
}

void log_data(joint jt,char jt_letter,boolean minmax) {
  #if SERIALOUT
    Serial.print(",");
    Serial.print(jt_letter);
    Serial.print(", pot_value,");
    Serial.print(jt.pot_value);
    Serial.print(", POTangle,");
    Serial.print(1.0*jt.pot_angle/SCLR,1);
    Serial.print(", ARMangle,");
    Serial.print(1.0*jt.arm_angle/SCLR,1);
    Serial.print(", servo_ms,");
    Serial.print(jt.servo_ms);
    if (minmax) {
      Serial.print(", min_pot,");
      Serial.print(jt.pot_min);
      Serial.print(", max_pot,");
      Serial.print(jt.pot_max);  
    }
  #endif
}

void loop() {
  #if SERIALOUT
    millisTime = millis();
    // output for debugging
    // Serial.print(val,digits)
    Serial.print("millis,");
    Serial.print(millisTime);
  #endif

  jA.pot_value = analogRead(PIN_A);  // read joint A
  jB.pot_value = analogRead(PIN_B);  // read joint B
  jC.pot_value = analogRead(PIN_C);  // read joint C (tuner pot)
  jD.pot_value = analogRead(PIN_D);  // read the claw
  jT.pot_value = analogRead(PIN_T);  // read the turntable
  
  #if SERIALOUT
    pot_min_max(jA);
    pot_min_max(jB);
    pot_min_max(jC);
    pot_min_max(jD);
    pot_min_max(jT);
  #endif

  pot_map(jA,POT_MIN_A,POT_MAX_A,IA_MIN_A,IA_MAX_A,true);

  pot_map(jB,POT_MIN_B,POT_MAX_B,IA_MIN_B,IA_MAX_B,false);
  jB.arm_angle = constrain((jA.arm_angle + jB.pot_angle),OA_MIN_B,OA_MAX_B);  
  
  // want the claw (joint C) to default to pointed down
  // The C potentiometer is for tuning (adding to) the C position.
  pot_map(jC,POT_MIN_C,POT_MAX_C,IA_MIN_C,IA_MAX_C,false);
  jC.pot_angle = -jC.pot_angle + 10*SCLR;  // REVERSED, TO BE TUNNED
  jC.arm_angle = -90*SCLR -jB.arm_angle + jC.pot_angle;

  // Joint D is the Claw... sorry confusing, but C was taken
  pot_map(jD,POT_MIN_D,POT_MAX_D,IA_MIN_D,IA_MAX_D,true);

  pot_map(jT,POT_MIN_T,POT_MAX_T,IA_MIN_T,IA_MAX_T,true);
  jT.arm_angle = -jT.pot_angle;  // reverse the angle

  // GET SERVO VALUE FROM ROBOT ARM OUTPUT ANGLE
  servo_map(jA,OA_MIN_A,OA_MAX_A,SVO_MIN_A,SVO_MAX_A);
  servo_map(jB,OA_MIN_B,OA_MAX_B,SVO_MIN_B,SVO_MAX_B);  
  servo_map(jC,OA_MIN_C,OA_MAX_C,SVO_MIN_C,SVO_MAX_C); 
  servo_map(jD,OA_MIN_D,OA_MAX_D,SVO_MIN_D,SVO_MAX_D); 
  servo_map(jT,OA_MIN_T,OA_MAX_T,SVO_MIN_T,SVO_MAX_T); 

  // calculate c arm positions from angles
  //cx = lenAB*cos(outputA*1000 / 57296) + lenBC*cos(outputB*1000 / 57296);
  //cy = lenAB*sin(outputA*1000 / 57296) + lenBC*sin(outputB*1000 / 57296);
  
  #if A_ON
    servoA.writeMicroseconds(jA.servo_ms);
  #endif
  #if B_ON
    servoB.writeMicroseconds(jB.servo_ms);
  #endif
  #if C_ON
    servoC.writeMicroseconds(jC.servo_ms);
  #endif
  #if D_ON
    servoD.writeMicroseconds(jD.servo_ms);
  #endif
  #if T_ON
    servoT.writeMicroseconds(jT.servo_ms);
  #endif

  #if SERIALOUT
    log_data(jA,'A',false);
    log_data(jB,'B',false);
    log_data(jC,'C',false);
    log_data(jD,'D',false);
    log_data(jT,'T',false);
    Serial.println(", END");
  #endif
}
