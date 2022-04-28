/* ROBOT ARM CONTROL SOFTWARE
 *  By, SrAmo, May 2022
 *  Lesson Learned: Short loop times equals smooth arm performance.
 *  Turning off serial output makes loops time much faster.
 *  Using int rather than float or double makes loop time faster.
 *  Turn off any unnesessary code.
 */
#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#define SERIALOUT false  // Controlls SERIAL output. Turn on when debugging. 
// SERIAL OUTPUT AFFECTS SMOOTHNESS OF SERVO PERFORMANCE 
//  WITH SERIAL true AND LOW 9600 BAUD RATE = JERKY PERFORMANCE
//  WITH false OR HIGH 500000 BAUD RATE = SMOOTH PERFORMANCE

const float V_MAX = 5.0; // Angular Velocity Limit, DEGREES PER MILLISECOND (~20 is full speed)

// Angular Acceleration Limit. This does not seem to work. Acceleration switches sign frequently.
//const float A_MAX = 0.1; //DEGREES PER MILLISECOND^2  ACCELERATION LIMITING DOESN'T WORK

// Switched to using float rather than int for angles
//#define SCLR 100  // scaler used to improve accuracy of int (angle) variables

// Booleans to turn on Servos. [bad code can damage servos. This can help isolate]
#define A_ON true
#define B_ON true
#define C_ON true
#define D_ON true
#define T_ON true
#define S_ON false

// SERVO RATE LIMITING, for smooth operation and prevent damate
// The limit that a servo can change in a unit time
//  Servo delta MicroSecond *1000 / delta time milliseonds
// Typical values are  2 ms * 1000 / 25 milliseconds = 80
//#define SVO_RATE_LIMIT 2

//#define lenAB 50.0     // Length of Input AB arm in mm
//#define lenBC 60.0     // Length of Input BC arm in mm
//#define RADIAN 57.2957795  // number of degrees in one radian

// CONSTANTS:
// Use PIN_ constants to identify the analog pin (0-5) for the Potentiometer.
// Use POT_ constants to set potetiometer ranges from 0 (0 Volts) to 1023 (5 Volts)
// Use IA_ constants to map the Input Angle (deg)for a given potentiometer value
// Compute Output Arm Angles from INPUTS and map() to compute value for servos.
// Use SVODIG_ to identify the digital pin that the servo is attached to.
// Use SVO_ constants to set servo range in microseconds. Also used to Constrain output.
// Values are microseconds (~400 to ~2400--SERVOS MUST BE TESTED), REGARDLESS OF SERVO RANGE
// 270 deg servo, gets 290 deg motion with 400 to 2500 microsecond range
// 180 deg servo, gets 160 deg motion with 500 to 2800 microsecond range
// 180 deg SAVOX, gets 170 deg motion with 500 to 2800 microsecond range
//   **        avoid current overloads           **

// JOINT A, Analog 4 input pin, Digital 9 output
#define PIN_A 4
#define POT_MIN_A 134
#define POT_MAX_A 870
const float IA_MIN_A = 2.0;
const float IA_MAX_A = 175.0;
#define SVODIG_A 9
const float OA_MIN_A = 0.0;  // DEG
const float OA_MAX_A = 170.0;  // DEG SHOULD BE same as input limits?
#define SVO_MIN_A 960   // microseconds
#define SVO_MAX_A 2200  // microseconds

// JOINT B, Analog 1 input pin, Digital 5 output pin
#define PIN_B 1
#define POT_MIN_B  1
#define POT_MAX_B 500
const float IA_MIN_B = -130.0;
const float IA_MAX_B = 1.0;
#define SVODIG_B 5
const float OA_MIN_B = -50.0;  // IA_MIN + 90??
const float OA_MAX_B = 90.0;  // IA_MAX - 90??
#define SVO_MIN_B 2300   // microseconds  BAD REVERSE
#define SVO_MAX_B 1000  // microseconds

// JOINT C, COMPUTED INPUT.
//  TUNER POT, Analog 5 input pin, Digital 6 output pin
#define PIN_C 5    // The C potentiometer is for tuning the Claw position.
#define POT_MIN_C 0   // 0 TO 1023
#define POT_MAX_C 1000   // 0 TO 1023
const float IA_MIN_C = -90.0;  // DEG CW (-110) Smaller than actual angle
const float IA_MAX_C = 90.0;  // DEG CCW (40) Smaller than actual angle
#define SVODIG_C 6   // Currently 180 deg servo. Could TO UPGRADE TO 270 DEG SERVO
const float OA_MIN_C = -90.0; // gets reversed
const float OA_MAX_C = 90.0;
#define SVO_MIN_C 2400   // microseconds   BAD REVERSE
#define SVO_MAX_C 1060  // microseconds

// JOINT D (CLAW), Analog 2 input pin, Digital 10 output pin
#define PIN_D 2
#define POT_MIN_D  370
#define POT_MAX_D  700
const float IA_MIN_D = 0.0;  // DEG
const float IA_MAX_D = 160.0;  // DEG
#define SVODIG_D 10
const float OA_MIN_D = 0.0;  // DEG
const float OA_MAX_D = 160.0;  // DEG
#define SVO_MIN_D 500   // microseconds
#define SVO_MAX_D 2300  // microseconds

// TURNTABLE, Analog 0 input pin, Digital 11 output pin
#define PIN_T 0
#define POT_MIN_T 130
#define POT_MAX_T 930
const float IA_MIN_T = -70.0;  // DEG
const float IA_MAX_T = 80.0;  // DEG
#define SVODIG_T 11
// straight out is at 0 DEG, XXX microseconds
const float OA_MIN_T = -25.0;  // DEG, was -45
const float OA_MAX_T = 50.0;  // DEG, was 70
#define SVO_MIN_T 400   // microseconds
#define SVO_MAX_T 2500  // microseconds

// SELECTOR, FUTURE IMPLEMENTATION. Analog 3 input pin
#define PIN_S 3
#define POT_MIN_S 130
#define POT_MAX_S 930
const float IA_MIN_S = 0.0;  // DEG
const float IA_MAX_S = 270.0;  // DEG
#define SVODIG_S 3
#define SVO_MIN_S 400   // microseconds  (RANGE = 290 DEG!)
#define SVO_MAX_S 2500  // microseconds  (RANGE = 290 DEG!)

// PATHS {{A_angle,B_angle,T_angle},{,,],...}
// path1 is  an elipse (100mm , 200mm) drawn on the floor, from OpenSCAD
static float path1[][3] ={{34.7317, -37.3821, 0}, {34.775, -37.4545, 8.8903}, {35.3858, -38.4718, 17.1517}, {37.6735, -42.2564, 24.2746}, {42.4945, -50.0908, 29.8915}, {49.887, -61.6998, 33.6901}, {59.3819, -75.7947, 35.2545}, {70.3676, -90.7694, 33.8524}, {81.9162, -104.731, 28.2158}, {91.8898, -115.18, 16.7852}, {96.146, -119.173, 0}, {91.8898, -115.18, -16.7852}, {81.9162, -104.731, -28.2158}, {70.3676, -90.7694, -33.8524}, {59.3819, -75.7947, -35.2545}, {49.887, -61.6998, -33.6901}, {42.4945, -50.0908, -29.8915}, {37.6735, -42.2564, -24.2746}, {35.3858, -38.4718, -17.1517}, {34.775, -37.4545, -8.8903}}
;

// ##### GLOBAL VARIABLES #####
Servo servoA,servoB,servoC,servoD,servoT,servoS;  // servos for robot arm

struct joint {
  int pot_value;  // analog pin value, int (0 to 1023)
  int pot_min;  // used for tuning the pot
  int pot_max;  // used for tuning the pot
  int servo_ms;   // servo command in microseconds
  float pot_angle;
  float desired_angle;  // angle from input device or array
  float previous_angle;  // used with control loop
  //float previous_velo;  // previous velocity, used to find acceleration
  unsigned long previous_millis; // used to look at servo rate of change
};
struct joint jA,jB,jC,jD,jT,jS;

unsigned long millisTime;

boolean f1_init, f2_init, f3_init;

// Function 1 global variables
int f1_index = 0;
boolean new_segment = true;

//float cx,cy;

joint setup_joint(int initial_ms) {
  // Initialize the variables in a joint struct
  struct joint jt;
  jt.pot_value = 500;  // middle ish
  jt.pot_min = 10000;  // opposite of min, for tuning
  jt.pot_max = 0;      // opposite of max, for tuning
  jt.desired_angle = 0,0;
  jt.previous_angle = 0.0;
  //jt.previous_velo = 0.0;
  jt.servo_ms = initial_ms;
  jt.previous_millis = millis();
  return jt;
}

void setup() {
  #if SERIALOUT
    Serial.begin(9600); // baud rate, slower is easier to read
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    } 
  #endif

  // Set so that Functions get Initialized
  f1_init = true;
  f2_init = true;
  f3_init = true;
  
  // initialize joints
  jA = setup_joint(1900);
  jB = setup_joint(1800);
  jC = setup_joint(2100);
  jD = setup_joint(1300);
  jT = setup_joint(1300); 
  jS = setup_joint(1300);

/*
 * servo.attach(pin, min, max)
min (optional): the pulse width, in microseconds, corresponding to the minimum (0 degree) angle on the servo (defaults to 544)
max (optional): the pulse width, in microseconds, corresponding to the maximum (180 degree) angle on the servo (defaults to 2400)
 */

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
    servoT.attach(SVODIG_T,SVO_MIN_T,SVO_MAX_T);
  #endif
  #if S_ON
    servoT.attach(SVODIG_S,SVO_MIN_S,SVO_MAX_S);
  #endif
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
  jt.desired_angle = jt.pot_angle;  // assume that the two are equal for now
}

void servo_map(joint & jt,int fromLow, int fromHigh, int toLow,int toHigh) {
  // PLAIN, No Rate Limiting
  // Map joint angle to the servo microsecond value
  jt.servo_ms = map(jt.desired_angle,fromLow,fromHigh,toLow,toHigh);
  
  if (toLow < toHigh) {
    jt.servo_ms = constrain(jt.servo_ms,toLow,toHigh);
  } else {
    jt.servo_ms = constrain(jt.servo_ms,toHigh,toLow);
  }
  
}
void servo_map_with_limits(joint & jt,int fromLow, int fromHigh, int toLow,int toHigh,float rate) {
// WITH rate limiting
// SERVO RATE LIMITING, for smooth operation and prevent damage
// Limit how much a servo can change in a unit time

  int dt;
  float current_velo;
  
  dt = jt.previous_millis - millis();

  current_velo = (jt.desired_angle-jt.previous_angle)/dt;
//  current_accel = (current_velo-jt.previous_velo)/dt;
//  jt.previous_velo = current_velo;
  
  if (current_velo > rate) {
    jt.previous_angle += rate*dt;
  } else if (-current_velo > rate) {
    jt.previous_angle -= rate*dt;
  } else {
    jt.previous_angle = jt.desired_angle;
  }

  // Map joint angle to the servo microsecond value
  jt.servo_ms = map(jt.previous_angle,fromLow,fromHigh,toLow,toHigh);
 

  if (toLow < toHigh) {
    jt.servo_ms = constrain(jt.servo_ms,toLow,toHigh);
  } else {
    jt.servo_ms = constrain(jt.servo_ms,toHigh,toLow);
  }
  
  jt.previous_millis = millis();
}

void log_data(joint jt,char jt_letter,boolean minmax) {
  #if SERIALOUT
    Serial.print(",");
    Serial.print(jt_letter);
    Serial.print(", pot_value,");
    Serial.print(jt.pot_value);
    Serial.print(", POTangle,");
    Serial.print(jt.pot_angle,1);
    Serial.print(", ARMangle,");
    Serial.print(jt.desired_angle,1);
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

boolean update_done(joint jt1,joint jt2,joint jt3){
  float error1,error2,error3;

  error1 = abs(jt1.desired_angle-jt1.previous_angle);
  error2 = abs(jt2.desired_angle-jt2.previous_angle);
  error3 = abs(jt3.desired_angle-jt3.previous_angle);
  
  if (error1 < 1.0 && error2 < 1.0 && error3 < 1.0) {
    return true;
  } else {
    return false;
  }
}

void f2_loop() {
  // Map potentiometer values to Angles and Convert to Robot arm angles
  if (f2_init) {
    // main loop
    pot_map(jA,POT_MIN_A,POT_MAX_A,IA_MIN_A,IA_MAX_A,true);
  
    pot_map(jB,POT_MIN_B,POT_MAX_B,IA_MIN_B,IA_MAX_B,false);
    jB.desired_angle = constrain((jA.desired_angle + jB.pot_angle),OA_MIN_B,OA_MAX_B);  
    
    // want the claw (joint C) to default to pointed down
    // The C potentiometer is for tuning (adding to) the C position.
    pot_map(jC,POT_MIN_C,POT_MAX_C,IA_MIN_C,IA_MAX_C,false);
    jC.pot_angle = -jC.pot_angle+ 60.0;  // REVERSED, TO BE TUNNED
    jC.desired_angle = -90.0 -jB.desired_angle + jC.pot_angle;
    //jC.desired_angle =  jC.pot_angle;
  
    // Joint D is the Claw... sorry confusing, but C was taken
    pot_map(jD,POT_MIN_D,POT_MAX_D,IA_MIN_D,IA_MAX_D,true);
  
    // Turntable
    pot_map(jT,POT_MIN_T,POT_MAX_T,IA_MIN_T,IA_MAX_T,true);
    jT.desired_angle = -jT.pot_angle;  // reverse the angle
  
    // Selector
    pot_map(jS,POT_MIN_S,POT_MAX_S,IA_MIN_S,IA_MAX_S,false);
    
    // GET SERVO VALUE FROM ROBOT ARM OUTPUT ANGLE
    servo_map_with_limits(jA,OA_MIN_A,OA_MAX_A,SVO_MIN_A,SVO_MAX_A,V_MAX);
    servo_map_with_limits(jB,OA_MIN_B,OA_MAX_B,SVO_MIN_B,SVO_MAX_B,V_MAX);  
    servo_map_with_limits(jC,OA_MIN_C,OA_MAX_C,SVO_MIN_C,SVO_MAX_C,V_MAX); 
    servo_map(jD,OA_MIN_D,OA_MAX_D,SVO_MIN_D,SVO_MAX_D); 
    servo_map_with_limits(jT,OA_MIN_T,OA_MAX_T,SVO_MIN_T,SVO_MAX_T,V_MAX); 
    servo_map_with_limits(jS,IA_MIN_S,IA_MAX_S,SVO_MIN_S,SVO_MAX_S,V_MAX); 
  
    // calculate c arm positions from angles
    //cx = lenAB*cos(outputA*1000 / 57296) + lenBC*cos(outputB*1000 / 57296);
    //cy = lenAB*sin(outputA*1000 / 57296) + lenBC*sin(outputB*1000 / 57296);
      
  } else {
    // initialize
    f2_init = true;
    f1_init = false;
    f3_init = false;
    
     // initialize joints
    jA = setup_joint(1900);
    jB = setup_joint(1600);
    jC = setup_joint(2100);
    jD = setup_joint(1300);
    jT = setup_joint(1300); 
    jS = setup_joint(1300);  
  }
}

void f1_loop(int velocity) {
  // reads the path array and moves the arm
  if (f1_init) {
    // main loop
    if (new_segment) {
      // INITIALIZE, DO THIS ONCE PER SEGMENT
      jA.pot_angle=path1[f1_index][0];
      jA.desired_angle=jA.pot_angle;
      
      jB.pot_angle=path1[f1_index][1];
      jB.desired_angle = constrain((jA.desired_angle + jB.pot_angle),OA_MIN_B,OA_MAX_B);  
      
      jT.pot_angle=path1[f1_index][2];
      jT.desired_angle=jT.pot_angle;

      new_segment = false;
    } else {
      if (update_done(jA,jB,jT)) {
        if (f1_index < path1) {
          f1_index +=1;
          new_segment = true;
        } else {
          f1_init = false;        
        }
      } else {
        // DO THIS UNTIL THE ANGLES ARE MET... UNTIL DONE ROUTINE
        // GET SERVO VALUE FROM ROBOT ARM OUTPUT ANGLE
        servo_map_with_limits(jA,OA_MIN_A,OA_MAX_A,SVO_MIN_A,SVO_MAX_A,velocity);
        servo_map_with_limits(jB,OA_MIN_B,OA_MAX_B,SVO_MIN_B,SVO_MAX_B,velocity);  
        servo_map_with_limits(jT,OA_MIN_T,OA_MAX_T,SVO_MIN_T,SVO_MAX_T,velocity);         
      }
    }
  } else {
    // initialize
    f1_init = true;
    f1_index = 0;
    new_segment = true;
    
    f2_init = false;
    f3_init = false;
    
     // initialize joints
    jA = setup_joint(1900);
    jB = setup_joint(1600);
    jC = setup_joint(2100);
    jD = setup_joint(1300);
    jT = setup_joint(1300); 
    jS = setup_joint(1300);  
  }
}

void f3_loop() {
  // reads the path array and moves the arm
  if (f3_init) {
    // main loop - do nothing
  } else {
    // initialize
    f1_init = false;
    f2_init = false;
    f3_init = true;
    
     // initialize joints
    jA = setup_joint(1900);
    jB = setup_joint(1600);
    jC = setup_joint(2100);
    jD = setup_joint(1300);
    jT = setup_joint(1300); 
    jS = setup_joint(1300);  
  }
}

void loop() {
  //########### MAIN LOOP ############

  // reads the pots
  jA.pot_value = analogRead(PIN_A);  // read joint A
  jB.pot_value = analogRead(PIN_B);  // read joint B
  jC.pot_value = analogRead(PIN_C);  // read joint C (tuner pot)
  jD.pot_value = analogRead(PIN_D);  // read the claw
  jT.pot_value = analogRead(PIN_T);  // read the turntable
  jS.pot_value = analogRead(PIN_S);  // read the selector

  #if SERIALOUT
    // output for debugging
    // Serial.print(val,digits)
    millisTime = millis();
    Serial.print("millis,");
    Serial.print(millisTime);

    pot_min_max(jA);
    pot_min_max(jB);
    pot_min_max(jC);
    pot_min_max(jD);
    pot_min_max(jT);
  #endif

  if (jS.pot_value > 400) {
    // Function 1 = draw a path
    f1_loop(5.0);
  } if (jS.pot_value < 600) {
    // Function 2 = manual arm control
    f2_loop();    
  } else {
    // Function 3 = hold in initialize position
    f3_loop();
  }

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
  #if S_ON
    servoT.writeMicroseconds(jS.servo_ms);
  #endif

  #if SERIALOUT
    log_data(jA,'A',false);
    log_data(jB,'B',false);
    //log_data(jC,'C',false);
    //log_data(jD,'D',false);
    //log_data(jT,'T',false);
    //log_data(jS,'S',false);
    Serial.println(", END");
  #endif
}
