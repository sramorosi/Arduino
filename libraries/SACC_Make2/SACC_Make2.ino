/* ROBOT ARM CONTROL SOFTWARE
 *  By, SrAmo, May 2022
 *  Lesson Learned: Short loop times equals smooth arm performance.
 *  Turning off serial output makes loops time much faster.
 *  Using int rather than float or double can make loop time faster.
 *  Turn off any unnesessary code.
 */
#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#define SERIALOUT true  // Controlls SERIAL output. Turn on when debugging. 
// SERIAL OUTPUT AFFECTS SMOOTHNESS OF SERVO PERFORMANCE 
//  WITH SERIAL true AND LOW 9600 BAUD RATE = JERKY PERFORMANCE
//  WITH false OR HIGH 500000 BAUD RATE = SMOOTH PERFORMANCE

float main_ang_velo = 0.05; // Angular Velocity Limit, DEGREES PER MILLISECOND (~20 is full speed)
// Servos Max Velocity is about 60 deg in 0.12 sec or 460 deg/sec, or 0.46 degrees per millisecond
// This assumes no load and full (7 V) voltage.

// Booleans to turn on Servos. [bad code can damage servos. This can help isolate]
#define A_ON true
#define B_ON true
#define C_ON true
#define D_ON true
#define T_ON true
#define S_ON false

//#define lenAB 50.0     // Length of Input AB arm in mm
//#define lenBC 60.0     // Length of Input BC arm in mm
//#define RADIAN 57.2957795  // number of degrees in one radian

//   **        avoid current overloads           **

// PATHS {{A_angle,B_angle,T_angle},{,,],...}
// path1 is  an elipse (100mm , 200mm) drawn on the floor, from OpenSCAD
//static float path1[][3] ={{34.7317, -37.3821, 0}, {34.775, -37.4545, 8.8903}, {35.3858, -38.4718, 17.1517}, {37.6735, -42.2564, 24.2746}, {42.4945, -50.0908, 29.8915}, {49.887, -61.6998, 33.6901}, {59.3819, -75.7947, 35.2545}, {70.3676, -90.7694, 33.8524}, {81.9162, -104.731, 28.2158}, {91.8898, -115.18, 16.7852}, {96.146, -119.173, 0}, {91.8898, -115.18, -16.7852}, {81.9162, -104.731, -28.2158}, {70.3676, -90.7694, -33.8524}, {59.3819, -75.7947, -35.2545}, {49.887, -61.6998, -33.6901}, {42.4945, -50.0908, -29.8915}, {37.6735, -42.2564, -24.2746}, {35.3858, -38.4718, -17.1517}, {34.775, -37.4545, -8.8903}}
;
static float path1[][3]={{34.7317, -37.3821, 0}, {34.7344, -37.3866, 4.48616}, {34.775, -37.4545, 8.8903}, {34.9463, -37.7399, 13.1351}, {35.3858, -38.4718, 17.1517}, {36.2492, -39.905, 20.8812}, {37.6735, -42.2564, 24.2746}, {39.7464, -45.6491, 27.2905}, {42.4945, -50.0908, 29.8915}, {45.8931, -55.4916, 32.0394}, {49.887, -61.6998, 33.6901}, {54.4076, -68.5327, 34.7871}, {59.3819, -75.7947, 35.2545}, {64.7324, -83.2803, 34.9891}, {70.3676, -90.7694, 33.8524}, {76.161, -98.0153, 31.6655}, {81.9162, -104.731, 28.2158}, {87.3178, -110.58, 23.2932}, {91.8898, -115.18, 16.7852}, {95.0219, -118.145, 8.83744}, {96.146, -119.173, 0}, {95.0219, -118.145, -8.83744}, {91.8898, -115.18, -16.7852}, {87.3178, -110.58, -23.2932}, {81.9162, -104.731, -28.2158}, {76.161, -98.0153, -31.6655}, {70.3676, -90.7694, -33.8524}, {64.7324, -83.2803, -34.9891}, {59.3819, -75.7947, -35.2545}, {54.4076, -68.5327, -34.7871}, {49.887, -61.6998, -33.6901}, {45.8931, -55.4916, -32.0394}, {42.4945, -50.0908, -29.8915}, {39.7464, -45.6491, -27.2905}, {37.6735, -42.2564, -24.2746}, {36.2492, -39.905, -20.8812}, {35.3858, -38.4718, -17.1517}, {34.9463, -37.7399, -13.1351}, {34.775, -37.4545, -8.8903}, {34.7344, -37.3866, -4.48616}}
;
static int path1_size = sizeof(path1)/(3*4);  // sizeof returns bytes in the array.  4 bytes per float. 

// ##### GLOBAL VARIABLES #####
Servo servoA,servoB,servoC,servoD,servoT,servoS;  // servos for robot arm

struct potentiometer {
  int analog_pin; // Arduino analog pin number (0 - 5)
  int low_mv; // low voltage point, from 0 (0 Volts) to 1023 (5 Volts)
  float low_ang; // corresponding angle at low voltage
  int high_mv; // high voltage point, from 0 (0 Volts) to 1023 (5 Volts)
  float high_ang; // corresponding angle at high voltage
  // Note: low and high values do not need to be  at the extreems
  //  Tip:  Pick low and high angles that are easy to read/set/measure
  //      Values outside of the low and high will be extrapolated
};

struct arm_servo {
  // Values are pulse width in microseconds (~400 to ~2400--SERVOS SHOULT BE TESTED)
  // 270 deg servo, gets 290 deg motion with 400 to 2500 microsecond range
  // 180 deg servo, gets 160 deg motion with 500 to 2800 microsecond range
  // 180 deg SAVOX, gets 170 deg motion with 500 to 2800 microsecond range
  int digital_pin; // Arduino digital pin number
  int low_ms; // low microsecond point, from about 500 to 2400
  float low_ang; // corresponding angle at low microsecond
  int high_ms; // high microsecond point, from about 500 to 2400
  float high_ang; // corresponding angle at high microsecond
  // Note: low and high values do not need to be  at the extreems
  //  Tip:  Pick low and high angles that are easy to read/set/measure
  //      Values outside of the low and high will be extrapolated  
}; 

struct joint {
  potentiometer pot; // Sub structure which contains static pot information
  arm_servo svo; // Sub structure which contains the static servo information
  int pot_value;  // current potentiometer value in millivolts 
  float pot_angle; // Potentiometer arm angle, if used
  //int pot_min;  // used for tuning the pot
  //int pot_max;  // used for tuning the pot
  unsigned long previous_millis; // used to find servo rate of change
  float previous_angle;  // used with rate limiting
  float desired_angle;  // angle from input device or array
  int servo_ms;   // servo command in microseconds
  //float previous_velo;  // previous velocity, used to find acceleration
};
struct joint jA,jB,jC,jD,jT,jS;

unsigned long millisTime;

boolean path_init, hold_init, input_arm_init;

// Path_Function global variables
int path_index = 0;
boolean new_segment = true;

//float cx,cy; // to be used for range limits

potentiometer set_pot(int pin, int lowmv, float lowang, int highmv, float highang) {
  // set_pot(pin,lowmv,lowang,highmv,highang)
  // Note: low and high values do not need to be  at the extreems
  struct potentiometer pot;
  pot.analog_pin = pin;
  pot.low_mv = lowmv;
  pot.low_ang = lowang;
  pot.high_mv = highmv;
  pot.high_ang = highang;
  return pot;
}

arm_servo set_servo(int pin, float lowang, int lowms, float highang, int highms) {
  // set_servo(pin,lowang,lowms,highang,highmv)
  // Note: low and high values do not need to be  at the extreems
  struct arm_servo svo;
  svo.digital_pin = pin;
  svo.low_ang = lowang;
  svo.low_ms = lowms;
  svo.high_ang = highang;
  svo.high_ms = highms;
  return svo;
}
  
void set_joint(joint & jt, float initial_angle) {
  // Sets the conversion from Angle to Servo Microseconds
  // Takes an initial angle
  jt.pot_value = 500;  // middle ish
  //jt.pot_min = 10000;  // opposite of min, for tuning
  //jt.pot_max = 0;      // opposite of max, for tuning
  jt.desired_angle = initial_angle;
  jt.previous_angle = initial_angle;
  jt.servo_ms = map(initial_angle,jt.svo.low_ang,jt.svo.high_ang,jt.svo.low_ms,jt.svo.high_ms);
  jt.previous_millis = millis();
}

void log_pot(joint jt) {
    Serial.print(", pot.low_mv,");
    Serial.print(jt.pot.low_mv);
    Serial.print(", pot.high_mv,");
    Serial.print(jt.pot.high_mv);
    Serial.print(", pot.low_ang,");
    Serial.print(jt.pot.low_ang);
    Serial.print(", pot.high_ang,");
    Serial.print(jt.pot.high_ang);
}

void setup() {
  #if SERIALOUT
    Serial.begin(9600); // baud rate, slower is easier to read
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    } 
    Serial.print("path1_size,");
    Serial.print(path1_size);
    Serial.println(",END");
   #endif

  // Set booleans so that all Functions get Initialized
  path_init = true;
  hold_init = true;
  input_arm_init = true;
  
  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(4 ,134,  0.0, 485, 90.0);
  jB.pot = set_pot(1 ,131,-90.0, 500,  0.0);
  jC.pot = set_pot(5 , 139,-90.0, 910, 90.0);
  jD.pot = set_pot(2 ,370, 0.0, 700, 160.0);
  jT.pot = set_pot(0 ,160, -70.0, 510, 0.0);
  jS.pot = set_pot(3 , 0, 0.0, 1023, 280.0);

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  jA.svo = set_servo(9,  0.0, 960, 170.0, 2200);
  jB.svo = set_servo(5, -50.0, 2300, 90.0, 1000); // high to low
  jC.svo = set_servo(6, -90.0, 2400, 90.0, 1060); // high to low
  jD.svo = set_servo(10,  0.0,  500, 160.0, 2300);
  jT.svo = set_servo(11,-70.0,  400, 70.0, 2500);
  jD.svo = set_servo(3,  0.0,  400, 180.0, 2500);

  // INITIALIZATION ANGLES FOR ARM
  set_joint(jA, 110.0);  
  set_joint(jB,  10.0);
  set_joint(jC,  -70.0);
  set_joint(jD,   80.0);
  set_joint(jT,    0.0); 
  set_joint(jS,   90.0);

//  log_pot(jT);
//    Serial.println(",END");

/*
 * servo.attach(pin, min, max)
min (optional): the pulse width, in microseconds, corresponding to the minimum (0 degree) angle on the servo (defaults to 544)
max (optional): the pulse width, in microseconds, corresponding to the maximum (180 degree) angle on the servo (defaults to 2400)
 */

  #if A_ON
    servoA.attach(jA.svo.digital_pin);
  #endif
  #if B_ON
    servoB.attach(jB.svo.digital_pin);
   #endif
  #if C_ON
    servoC.attach(jC.svo.digital_pin,jC.svo.low_ms,jC.svo.high_ms);
   #endif
  #if D_ON
    servoD.attach(jD.svo.digital_pin);
  #endif
  #if T_ON
    servoT.attach(jT.svo.digital_pin,jT.svo.low_ms,jT.svo.high_ms);
  #endif
  #if S_ON
    servoT.attach(jS.svo.digital_pin);
  #endif
}

/*
void pot_min_max(joint & jt) {
  // Save min and max pot values. Sweep pots, read values.
  // Not needed when Serial is off.
  jt.pot_min = min(jt.pot_value,jt.pot_min); // update min
  jt.pot_max = max(jt.pot_value,jt.pot_max); // update max
}
*/
void pot_map(joint & jt) {
  // Map a potentiometer value in millivolts to an angle
  // map(value, fromLow, fromHigh, toLow, toHigh), uses integer math
  jt.pot_angle = map(jt.pot_value, jt.pot.low_mv, jt.pot.high_mv, jt.pot.low_ang, jt.pot.high_ang); 
  
  jt.desired_angle = jt.pot_angle;  // assume that the two are equal for now
}

void servo_map(joint & jt) {
  // Map an arm angle to servo microseconds
  // PLAIN, No Rate Limiting
  // Map joint angle to the servo microsecond value
  jt.servo_ms = map(jt.desired_angle, jt.svo.low_ang, jt.svo.high_ang, jt.svo.low_ms, jt.svo.high_ms);
  /*
  if (toLow < toHigh) {
    jt.servo_ms = constrain(jt.servo_ms,toLow,toHigh);
  } else {
    jt.servo_ms = constrain(jt.servo_ms,toHigh,toLow);
  }
  */
}

void servo_map_with_limits(joint & jt, float rate) {
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
  jt.servo_ms = map(jt.previous_angle, jt.svo.low_ang, jt.svo.high_ang, jt.svo.low_ms, jt.svo.high_ms);
 
  /*
  if (toLow < toHigh) {
    jt.servo_ms = constrain(jt.servo_ms,toLow,toHigh);
  } else {
    jt.servo_ms = constrain(jt.servo_ms,toHigh,toLow);
  }
  */
  
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
//    Serial.print(", PrevAngle,");
//    Serial.print(jt.previous_angle,1);
    Serial.print(", DESAngle,");
    Serial.print(jt.desired_angle,1);
//    Serial.print(", servo_ms,");
//    Serial.print(jt.servo_ms);
/*    if (minmax) {
      Serial.print(", min_pot,");
      Serial.print(jt.pot_min);
      Serial.print(", max_pot,");
      Serial.print(jt.pot_max);  
    }  */
  #endif
}

boolean update_done(joint jt1,joint jt2,joint jt3){
  float error1,error2,error3;
  float is_zero = 0.1;  // USED TO DETERMINE ZERO, DEGREES

  error1 = abs(jt1.desired_angle-jt1.previous_angle);
  error2 = abs(jt2.desired_angle-jt2.previous_angle);
  error3 = abs(jt3.desired_angle-jt3.previous_angle);
  
  if (error1 < is_zero && error2 < is_zero && error3 < is_zero) {
      #if SERIALOUT
        Serial.print(",ud-done,");
      #endif
    return true;
  } else {
        #if SERIALOUT
          Serial.print(",ud-NOT-done,");
        #endif
    return false;
  }
}

void path_loop() {
  // reads the path array and moves the arm
  if (path_init) {
    // first time in this function, do initialize
    //  and set the other function to require initialize
    path_init=false; 
    hold_init=true;  
    input_arm_init=true;
    
     // initialize joints
    jA.desired_angle = 120.0;
    jB.desired_angle = 20.0;
    jC.desired_angle = -70.0;
    jD.desired_angle = 80.0;
    jT.desired_angle = 0.0;  

    main_ang_velo = 0.02;
    path_index = 0;
    new_segment = true;

  } else if (update_done(jA,jB,jT)) {
        if (path_index < path1_size-1) { // step through the array
          path_index +=1;
          jA.pot_angle=path1[path_index][0];
          jA.desired_angle=jA.pot_angle;
          
          jB.pot_angle=path1[path_index][1];
          jB.desired_angle = constrain((jA.desired_angle + jB.pot_angle), jB.svo.low_ang, jB.svo.high_ang);  
          
          jT.pot_angle=path1[path_index][2];
          jT.desired_angle=jT.pot_angle;
        } else { // Done with array, restart the array
          path_index = 0;
          new_segment = true;
          //path_init = true;        
        }
      } else {
        // LOOP UNTIL THE ANGLES ARE MET... UNTIL DONE ROUTINE        
      }
    }

void hold_loop() {
  // F2 - HOLD
  if (hold_init) {
    // first time in this function, do initialize
    //  and set the other function to require initialize
    path_init=true; 
    hold_init=false;  
    input_arm_init=true;

    main_ang_velo = 0.1; // set the max velocity
    
  } else {
     // initialize joints
    jA.desired_angle = 110.0;
    jB.desired_angle = 10.0;
    jC.desired_angle = -70.0;
    jD.desired_angle = 80.0;
    jT.desired_angle = 0.0;  
  }
}

void input_arm_loop() {
  // Map potentiometer values to Angles and Convert to Robot arm angles
  if (input_arm_init) {
    // first time in this function, do initialize
    //  and set the other function to require initialize
    path_init=true; 
    hold_init=true;  
    input_arm_init=false;

    main_ang_velo = 0.05;
    
  } else {
    // main loop
    pot_map(jA);
  
    pot_map(jB);
    jB.desired_angle = constrain((jA.desired_angle + jB.pot_angle), jB.svo.low_ang, jB.svo.high_ang);  
    // Turntable
    pot_map(jT);
    jT.desired_angle = -jT.pot_angle;  // reverse the angle
    
    // calculate c arm positions from angles
    //cx = lenAB*cos(outputA*1000 / 57296) + lenBC*cos(outputB*1000 / 57296);
    //cy = lenAB*sin(outputA*1000 / 57296) + lenBC*sin(outputB*1000 / 57296);
      
  }
}

void loop() {
  //########### MAIN LOOP ############

  // reads the pots
  jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
  jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
  jC.pot_value = analogRead(jC.pot.analog_pin);  // read joint C (tuner pot)
  jD.pot_value = analogRead(jD.pot.analog_pin);  // read the claw
  jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
  jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector

  #if SERIALOUT
    // output for debugging
    // Serial.print(val,digits)
    millisTime = millis();
    Serial.print("millis,");
    Serial.print(millisTime);
/*
    pot_min_max(jA);
    pot_min_max(jB);
    pot_min_max(jC);
    pot_min_max(jD);
    pot_min_max(jT);
*/
  #endif

  pot_map(jS);  // get Selector angle

  if (jS.pot_value < 400) {
    // Move the arm using the path array
    #if SERIALOUT
      Serial.print(",<400-PATH,");
    #endif
    path_loop();
  } else if (jS.pot_value < 600) {
    // Hold arm in initialize position
    #if SERIALOUT
      Serial.print(",400-600-HOLD,");
    #endif
    hold_loop();
  } else {
    // Manually control arm using input arm
    #if SERIALOUT
      Serial.print(",>600-MANUAL,");
    #endif
    input_arm_loop();    
  }
  // want Joint C to have a fix angle relative to ground, thus driven by joint B.
  // The C potentiometer is for tuning (adding to) the C position.
  pot_map(jC);
  jC.pot_angle = -jC.pot_angle;  // REVERSED
  jC.desired_angle = jC.pot_angle -jB.desired_angle - 30.0;

  // Joint D is the Claw... sorry confusing, but C was taken
  pot_map(jD);

  // GET SERVO Pulse width VALUES FROM ROBOT ARM OUTPUT ANGLE
  servo_map_with_limits(jA, main_ang_velo);
  servo_map_with_limits(jB, main_ang_velo);  
  servo_map_with_limits(jC, main_ang_velo); 
  servo_map(jD);  // full speed on servo claw

  // Turntable is geared down 2:1 
  servo_map_with_limits(jT, main_ang_velo); 

  // No S servo attached
  //servo_map_with_limits(jS,main_ang_velo); 

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
  /*
  #if S_ON
    servoT.writeMicroseconds(jS.servo_ms);
  #endif
*/
  #if SERIALOUT
    //log_data(jA,'A',false);
    //log_data(jB,'B',false);
    log_data(jC,'C',false);
    log_data(jD,'D',false);
    //log_data(jT,'T',false);
    //log_pot(jT);
    log_data(jS,'S',false);
    Serial.println(", END");
  #endif
}
