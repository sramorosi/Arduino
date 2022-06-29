/* ROBOT ARM CONTROL SOFTWARE
 *  By, SrAmo, May 2022
 *  Lesson Learned: Short loop times equals smooth arm performance.
 *  Turning off serial output makes loops time much faster.
 *  Using int rather than float or double can make loop time faster.
 *  Turn off any unnesessary code.
 */
//#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERIALOUT false  // Controlls SERIAL output. Turn on when debugging. 
// SERIAL OUTPUT AFFECTS SMOOTHNESS OF SERVO PERFORMANCE 
//  WITH SERIAL true AND LOW 9600 BAUD RATE = JERKY PERFORMANCE
//  WITH false OR HIGH 500000 BAUD RATE = SMOOTH PERFORMANCE

float main_ang_velo = 0.07; // Angular Velocity Limit, DEGREES PER MILLISECOND (~20 is full speed)
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

// PATHS {{A_angle,B_angle,T_angle},{,,],...}

// path1 is  an elipse drawn parallel to horizon
static float path1[][3]={{47.0282, -38.0823, 0}, {46.9841, -38.0134, 4.84945}, {46.8869, -37.8616, 9.60769}, {46.8385, -37.786, 14.1898}, {46.9953, -38.031, 18.5216}, {47.5436, -38.8852, 22.5422}, {48.6615, -40.6161, 26.2034}, {50.4793, -43.4001, 29.4666}, {53.0571, -47.2821, 32.2985}, {56.3897, -52.1843, 34.6659}, {60.4295, -57.9462, 36.5289}, {65.1092, -64.3659, 37.8335}, {70.355, -71.2263, 38.5035}, {76.0871, -78.3029, 38.4289}, {82.2084, -85.3612, 37.4534}, {88.5794, -92.1483, 35.3606}, {94.9763, -98.3838, 31.868}, {101.034, -103.756, 26.6533}, {106.198, -107.931, 19.4622}, {109.754, -110.596, 10.3547}, {111.035, -111.513, 0}, {109.754, -110.596, -10.3547}, {106.198, -107.931, -19.4622}, {101.034, -103.756, -26.6533}, {94.9763, -98.3838, -31.868}, {88.5794, -92.1483, -35.3606}, {82.2084, -85.3612, -37.4534}, {76.0871, -78.3029, -38.4289}, {70.355, -71.2263, -38.5035}, {65.1092, -64.3659, -37.8335}, {60.4295, -57.9462, -36.5289}, {56.3897, -52.1843, -34.6659}, {53.0571, -47.2821, -32.2985}, {50.4793, -43.4001, -29.4666}, {48.6615, -40.6161, -26.2034}, {47.5436, -38.8852, -22.5422}, {46.9953, -38.031, -18.5216}, {46.8385, -37.786, -14.1898}, {46.8869, -37.8616, -9.60769}, {46.9841, -38.0134, -4.84945}}
;
static int path1_size = sizeof(path1)/(3*4);  // sizeof returns bytes in the array.  4 bytes per float. 

// path2 is a line along the y axis
static float path2[][3]={{51.1959, -44.4871, -56.3099}, {56.0625, -51.7088, -54.9406}, {60.4295, -57.9462, -53.4711}, {64.4477, -63.4755, -51.8924}, {68.2014, -68.4536, -50.1944}, {71.7411, -72.9782, -48.3665}, {75.0977, -77.1133, -46.3972}, {78.2893, -80.9022, -44.2748}, {81.3248, -84.3746, -41.9872}, {84.2057, -87.5507, -39.5226}, {86.9275, -90.4438, -36.8699}, {89.4807, -93.0621, -34.0193}, {91.8511, -95.4101, -30.9638}, {94.0208, -97.4893, -27.6995}, {95.9688, -99.2993, -24.2277}, {97.6722, -100.838, -20.556}, {99.1075, -102.103, -16.6992}, {100.252, -103.091, -12.6804}, {101.086, -103.799, -8.53077}, {101.592, -104.225, -4.28915}, {101.763, -104.367, 0}, {101.592, -104.225, 4.28915}, {101.086, -103.799, 8.53077}, {100.252, -103.091, 12.6804}, {99.1075, -102.103, 16.6992}, {97.6722, -100.838, 20.556}, {95.9688, -99.2993, 24.2277}, {94.0208, -97.4893, 27.6995}, {91.8511, -95.4101, 30.9638}, {89.4807, -93.0621, 34.0193}, {86.9275, -90.4438, 36.8699}, {84.2057, -87.5507, 39.5226}, {81.3248, -84.3746, 41.9872}, {78.2893, -80.9022, 44.2748}, {75.0977, -77.1133, 46.3972}, {71.7411, -72.9782, 48.3665}, {68.2014, -68.4536, 50.1944}, {64.4477, -63.4755, 51.8924}, {60.4295, -57.9462, 53.4711}, {56.0625, -51.7088, 54.9406}}
;
static int path2_size = sizeof(path2)/(3*4);  // sizeof returns bytes in the array.  4 bytes per float. 
boolean forward = true; // used with path2 to reverse direction

// ##### GLOBAL VARIABLES #####
//Servo servoA,servoB,servoC,servoD,servoT,servoS;  // servos for robot arm

struct potentiometer {
  int analog_pin; // Arduino analog pin number (0 - 5)
  int low_mv; // low voltage point, from 0 (0 Volts) to 1023 (5 Volts)
  int low_ang; // corresponding angle at low voltage
  int high_mv; // high voltage point, from 0 (0 Volts) to 1023 (5 Volts)
  int high_ang; // corresponding angle at high voltage
  // Note: low and high values do not need to be  at the extreems
  //  Tip:  Pick low and high angles that are easy to read/set/measure
  //      Values outside of the low and high will be extrapolated
};

struct arm_servo {
  // Values are pulse width in microseconds (~400 to ~2400--SERVOS SHOULT BE TESTED)
  // 270 deg servo, gets 290 deg motion with 400 to 2500 microsecond range
  // 180 deg servo, gets 160 deg motion with 500 to 2800 microsecond range
  // 180 deg SAVOX, gets 170 deg motion with 500 to 2800 microsecond range
  //int digital_pin; // Arduino digital pin number
  uint8_t digital_pin; // Adafruit digital pin number
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

boolean path1_init,path2_init, hold_init, input_arm_init;

// Path_Function global variables
int path_index = 0;
boolean new_segment = true;

//float cx,cy; // to be used for range limits

potentiometer set_pot(int pin, int lowmv, int lowang, int highmv, int highang) {
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
  path1_init = true;
  hold_init = true;
  input_arm_init = true;
  
  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(4 ,134,  0, 485, 90);
  jB.pot = set_pot(1 ,131,-90, 500,  0);
  jC.pot = set_pot(5 , 139,-90, 910, 90);
  jD.pot = set_pot(2 ,370, 0, 700, 160);
  jT.pot = set_pot(0 ,160, -70, 510, 0);
  jS.pot = set_pot(3 , 0, 0, 1023, 280);

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  jA.svo = set_servo(9,  0.0, 960, 170.0, 2200);
  jB.svo = set_servo(5, -50.0, 2300, 90.0, 1000); // high to low
  jC.svo = set_servo(6, -90.0, 2400, 90.0, 1060); // high to low
  jD.svo = set_servo(10,  0.0,  500, 160.0, 2300);
  jT.svo = set_servo(11,-70.0,  400, 70.0, 2500);
  //jS.svo = set_servo(3,  0.0,  400, 180.0, 2500);

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

/*
  #if A_ON
    servoA.attach(jA.svo.digital_pin);
  #endif
  #if B_ON
    servoB.attach(jB.svo.digital_pin);
   #endif
  #if C_ON
    servoC.attach(jC.svo.digital_pin, 400 , 2500);
   #endif
  #if D_ON
    servoD.attach(jD.svo.digital_pin);
  #endif
  #if T_ON
    servoT.attach(jT.svo.digital_pin, 400 , 2500);
  #endif
  #if S_ON
    servoT.attach(jS.svo.digital_pin);
  #endif
*/

  pwm.begin();
  /*  Adafruit sevo library
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

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
  // NOTE: SCALE ANGLES *10 THEN DIVIDE BY 10.0 TO GET 0.1 PRECISION FROM POT VALUES
  jt.pot_angle = map(jt.pot_value, jt.pot.low_mv, jt.pot.high_mv, jt.pot.low_ang*10, jt.pot.high_ang*10) / 10.0; 
  
  jt.desired_angle = jt.pot_angle;  // assume that the two are equal for now
}

void servo_map(joint & jt) {
  // Map an arm angle to servo microseconds
  // PLAIN, No Rate Limiting
  // Map joint angle to the servo microsecond value
  jt.servo_ms = map(jt.desired_angle, jt.svo.low_ang, jt.svo.high_ang, jt.svo.low_ms, jt.svo.high_ms);

  //jt.servo_ms = map(jt.previous_angle, jt.svo.low_ang, jt.svo.high_ang, jt.svo.low_ms, jt.svo.high_ms);

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
  float is_zero = 0.05;  // USED TO DETERMINE ZERO, DEGREES

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

void path1_loop() {
  // reads the path array and moves the arm
  if (path1_init) {
    // first time in this function, do initialize
    //  and set the other function to require initialize
    path1_init=false; 
    path2_init=true;
    hold_init=true;  
    input_arm_init=true;
    
     // initialize joints
    jA.desired_angle = 80.0;
    jB.desired_angle = 20.0;
    jC.desired_angle = -70.0;
    jD.desired_angle = 80.0;
    jT.desired_angle = 0.0;  

    //main_ang_velo = 0.02;
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
        }
      } else {
        // LOOP UNTIL THE ANGLES ARE MET... UNTIL DONE ROUTINE        
      }
    }

void path2_loop() {
  // reads the path array and moves the arm
  if (path2_init) {
    // first time in this function, do initialize
    //  and set the other function to require initialize
    path2_init=false; 
    path1_init=true;
    hold_init=true;  
    input_arm_init=true;
    forward=true;  // direction of array traverse
    
     // initialize joints
    jA.desired_angle = 80.0;
    jB.desired_angle = 20.0;
    jC.desired_angle = -70.0;
    jD.desired_angle = 80.0;
    jT.desired_angle = 0.0;  

    main_ang_velo = 0.05;
    path_index = 0;
    new_segment = true;

  } else if (update_done(jA,jB,jT)) {
    main_ang_velo = 0.04;
    if (forward) {
        if (path_index < path1_size-1) { // step through the array
          path_index +=1;
          jA.pot_angle=path2[path_index][0];
          jA.desired_angle=jA.pot_angle;
          
          jB.pot_angle=path2[path_index][1];
          jB.desired_angle = constrain((jA.desired_angle + jB.pot_angle), jB.svo.low_ang, jB.svo.high_ang);  
          
          jT.pot_angle=path2[path_index][2];
          jT.desired_angle=jT.pot_angle;
        } else { // Done with array, restart the array
          //path_index = 0;
          new_segment = true;
          forward=false;
        }
      }  else {  // reverse 
        if (path_index > 0) { // step through the array
          path_index -=1;
          jA.pot_angle=path2[path_index][0];
          jA.desired_angle=jA.pot_angle;
          
          jB.pot_angle=path2[path_index][1];
          jB.desired_angle = constrain((jA.desired_angle + jB.pot_angle), jB.svo.low_ang, jB.svo.high_ang);  
          
          jT.pot_angle=path2[path_index][2];
          jT.desired_angle=jT.pot_angle;
        } else { // Done with array, restart the array
          path_index = 0;
          new_segment = true;
          forward=true;
        }
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
    path1_init=true; 
    path2_init=true; 
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
    path1_init=true; 
    path2_init=true; 
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

  if (jS.pot_value < 200) {
    // Move the arm using the path2 array
    #if SERIALOUT
      Serial.print(",<200-PATH,");
    #endif
    //main_ang_velo = 0.01;
    path2_loop();
  } else if (jS.pot_value < 400) {
    // Move the arm using the path1 array
    #if SERIALOUT
      Serial.print(",<400-PATH,");
    #endif
    main_ang_velo = 0.04;
    path1_loop();
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
  jC.desired_angle = jC.pot_angle -jB.desired_angle - 60.0;

  // Joint D is the Claw... sorry confusing, but C was taken
  pot_map(jD);
  jD.desired_angle = jD.pot_angle;

  // GET SERVO Pulse width VALUES FROM ROBOT ARM OUTPUT ANGLE
  servo_map_with_limits(jA, main_ang_velo);
  servo_map_with_limits(jB, main_ang_velo);  
  servo_map_with_limits(jC, main_ang_velo); 
  servo_map(jD);  // full speed on servo claw
//  servo_map_with_limits(jD, 1.0);  // full speed on servo claw

  // Turntable is geared down 2:1 
  servo_map_with_limits(jT, main_ang_velo); 

  // No S servo attached
  //servo_map_with_limits(jS,main_ang_velo); 

  #if A_ON
    pwm.writeMicroseconds(jA.svo.digital_pin, jA.servo_ms); // Adafruit servo library
    //servoA.writeMicroseconds(jA.servo_ms);
  #endif
  #if B_ON
    pwm.writeMicroseconds(jB.svo.digital_pin, jB.servo_ms); // Adafruit servo library
    //servoB.writeMicroseconds(jB.servo_ms);
  #endif
  #if C_ON
    pwm.writeMicroseconds(jC.svo.digital_pin, jC.servo_ms); // Adafruit servo library
    //servoC.writeMicroseconds(jC.servo_ms);
  #endif
  #if D_ON
    pwm.writeMicroseconds(jD.svo.digital_pin, jD.servo_ms); // Adafruit servo library
//    servoD.writeMicroseconds(jD.servo_ms);
  #endif
  #if T_ON
    pwm.writeMicroseconds(jT.svo.digital_pin, jT.servo_ms); // Adafruit servo library
//    servoT.writeMicroseconds(jT.servo_ms);
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
    //log_data(jD,'D',false);
    log_data(jT,'T',false);
    //log_pot(jT);
    //log_data(jS,'S',false);
    Serial.println(", END");
  #endif
}
