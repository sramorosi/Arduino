/* ROBOT ARM CONTROL SOFTWARE FOR SKYSTONE ROBOT ARM
 *  By, SrAmo, June 2022
 *  
 *  Joint A, B and T are mechanically similar to SACC Make 2
 *  Joint C is the Claw on this Skystone Arm
 *  The angle of the hand is held parallel to the floor by a belt.
 *  Joint D is the Wrist rotation in the Top View, dosn't exist on Make 2
 *  
 *  Lesson Learned: Short loop times equals smooth arm performance.
 *  Turning off serial output makes loops time much faster.
 *  Turn off any unnesessary code.
 *  Path method uses a lot of memory.
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
  uint8_t digital_pin; // Adafruit digital pin number
  int low_ms; // low microsecond point, from about 500 to 2400
  float low_ang; // corresponding angle at low microsecond
  int high_ms; // high microsecond point, from about 500 to 2400
  float high_ang; // corresponding angle at high microsecond
}; 

struct joint {
  potentiometer pot; // Sub structure which contains static pot information
  arm_servo svo; // Sub structure which contains the static servo information
  int pot_value;  // current potentiometer value in millivolts 
  float pot_angle; // Potentiometer arm angle, if used
  unsigned long previous_millis; // used to find servo rate of change
  float previous_angle;  // used with rate limiting
  float desired_angle;  // angle from input device or array
  int servo_ms;   // servo command in microseconds
  //float previous_velo;  // previous velocity, used to find acceleration
};
struct joint jA,jB,jC,jD,jT,jS;

unsigned long millisTime;

//#define lenAB 50.0     // Length of Input AB arm in mm
//#define lenBC 60.0     // Length of Input BC arm in mm
//#define RADIAN 57.2957795  // number of degrees in one radian

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
   #endif
 
  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(0 ,134,  0, 895, 178); 
  jB.pot = set_pot(1 ,500,-90, 908,  0); 
  jC.pot = set_pot(3 , 116,-90, 903, 90); 
  jD.pot = set_pot(2 ,250, 60, 747, -60); 
  //jT.pot = set_pot(4 ,160, -70, 510, 0); 
  //jS.pot = set_pot(5 , 0, 0, 1023, 280); 

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  jA.svo = set_servo(0,  0.0, 2250, 90.0, 1480); // high to low
  jB.svo = set_servo(1, 0.0, 1500, 90.0, 850); // high to low
  // C = Claw
  jC.svo = set_servo(2, -80.0, 600, 80.0, 2500); // high to low
  // D = Wrist (top view rotation)
  jD.svo = set_servo(3,  -60.0,  500, 100.0, 2300);
  jT.svo = set_servo(4,-70.0,  400, 70.0, 2500);
  //jS.svo = set_servo(3,  0.0,  400, 180.0, 2500);

  // INITIALIZATION ANGLES FOR ARM
  set_joint(jA, 110.0);  
  set_joint(jB,  10.0);
  set_joint(jC,  -70.0);
  set_joint(jD,   80.0);
  set_joint(jT,    -10.0); 
  //set_joint(jS,   90.0);

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
}

void servo_map_with_limits(joint & jt, float rate) {
// WITH rate limiting
// SERVO RATE LIMITING, for smooth operation and prevent damage
// Limit how much a servo can change in a unit time

  int dt;
  float current_velo;
  
  dt = jt.previous_millis - millis();

  current_velo = (jt.desired_angle-jt.previous_angle)/dt;
  
  if (current_velo > rate) {
    jt.previous_angle += rate*dt;
  } else if (-current_velo > rate) {
    jt.previous_angle -= rate*dt;
  } else {
    jt.previous_angle = jt.desired_angle;
  }

  // Map joint angle to the servo microsecond value
  jt.servo_ms = map(jt.previous_angle, jt.svo.low_ang, jt.svo.high_ang, jt.svo.low_ms, jt.svo.high_ms);
 
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
    Serial.print(", PrevAngle,");
    Serial.print(jt.previous_angle,1);
    Serial.print(", DESAngle,");
    Serial.print(jt.desired_angle,1);
    Serial.print(", servo_ms,");
    Serial.print(jt.servo_ms);
  #endif
}
void input_arm_loop() {
  // Map potentiometer values to Angles and Convert to Robot arm angles
  // main loop
  pot_map(jA);

  pot_map(jB);
  //jB.desired_angle = -constrain((jA.desired_angle + jB.pot_angle), jB.svo.low_ang, jB.svo.high_ang);  
  jB.desired_angle = -(jA.desired_angle + jB.pot_angle);  
  if (jB.desired_angle < -35.0) {
    jB.desired_angle = -35.0;  // limit the B joint weight from contacting base 
  } 
 
  // Turntable
  //pot_map(jT);
  //jT.desired_angle = -jT.pot_angle;  // reverse the angle
  
  // calculate c arm positions from angles
  //cx = lenAB*cos(outputA*1000 / 57296) + lenBC*cos(outputB*1000 / 57296);
  //cy = lenAB*sin(outputA*1000 / 57296) + lenBC*sin(outputB*1000 / 57296);
}

void loop() {
  //########### MAIN LOOP ############

  // reads the pots
  jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
  jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
  jC.pot_value = analogRead(jC.pot.analog_pin);  // read joint Claw
  jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
  //jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
  //jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector
  //jS.pot_value = 800;  // read the selector

  #if SERIALOUT
    // output for debugging
    // Serial.print(val,digits)
    millisTime = millis();
    Serial.print("millis,");
    Serial.print(millisTime);
  #endif

  input_arm_loop();
  
  pot_map(jC);

  pot_map(jD);
  jD.desired_angle = jD.pot_angle;

  // GET SERVO Pulse width VALUES FROM ROBOT ARM OUTPUT ANGLE
  servo_map_with_limits(jA, main_ang_velo);
  servo_map_with_limits(jB, main_ang_velo);  
  servo_map(jC); 
  servo_map(jD);  // full speed on servo claw
  
  // Turntable 
  servo_map_with_limits(jT, main_ang_velo); 

  // No S servo attached
  //servo_map_with_limits(jS,main_ang_velo); 

  #if A_ON
    pwm.writeMicroseconds(jA.svo.digital_pin, jA.servo_ms); // Adafruit servo library
  #endif
  #if B_ON
    pwm.writeMicroseconds(jB.svo.digital_pin, jB.servo_ms); // Adafruit servo library
  #endif
  #if C_ON
    pwm.writeMicroseconds(jC.svo.digital_pin, jC.servo_ms); // Adafruit servo library
  #endif
  #if D_ON
    pwm.writeMicroseconds(jD.svo.digital_pin, jD.servo_ms); // Adafruit servo library
  #endif
  #if T_ON
    pwm.writeMicroseconds(jT.svo.digital_pin, jT.servo_ms); // Adafruit servo library
  #endif

  #if SERIALOUT
    //log_data(jA,'A',false);
    log_data(jB,'B',false);
    //log_data(jC,'C',false);
    //log_data(jD,'D',false);
    //log_data(jT,'T',false);
    //log_pot(jT);
    //log_data(jS,'S',false);
    Serial.println(", END");
  #endif
}
