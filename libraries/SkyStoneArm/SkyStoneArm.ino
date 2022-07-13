/* ROBOT ARM CONTROL SOFTWARE FOR SKYSTONE ROBOT ARM
 *  By, SrAmo, July 2022
 *  
 *  Joint A, B and T are mechanically similar to SACC Make 2
 *  Joint C is the Claw on this Skystone Arm
 *  The angle of the hand is held parallel to the floor by a belt.
 *  Joint D is the Wrist rotation in the Top View, dosn't exist on Make 2
 *  
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotionControl.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERIALOUT true  // Controlls SERIAL output. Turn on when debugging. 
// SERIAL OUTPUT AFFECTS SMOOTHNESS OF SERVO PERFORMANCE 
//  WITH SERIAL true AND LOW 9600 BAUD RATE = JERKY PERFORMANCE
//  WITH false OR HIGH 500000 BAUD RATE = SMOOTH PERFORMANCE

#define LEN_AB 320.0     // Length of Input AB arm in mm
#define LEN_BC 320.0     // Length of Input BC arm in mm

float main_ang_velo = 0.08; // Angular Velocity Limit, DEGREES PER MILLISECOND
// Servos Max Velocity is about 60 deg in 0.12 sec or 460 deg/sec, or 0.46 degrees per millisecond
// This assumes no load and full (7 V) voltage.

// Booleans to turn on Servos. [bad code can damage servos. This can help isolate]
#define A_ON false
#define B_ON false
#define C_ON false
#define D_ON false
#define T_ON false
#define S_ON false

struct machine_state skystone_arm; 

struct joint jA,jB,jC,jD,jT,jS;

static int cmd_array[][SIZE_CMD_ARRAY]={{1,100,LEN_BC-20,0,LEN_AB,0,0,0},
                           {0,3000,0,0,0,0,0,0},
                           {1,100,LEN_BC-40,-200,LEN_AB,0,0,0},
                           {1,50,LEN_BC,LEN_BC,LEN_AB,0,0,0}};

void setup() {
  static int cmd_size = sizeof(cmd_array)/(SIZE_CMD_ARRAY*2);  // sizeof array.  2 bytes per int
  #if SERIALOUT
    Serial.begin(9600); // baud rate, slower is easier to read
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    } 
    Serial.print("cmd_size,");
    Serial.println(cmd_size);
  #endif

  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(0 ,134,  0, 895, 178); 
  jB.pot = set_pot(1 ,500,-90, 908,  0); 
  jC.pot = set_pot(3 , 116,-90, 903, 90); 
  jD.pot = set_pot(2 ,250, 60, 747, -60); 
  jT.pot = set_pot(4 ,160, -90, 510, 0); 
  jS.pot = set_pot(5 , 0, 0, 1023, 280);  // to tune

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  jA.svo = set_servo(0,  0.0, 2250, 90.0, 1480); // high to low
  jB.svo = set_servo(1, 0.0, 1500, 90.0, 850); // high to low
  // C = Claw
  jC.svo = set_servo(2, -80.0, 600, 80.0, 2500); // high to low
  // D = Wrist (top view rotation)
  jD.svo = set_servo(3,  -60.0,  500, 100.0, 2300);
  jT.svo = set_servo(4,-90.0,  670, 90.0, 1930);
  //jS.svo = set_servo(3,  0.0,  400, 180.0, 2500);

  // INITIALIZATION ANGLES FOR ARM
  set_joint(jA, 110.0);  
  set_joint(jB,  10.0);
  set_joint(jC,  -70.0);
  set_joint(jD,   80.0);
  set_joint(jT,    -10.0); 
  //set_joint(jS,   90.0);
  
  skystone_arm = setup_ms(LEN_BC, 0.0, LEN_AB,cmd_size);
  skystone_arm.state = 1; // this should be overridden by S potentiometer

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

  delay(10); // not sure why, but adafruit did it.
}

void input_arm_loop() {
  // reads the pots
  jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
  jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
  jC.pot_value = analogRead(jC.pot.analog_pin);  // read joint Claw
  jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
  jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable

  pot_map(jA);
  pot_map(jB);
  jB.desired_angle = -(jA.desired_angle + jB.pot_angle);  
  if (jB.desired_angle < -35.0) {
    jB.desired_angle = -35.0;  // limit the B joint weight from contacting base 
  } 
  pot_map(jC);  // Claw
  pot_map(jD); // Wrist
  jD.desired_angle = jD.pot_angle;
  pot_map(jT); // Turntable
  jT.desired_angle = jT.pot_angle-20.0;  // adjust zero
}

void loop() {
  //########### MAIN LOOP ############
  static float *angles;
  static unsigned long mst;

  mst = millis();
  skystone_arm.dt = mst - skystone_arm.prior_mst; // delta time
  skystone_arm.prior_mst = mst;

  jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector

  if (jS.pot_value < 600) {
    skystone_arm.state = 1;  // read command list
  } else {
    skystone_arm.state = 2;  // Manually control using input arm
  }
 
  #if SERIALOUT
    // output for debugging
    // Serial.print(val,digits)
    Serial.print("mst,");
    Serial.print(mst);
    Serial.print(",S,");
    Serial.print(jS.pot_value);
    Serial.print(", STATE,");
    Serial.print(skystone_arm.state);
    Serial.print(", N,"); // command line
    Serial.print(skystone_arm.n);
    Serial.print(", CMD,");
    Serial.print(cmd_array[skystone_arm.n][0]);
  #endif

  switch (skystone_arm.state) {
    case 0:
      state_setup(skystone_arm);
      break;
    case 1:
      state_setup(skystone_arm);
      main_ang_velo = 1.0; // this is fast.  feed rate should set speed
      commands_loop(skystone_arm,cmd_array);
      // GET OUTPUT ANGLES FROM INPUTS
      angles = inverse_arm_kinematics(skystone_arm.at_ptC,LEN_AB,LEN_BC,skystone_arm.at_ptF); 
      jA.desired_angle = angles[0];
      jB.desired_angle = angles[1];
      jT.desired_angle = angles[2];
      jD.desired_angle = angles[3];
      break;
    case 2:
      state_setup(skystone_arm);
      main_ang_velo = 0.2;
      input_arm_loop();
      break;
  }
 
  // GET SERVO Pulse width VALUES FROM ARM OUTPUT ANGLE
  servo_map_with_limits(jA, main_ang_velo);
  servo_map_with_limits(jB, main_ang_velo);  
  servo_map(jC); 
  servo_map(jD);  // full speed on servo wrist...
  servo_map_with_limits(jT, main_ang_velo/1.20); 

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
    Serial.print(", Cx,");
    Serial.print(skystone_arm.at_ptC[0]);
    Serial.print(" ,Cy,");
    Serial.print(skystone_arm.at_ptC[1]);
    Serial.print(" ,Cz,");
    Serial.print(skystone_arm.at_ptC[2]);
    //log_data(jA,'A',false);
    //log_data(jB,'B',false);
    //log_data(jC,'C',false);
    //log_data(jD,'D',false);
    //log_data(jT,'T',false);
    //log_pot(jT);
    //log_data(jS,'S',false);
    Serial.println(", END");
  #endif
}
