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

#define SERIALOUT true  // Controlls SERIAL output. Set true when debugging. 

#define LEN_AB 320.0     // Skystone AB arm in mm
#define LEN_BC 320.0     // Skystone BC arm in mm
#define LEN_CD 140.0

// Booleans to turn on Servos. [bad code can damage servos. Use to isolate problems]
#define A_ON true
#define B_ON true
#define C_ON true
#define D_ON true
#define T_ON true
#define S_ON false

struct machine_state skystone_arm; 

struct joint jA,jB,jC,jD,jT,jS;

static point pointC;
static float *angles;

#define MMPS 200 // mm per second
#define X_PP 280 // x mm for pick and place
#define Y_MV 200 // y swing in mm
#define FLOORH -80 // z of floor for picking
#define BLOCKH 100 // block height mm
#define ALPHAC -90 // global C angle, all moves
#define ALPHADPICK 90 // global D for pick
#define ALPHADPLACE 0 // global D for place

static int cmd_array[][SIZE_CMD_ARRAY]={{1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,  ALPHAC,ALPHADPICK}, // ready block 1
                           {2,200,45,0,0,0,0}, // pause to pick block 1 - UNIQUE IN SEQUENCE
                           {1,MMPS, X_PP,Y_MV,FLOORH,             ALPHAC,ALPHADPICK}, // down to block 2
                           {2,500,-45,0,0,0,0}, // pick block 1
                           {1,MMPS, X_PP,Y_MV,FLOORH+2*BLOCKH,    ALPHAC,ALPHADPICK}, // up block 1 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+2*BLOCKH,   ALPHAC,ALPHADPLACE}, // over block 1 
                           {1,MMPS, X_PP,-Y_MV,FLOORH,            ALPHAC,ALPHADPLACE}, // place block 1 
                           {2,500,45,0,0,0,0}, // drop block 1
                           {1,MMPS, X_PP,-Y_MV,FLOORH+2*BLOCKH,   ALPHAC,ALPHADPLACE}, // up clear block 1 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,      ALPHAC,ALPHADPICK}, // ready block 2
                           {1,MMPS, X_PP,Y_MV,FLOORH,             ALPHAC,ALPHADPICK}, // down to block 2
                           {2,500,-45,0,0,0,0}, // pick block 2
                           {1,MMPS, X_PP,Y_MV,FLOORH+3*BLOCKH,    ALPHAC,ALPHADPICK}, // up block 2 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+3*BLOCKH,   ALPHAC,ALPHADPLACE}, // over block 2 
                           {1,MMPS/2, X_PP,-Y_MV,FLOORH+1*BLOCKH, ALPHAC,ALPHADPLACE}, // place block 2 
                           {2,500,45,0,0,0,0}, // drop block 2
                           {1,MMPS, X_PP,-Y_MV,FLOORH+3*BLOCKH,   ALPHAC,ALPHADPLACE}, // up clear block 2 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       ALPHAC,ALPHADPICK},  // ready block 3
                           {1,MMPS, X_PP,Y_MV,FLOORH,              ALPHAC,ALPHADPICK}, // down to block 2
                           {2,500,-45,0,0,0,0}, // pick  block 3
                           {1,MMPS, X_PP,Y_MV,FLOORH+4*BLOCKH,     ALPHAC,ALPHADPICK}, // up block 3 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+4*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 3 
                           {1,MMPS/2, X_PP,-Y_MV,FLOORH+2*BLOCKH,  ALPHAC,ALPHADPLACE}, // place block 3 
                           {2,500,45,0,0,0,0}, // drop block 3
                           {1,MMPS, X_PP,-Y_MV,FLOORH+4*BLOCKH,    ALPHAC,ALPHADPLACE}, // up clear block 3 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       ALPHAC,ALPHADPICK},  // ready block 4
                           {1,MMPS, X_PP,Y_MV,FLOORH,              ALPHAC,ALPHADPICK},  // pick block 4
                           {2,500,-45,0,0,0,0}, // pick  block 4
                           {1,MMPS, X_PP,Y_MV,FLOORH+5*BLOCKH,     ALPHAC,ALPHADPICK}, // up block 4 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+5*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 4 
                           {1,MMPS/2, X_PP,-Y_MV,FLOORH+3*BLOCKH,  ALPHAC,ALPHADPLACE}, // place block 4
                           {2,500,45,0,0,0,0}, // drop block 4
                           {1,MMPS, X_PP,-Y_MV,FLOORH+5*BLOCKH,    ALPHAC,ALPHADPLACE}, // up clear block 4 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       ALPHAC,ALPHADPICK},  // pick block 5
                           {1,MMPS, X_PP,Y_MV,FLOORH,              ALPHAC,ALPHADPICK},  // pick block 5
                           {2,500,-45,0,0,0,0}, // pick  block 5
                           {1,MMPS, X_PP,Y_MV,FLOORH+6*BLOCKH,     ALPHAC,ALPHADPICK}, // up block 5 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+6*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 5 
                           {1,MMPS/2, X_PP,-Y_MV,FLOORH+4*BLOCKH,  ALPHAC,ALPHADPLACE}, // place block 5
                           {2,500,45,0,0,0,0}, // drop block 5
                           {1,MMPS, X_PP,-Y_MV,FLOORH+6*BLOCKH,    ALPHAC,ALPHADPLACE}}; // up clear block 5 
                           
void logData(joint jt,char jt_letter) {
  Serial.print(",");
  Serial.print(jt_letter);
//  Serial.print(", p_value,");
//  Serial.print(jt.pot_value);
//  Serial.print(", Pang,");
//  Serial.print(jt.pot_angle*RADIAN,1);
  Serial.print(",");
  Serial.print(jt.desired_angle*RADIAN,1);
//  Serial.print(", servo_ms,");
//  Serial.print(jt.servo_ms);
}

void common_loop() { // always in loop regardless of state
  static point pointC;
  // PRIOR TO HERE, Input recieved from Input Arm or Command Lines
  //   For Input Arm,  Angles converted to Point C.
  // MACHINE GOVENOR => limits the speed of motion OF POINT C.

  // Convert point C to angles A,B,T
  pointC = anglesToC(jA.pot_angle/RADIAN,jB.pot_angle/RADIAN,jT.pot_angle/RADIAN, LEN_AB, LEN_BC);
  angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC);  // get A,B,T
  jA.desired_angle = angles[0];
  jB.desired_angle = -(jA.desired_angle + angles[1]);  // Skystone Specific Adjustment
  if (jB.desired_angle < -35.0/RADIAN) {
    jB.desired_angle = -35.0/RADIAN;  // limit the B joint weight from contacting base 
  } 
  jT.desired_angle = angles[2];
  jD.desired_angle = angles[3];
  jC.desired_angle = skystone_arm.angClaw;
 
  // GET SERVO Pulse width VALUES FROM ARM OUTPUT ANGLE
  servo_map(jA);
  servo_map(jB);  
  servo_map(jC); 
  servo_map(jD); 
  servo_map(jT); 

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
  return;
}

void setup() {
  static int cmd_size = sizeof(cmd_array)/(SIZE_CMD_ARRAY*2);  // sizeof array.  2 bytes per int
  #if SERIALOUT
    Serial.begin(9600); // baud rate, slower is easier to read
    while (!Serial) { // With Leonardo, if SERIALOUT = true then this is never true.
      ; // wait for serial port to connect. Needed for native USB port only
    } 
    Serial.print("cmd_size,");
    Serial.println(cmd_size);
  #endif

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

  skystone_arm = setup_ms(LEN_BC, 0.0, LEN_AB,cmd_size);
  skystone_arm.state = 1; // this should be overridden by S potentiometer

  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(0 ,134,  0.0/RADIAN, 895, 178.0/RADIAN); 
  jB.pot = set_pot(1 ,500,-90.0/RADIAN, 908,  0.0/RADIAN); 
  jC.pot = set_pot(3 , 116,-90.0/RADIAN, 903, 90.0/RADIAN); 
  jD.pot = set_pot(2 ,250, 60.0/RADIAN, 747, -60.0/RADIAN); 
  jT.pot = set_pot(4 ,160, -90.0/RADIAN, 510, 0.0/RADIAN); 
  jS.pot = set_pot(5 , 0, 0.0/RADIAN, 1023, 280.0/RADIAN);  // to tune

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  jA.svo = set_servo(0,  0.0/RADIAN, 2250, 90.0/RADIAN, 1480); // high to low
  jB.svo = set_servo(1, 0.0/RADIAN, 1500, 90.0/RADIAN, 850); // high to low
  jC.svo = set_servo(2, -80.0/RADIAN, 600, 80.0/RADIAN, 2500); // 45 DEG = FULL OPEN, -45 = FULL CLOSE
  // D = Wrist (top view rotation)
  jD.svo = set_servo(3,  0.0/RADIAN,  1500, 90.0/RADIAN, 2175);
  jT.svo = set_servo(4,-90.0/RADIAN,  890, 0.0/RADIAN, 1475);

  // INITIALIZATION ANGLES FOR ARM
  set_joint(jA, 130.0/RADIAN);  
  set_joint(jB,  0.0/RADIAN);
  set_joint(jC,  -70.0/RADIAN);
  set_joint(jD,   80.0/RADIAN);
  set_joint(jT,    -10.0/RADIAN); 
  
  pointC = anglesToC(jA.desired_angle,jB.desired_angle,jT.desired_angle, LEN_AB, LEN_BC);
  machineGovenor(skystone_arm, pointC, 200, 0.0, 0.0); // limits movement given feed rate
  common_loop();
}

void loop() {  //########### MAIN LOOP ############
  static int old_state;

  jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector
  if (jS.pot_value < 600) {
    skystone_arm.state = 1;  // read command list
    if (old_state != 1) {
      skystone_arm.initialize = true;
    }
  } else {
    skystone_arm.state = 2;  // Manually control using input arm
    if (old_state != 2) {
      skystone_arm.initialize = true;
    }
  }
  old_state = skystone_arm.state;
  state_setup(skystone_arm);  // check if state changed
 
  #if SERIALOUT
    // output for debugging
    // Serial.print(val,digits)
    Serial.print("mst,");
    Serial.print(skystone_arm.prior_mst);
    Serial.print(",STATE,");
    Serial.print(skystone_arm.state);
    Serial.print(",N,"); // command line
    Serial.print(skystone_arm.n);
    Serial.print(",CMD,");
    Serial.print(cmd_array[skystone_arm.n][0]);
  #endif

  switch (skystone_arm.state) {
    case 0:  // DO NOTHING STATE
      break;
    case 1: // COMMAND CONTROL
      commands_loop(skystone_arm,cmd_array);
      // GET OUTPUT ANGLES FROM INPUTS
      break;
    case 2:  // MANUAL INPUT ARM CONTROL
      jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
      jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
      jC.pot_value = analogRead(jC.pot.analog_pin);  // read joint Claw
      jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
      jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
    
      pot_map(jA);
      pot_map(jB);
      pot_map(jC); 
      skystone_arm.angClaw = jC.desired_angle;
      pot_map(jD); // Wrist
      pot_map(jT); // Turntable
      // get point C and D from input arm angles:
      pointC = anglesToC(jA.desired_angle,jB.desired_angle,jT.desired_angle, LEN_AB, LEN_BC);
      machineGovenor(skystone_arm, pointC, 200, 0.0, 0.0); // limits movement given feed rate
      break;
  }

  common_loop();
  #if SERIALOUT
    Serial.print(",G,");
    Serial.print(skystone_arm.at_ptG.x);
    Serial.print(",");
    Serial.print(skystone_arm.at_ptG.y);
    Serial.print(",");
    Serial.print(skystone_arm.at_ptG.z);
    logData(jA,'A');
    logData(jB,'B');
    logData(jC,'C');
    logData(jD,'D');
    logData(jT,'T');
    //logData(jS,'S');
    Serial.println(", END");
  #endif
}
