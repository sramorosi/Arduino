/* ROBOT ARM CONTROL SOFTWARE FOR SACC MAKE 2 ROBOT ARM
 *  By, SrAmo, July 2022
 *  
 *  Turning off serial output makes loops time  faster.
 */
//#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotionControl.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERIALOUT false  // Controlls SERIAL output. Set true when debugging. 

#define LEN_AB 195.0     // SACC MK2 AB arm in mm
#define LEN_BC 240.0     // SACC MK2 BC arm in mm
#define LEN_CD 120.0

// Booleans to turn on Servos. [bad code can damage servos. Use to isolate problems]
#define A_ON true
#define B_ON true
#define C_ON true
#define D_ON true
#define T_ON true
#define S_ON false

static struct machine_state sacc_arm; 

static struct joint jA,jB,jC,jD,jT,jS;

#define MMPS 200 // mm per second
#define X_PP 240 // x mm for pick and place
#define Y_MV 140 // y swing in mm
#define FLOORH -10 // z of floor for picking
#define BLOCKH 51 // block height mm of 2 inch block

static int cmd_array[][SIZE_CMD_ARRAY]={{1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,  X_PP,1000,0}, // ready block 1
                           {0,2000,45,0,0,0,0,0}, // pause to pick block 1 - UNIQUE IN SEQUENCE
                           {1,MMPS, X_PP,Y_MV,FLOORH,             X_PP,1000,0}, // down to block 2
                           {0,500,-45,0,0,0,0,0}, // pick block 1
                           {1,MMPS, X_PP,Y_MV,FLOORH+2*BLOCKH,    1000, Y_MV,0}, // up block 1 
                           {1,MMPS, X_PP-10,-Y_MV+10,FLOORH+2*BLOCKH,   1000, -Y_MV,0}, // over block 1 
                           {1,MMPS, X_PP-20,-Y_MV+20,FLOORH,            1000, -Y_MV,0}, // place block 1 
                           {0,500,45,0,0,0,0,0}, // drop block 1
                           {1,MMPS, X_PP,-Y_MV,FLOORH+2*BLOCKH,      1000, -Y_MV,0}, // up clear block 1 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,      X_PP,1000,0}, // ready block 2
                           {1,MMPS, X_PP,Y_MV,FLOORH,             X_PP,1000,0}, // down to block 2
                           {0,500,-45,0,0,0,0,0}, // pick block 2
                           {1,MMPS, X_PP,Y_MV,FLOORH+3*BLOCKH,    1000, Y_MV,0}, // up block 2 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+3*BLOCKH,    1000, -Y_MV,0}, // over block 2 
                           {1,MMPS/2, X_PP,-Y_MV,FLOORH+1*BLOCKH,  1000, -Y_MV,0}, // place block 2 
                           {0,500,45,0,0,0,0,0}, // drop block 2
                           {1,MMPS, X_PP,-Y_MV,FLOORH+3*BLOCKH,    1000, -Y_MV,0}, // up clear block 2 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       X_PP,1000,0},  // ready block 3
                           {1,MMPS, X_PP,Y_MV,FLOORH,              X_PP,1000,0}, // down to block 2
                           {0,500,-45,0,0,0,0,0}, // pick  block 3
                           {1,MMPS, X_PP,Y_MV,FLOORH+4*BLOCKH,    1000, Y_MV,0}, // up block 3 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+4*BLOCKH,    1000, -Y_MV,0}, // over block 3 
                           {1,MMPS/2, X_PP,-Y_MV,FLOORH+2*BLOCKH,  1000, -Y_MV,0}, // place block 3 
                           {0,500,45,0,0,0,0,0}, // drop block 3
                           {1,MMPS, X_PP,-Y_MV,FLOORH+4*BLOCKH,    1000, -Y_MV,0}, // up clear block 3 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       X_PP,1000,0},  // ready block 4
                           {1,MMPS, X_PP,Y_MV,FLOORH,              X_PP,1000,0},  // pick block 4
                           {0,500,-45,0,0,0,0,0}, // pick  block 4
                           {1,MMPS, X_PP,Y_MV,FLOORH+5*BLOCKH,    1000, Y_MV,0}, // up block 4 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+5*BLOCKH,    1000, -Y_MV,0}, // over block 4 
                           {1,MMPS/2, X_PP,-Y_MV,FLOORH+3*BLOCKH,  1000, -Y_MV,0}, // place block 4
                           {0,500,45,0,0,0,0,0}, // drop block 4
                           {1,MMPS, X_PP,-Y_MV,FLOORH+5*BLOCKH,    1000, -Y_MV,0}, // up clear block 4 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       X_PP,1000,0},  // pick block 5
                           {1,MMPS, X_PP,Y_MV,FLOORH,              X_PP,1000,0},  // pick block 5
                           {0,500,-45,0,0,0,0,0}, // pick  block 5
                           {1,MMPS, X_PP,Y_MV,FLOORH+6*BLOCKH,    1000, Y_MV,0}, // up block 5 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+6*BLOCKH,    1000, -Y_MV,0}, // over block 5 
                           {1,MMPS/2, X_PP,-Y_MV,FLOORH+4*BLOCKH,  1000, -Y_MV,0}, // place block 5
                           {0,500,45,0,0,0,0,0}, // drop block 5
                           {1,MMPS, X_PP,-Y_MV,FLOORH+6*BLOCKH,    1000, -Y_MV,0}}; // up clear block 5 

void logData(joint jt,char jt_letter) {
  Serial.print(",");
  Serial.print(jt_letter);
//  Serial.print(", p_value,");
//  Serial.print(jt.pot_value);
  Serial.print(", Pang,");
  Serial.print(jt.pot_angle,1);
//  Serial.print(", PrevAngle,");
  //Serial.print(jt.previous_angle,1);
  Serial.print(",dsr_ang,");
  Serial.print(jt.desired_angle,1);
  Serial.print(", servo_ms,");
  Serial.print(jt.servo_ms);
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
  
  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(1 ,160,  0, 870, 175);
  jB.pot = set_pot(5 ,131,-90, 500,  0);
  jC.pot = set_pot(4 , 139,-50, 910, 50); 
  jD.pot = set_pot(0 ,460, 0, 800, -60); 
  jT.pot = set_pot(2 ,110, -90, 885, 90); 
  jS.pot = set_pot(3 , 0, 0, 1023, 280); 

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  jA.svo = set_servo(0,  0.0, 960, 170.0, 2200);
  jB.svo = set_servo(1, 0.0, 1500, 80.0, 1070); // high to low
  jC.svo = set_servo(3, -50.0, 1060, 50.0, 2400); // high to low
  jD.svo = set_servo(2,  90.0,  1000, 0.0, 1635);
  jT.svo = set_servo(4,-70.0,  400, 70.0, 2500);
  //jS.svo = set_servo(5,  0.0,  400, 180.0, 2500);

  // INITIALIZATION ANGLES FOR ARM
  set_joint(jA, 110.0);  
  set_joint(jB,  10.0);
  set_joint(jC,  -70.0);
  set_joint(jD,   80.0);
  set_joint(jT,    0.0); 
  set_joint(jS,   90.0);
  
  sacc_arm = setup_ms(LEN_BC, 0.0, LEN_AB,cmd_size);
  sacc_arm.state = 2; // this should be overridden by S potentiometer

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
/*
void input_arm_loop() {
    pot_map(jB);
    jB.desired_angle = constrain((jA.desired_angle + jB.pot_angle), jB.svo.low_ang, jB.svo.high_ang); 

  // want Joint C to have a fix angle relative to ground, thus driven by joint B.
  // The C potentiometer is for tuning (adding to) the C position.
  pot_map(jC);
  jC.pot_angle = -jC.pot_angle;  // REVERSED
  jC.desired_angle = jC.pot_angle -jB.desired_angle - 60.0;

     
    // Turntable
    pot_map(jT);
    jT.desired_angle = -jT.pot_angle;  // reverse the angle
  }
} */

void loop() {
  //########### MAIN LOOP ############
  static float *angles;
  static unsigned long mst;
  static int old_state;
  static line lineCD;

  mst = millis();
  sacc_arm.dt = mst - sacc_arm.prior_mst; // delta time
  sacc_arm.prior_mst = mst;

  jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector
  if (jS.pot_value < 600) {
    sacc_arm.state = 1;  // read command list
    if (old_state != 1) {
      sacc_arm.initialize = true;
    }
  } else {
    sacc_arm.state = 2;  // Manually control using input arm
    if (old_state != 2) {
      sacc_arm.initialize = true;
    }
  }
  old_state = sacc_arm.state;
  state_setup(sacc_arm);  // check if state changed at t   

  #if SERIALOUT
    // output for debugging
    // Serial.print(val,digits)
    Serial.print("mst,");
    Serial.print(mst);
    Serial.print(",STATE,");
    Serial.print(sacc_arm.state);
    Serial.print(",N,"); // command line
    Serial.print(sacc_arm.n);
    Serial.print(",CMD,");
    Serial.print(cmd_array[sacc_arm.n][0]);
  #endif

  switch (sacc_arm.state) {
    case 0:  // DO NOTHING STATE
      break;
    case 1: // COMMAND CONTROL
      commands_loop(sacc_arm,cmd_array);
      jD.desired_angle = 0.0;      // to be fixed
      break;
    case 2:  // MANUAL INPUT ARM CONTROL
      jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
      jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
      jC.pot_value = analogRead(jC.pot.analog_pin);  // read joint Claw
      jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
      jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
    
      pot_map(jA);
      pot_map(jB);
      pot_map(jC); // Claw
      sacc_arm.angClaw = jC.desired_angle;
      pot_map(jD); // Wrist
      pot_map(jT); // Turntable
      // get point C and D from input arm angles:
      lineCD = forward_arm_kinematics(jA.desired_angle/RADIAN,jB.desired_angle/RADIAN,jD.desired_angle/RADIAN,jT.desired_angle/RADIAN, LEN_AB, LEN_BC, LEN_CD);
      inputArmLoop(sacc_arm, lineCD, 200); // limits movement given feed rate
      jD.desired_angle = angles[3]*RADIAN;
      break;
  }
  angles = inverse_arm_kinematics(sacc_arm.at_ptC,LEN_AB,LEN_BC,sacc_arm.at_ptD); 
  jA.desired_angle = angles[0]*RADIAN;
  jB.desired_angle = (jA.desired_angle + angles[1]*RADIAN);  // sacc Specific Adjustment
  //if (jB.desired_angle < -35.0) {
  //  jB.desired_angle = -35.0;  // limit the B joint weight from contacting base 
  //} 
  jT.desired_angle = -angles[2]*RADIAN;
  jC.desired_angle = sacc_arm.angClaw;

  switch (sacc_arm.state) {  // TO DO: find a better way
    case 0:  // DO NOTHING STATE
      break;
    case 1: // COMMAND CONTROL
      jD.desired_angle = -jB.desired_angle-85.0;      // to be fixed
      break;
    case 2:  // MANUAL INPUT ARM CONTROL
      jD.desired_angle = -jB.desired_angle+jD.pot_angle;      // to be fixed
      break;
  }

  // GET SERVO Pulse width VALUES FROM ARM OUTPUT ANGLE
  servo_map(jA);
  servo_map(jB);  
  servo_map(jC); 
  servo_map(jD); 
  servo_map(jT); 

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

  #if SERIALOUT
    Serial.print(",C,");
    Serial.print(sacc_arm.at_ptC.x);
    Serial.print(",");
    Serial.print(sacc_arm.at_ptC.y);
    Serial.print(",");
    Serial.print(sacc_arm.at_ptC.z); 
    //logData(jA,'A');
    logData(jB,'B');
    //logData(jC,'C');
    logData(jD,'D');
    //logData(jT,'T');
    //logData(jS,'S');
    Serial.println(", END");
  #endif
}
