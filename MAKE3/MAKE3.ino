/* ROBOT ARM CONTROL SOFTWARE FOR MAKE3 ROBOT ARM
 *  By, SrAmo, August 2022
 *  
 *  
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotionControl.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERIALOUT false  // Controlls SERIAL output. Set true when debugging. 

void logData(joint jt,char jt_letter) {
  Serial.print(",");
  Serial.print(jt_letter);
//  Serial.print(", p_value,");
//  Serial.print(jt.pot_value);
//  Serial.print(", Pang,");
//  Serial.print(jt.pot_angle*RADIAN,1);
  Serial.print(",");
//  Serial.print(",dsr_ang,");
  Serial.print(jt.desired_angle*RADIAN,1);
  Serial.print(", servo_ms,");
  Serial.print(jt.servo_ms);
} 

#define LEN_AB 350.0     // Length of Input AB arm in mm
#define LEN_BC 380.0     // Length of Input BC arm in mm
#define S_CG_X 180.0 
#define S_CG_Y 40.0

struct machine_state make3; 
struct joint jA,jB,jC,jD,jCLAW,jT,jS;

#define MMPS 400 // mm per second
#define X_PP 250 // x mm for pick and place
#define Y_MV 200 // y swing in mm
#define Y_MV_NEG -150 // y swing in mm
#define FLOORH -80 // z of floor for picking
#define BLOCKH 100 // block height mm
#define BLOCKW 100 // block width mm
#define ALPHAC -90 // global C angle, all moves
#define ALPHADPICK -70 // global D for pick
#define ALPHADPLACE -40 // global D for place
#define CLAWCLOSE -30
#define CLAWOPEN 50

static int lineCmds[][SIZE_CMD_ARRAY]={{2,100,CLAWOPEN,0,0,0,0},
                           {1,100, 100,400,300,  -90,90}, // ready
                           {2,2000,CLAWOPEN,0,0,0,0}, // pause to pick 
                           {2,1000,CLAWCLOSE,0,0,0,0}, // close to pick camera
                           {1,80, 100,-400,300,   -90,-90}, // line over
                           {0,1000,0,0,0,0,0},    // pause
                           {1,80, 100,400,300,   -90,90}}; // line back
                           
static int line2Cmds[][SIZE_CMD_ARRAY]={{2,100,CLAWOPEN,0,0,0,0},
                           {1,100, 200,400,0,  90,0}, // ready
                           {2,2000,CLAWOPEN,0,0,0,0}, // pause to pick 
                           {2,1000,CLAWCLOSE,0,0,0,0}, // close to pick camera
                           {1,50, 200,-400,0,   -90,0}, // line over
                           {0,1000,0,0,0,0,0},    // pause
                           {1,50, 200,400,0,   90,0}}; // line back

static int cmd_array[][SIZE_CMD_ARRAY]={{2,100,CLAWOPEN,0,0,0,0},
                           {1,100, X_PP,Y_MV,FLOORH+BLOCKH,  ALPHAC,ALPHADPICK}, // ready block 1
                           {2,1000,CLAWOPEN,0,0,0,0}, // pause to pick block 1 - UNIQUE IN SEQUENCE
                           {1,MMPS, X_PP,Y_MV,FLOORH,             ALPHAC,ALPHADPICK}, // down to block 2
                           {2,500,CLAWCLOSE,0,0,0,0}, // pick block 1
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+1*BLOCKH,   ALPHAC,ALPHADPLACE}, // over block 1 
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH,            ALPHAC,ALPHADPLACE}, // place block 1 
                           {2,500,CLAWOPEN,0,0,0,0}, // drop block 1
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+1*BLOCKH,   ALPHAC,ALPHADPLACE}, // up clear block 1 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,      ALPHAC,ALPHADPICK}, // ready block 2
                           {1,MMPS, X_PP,Y_MV,FLOORH,             ALPHAC,ALPHADPICK}, // down to block 2
                           {2,500,CLAWCLOSE,0,0,0,0}, // pick block 2
                           {1,MMPS, X_PP,Y_MV_NEG+BLOCKW,FLOORH+2*BLOCKH,   ALPHAC,ALPHADPLACE}, // over block 2 
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+2*BLOCKH,   ALPHAC,ALPHADPLACE}, // over block 2 
                           {1,MMPS/2, X_PP,Y_MV_NEG,FLOORH+1*BLOCKH, ALPHAC,ALPHADPLACE}, // place block 2 
                           {2,500,CLAWOPEN,0,0,0,0}, // drop block 2
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+2*BLOCKH,   ALPHAC,ALPHADPLACE}, // over block 2 
                           {1,MMPS, X_PP,Y_MV_NEG+BLOCKW,FLOORH+2*BLOCKH,   ALPHAC,ALPHADPLACE}, // up clear block 2 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       ALPHAC,ALPHADPICK},  // ready block 3
                           {1,MMPS, X_PP,Y_MV,FLOORH,              ALPHAC,ALPHADPICK}, // down to block 2
                           {2,500,CLAWCLOSE,0,0,0,0}, // pick  block 3
                           {1,MMPS, X_PP,Y_MV_NEG+BLOCKW,FLOORH+3*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 3 
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+3*BLOCKH,    ALPHAC,ALPHADPLACE}, // up clear block 3 
                           {1,MMPS/2, X_PP,Y_MV_NEG,FLOORH+2*BLOCKH,  ALPHAC,ALPHADPLACE}, // place block 3 
                           {2,500,CLAWOPEN,0,0,0,0}, // drop block 3
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+3*BLOCKH,    ALPHAC,ALPHADPLACE}, // up clear block 3 
                           {1,MMPS, X_PP,Y_MV_NEG+BLOCKW,FLOORH+3*BLOCKH,    ALPHAC,ALPHADPLACE}, // up clear block 3 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       ALPHAC,ALPHADPICK},  // ready block 4
                           {1,MMPS, X_PP,Y_MV,FLOORH,              ALPHAC,ALPHADPICK},  // pick block 4
                           {2,500,CLAWCLOSE,0,0,0,0}, // pick  block 4
                           {1,MMPS, X_PP,Y_MV_NEG+BLOCKW,FLOORH+4*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 4 
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+4*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 4 
                           {1,MMPS/2, X_PP,Y_MV_NEG,FLOORH+3*BLOCKH,  ALPHAC,ALPHADPLACE}, // place block 4
                           {2,500,CLAWOPEN,0,0,0,0}, // drop block 4
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+4*BLOCKH,    ALPHAC,ALPHADPLACE}, // up clear block 4 
                           {1,MMPS, X_PP,Y_MV_NEG+BLOCKW,FLOORH+4*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 4 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       ALPHAC,ALPHADPICK},  // pick block 5
                           {1,MMPS, X_PP,Y_MV,FLOORH,             ALPHAC,ALPHADPICK},  // pick block 5
                           {2,500,CLAWCLOSE,0,0,0,0}, // pick  block 5
                           {1,MMPS, X_PP,Y_MV_NEG+BLOCKW,FLOORH+5*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 5 
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+5*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 5 
                           {1,MMPS/2, X_PP,Y_MV_NEG,FLOORH+4*BLOCKH,  ALPHAC,ALPHADPLACE}, // place block 5
                           {2,500,CLAWOPEN,0,0,0,0}, // drop block 5
                           {1,MMPS, X_PP,Y_MV_NEG,FLOORH+5*BLOCKH,    ALPHAC,ALPHADPLACE}, // up clear block 5 
                           {1,MMPS, X_PP,Y_MV_NEG+BLOCKW,FLOORH+5*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 5 
                           
                           {1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,       ALPHAC,ALPHADPICK},  // pick block 6
                           {1,MMPS, X_PP,Y_MV,FLOORH,              ALPHAC,ALPHADPICK},  // pick block 6
                           {2,500,CLAWCLOSE,0,0,0,0}, // pick  block 6
                           {1,MMPS/2, X_PP,Y_MV_NEG+BLOCKW,FLOORH+6*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 6 
                           {1,MMPS/2, X_PP,Y_MV_NEG,FLOORH+6*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 6 
                           {1,MMPS/2, X_PP,Y_MV_NEG,FLOORH+5*BLOCKH,  ALPHAC,ALPHADPLACE}, // place block 6
                           {2,500,CLAWOPEN,0,0,0,0}, // drop block 6
                           {1,MMPS, X_PP-100,Y_MV_NEG,FLOORH+6*BLOCKH,    ALPHAC,ALPHADPLACE}, // up clear block 6 
                           {1,MMPS/2, X_PP,Y_MV_NEG+2*BLOCKW,FLOORH+6*BLOCKH,    ALPHAC,ALPHADPLACE}, // over block 6 

                           {1,MMPS/2, X_PP,Y_MV,FLOORH+BLOCKH,       ALPHAC,ALPHADPICK}};  // pick block ready

void setup() {  // put your setup code here, to run once:
  static int cmd_size = sizeof(cmd_array)/(SIZE_CMD_ARRAY*2);  // sizeof array.  2 bytes per int
  make3 = setup_ms(LEN_BC, 0.0, LEN_AB,cmd_size);
  make3.state = 1; 

  #if SERIALOUT  
    Serial.begin(9600); // baud rate
    while (!Serial) {
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
  
  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(0 ,134,  0/RADIAN, 895, 178/RADIAN); // good
  jB.pot = set_pot(1 ,500,-90/RADIAN, 908,  0/RADIAN); // good
  //jC.pot = set_pot(3 , 116,-90/RADIAN, 903, 90/RADIAN); // C will not have a pot on Make3
  jD.pot = set_pot(2 ,250, 60/RADIAN, 747, -60/RADIAN); 
  jT.pot = set_pot(4 ,166, -90/RADIAN, 960, 90/RADIAN); // better
  jCLAW.pot = set_pot(3 , 250,-50/RADIAN, 750, 50/RADIAN);  // input arm is limited to 250 to 750
  jS.pot = set_pot(5 , 0, 0/RADIAN, 1023, 280/RADIAN);  // to tune

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  jA.svo = set_servo(0,  2.2/RADIAN, 478, 176.8/RADIAN, 2390); // good
  jB.svo = set_servo(1, 0.0/RADIAN, 500,-175.0/RADIAN, 2320); // good
  jC.svo = set_servo(2, -153.5/RADIAN, 475, 5.0/RADIAN, 2156); // good.  TO DO REPLACE WITH 270 DEG SERVO
  jD.svo = set_servo(3,  -90.0/RADIAN,  811, 90.0/RADIAN, 2054); // good
  jCLAW.svo = set_servo(4, -50.0/RADIAN,  900, 50.0/RADIAN, 1900); // 45 DEG = FULL OPEN, -45 = FULL CLOSE
  jT.svo = set_servo(5,  0.0/RADIAN,  1650, 90.0/RADIAN, 450); // good, SERVO RANGE: -57 TO +92 DEG

  // INITIALIZATION ANGLES FOR ARM
  set_joint(jA, 130.0/RADIAN);  
  set_joint(jB,  -130.0/RADIAN);
  set_joint(jC,  -70.0/RADIAN);
  set_joint(jD,   0.0/RADIAN);
  set_joint(jT,    -10.0/RADIAN); 
  set_joint(jCLAW,  45.0/RADIAN); 
  make3.angClaw = 45.0/RADIAN;
  
  //ADD STUFF TO COMPUT POINTS
}

void loop() {  //########### MAIN LOOP ############
  // put your main code here, to run repeatedly:
  static int old_state = 1;
  static float *angles;
  static float alphaB;
  static point pointC;
  static line lineCG;
  static boolean govenorDone;
  static float testb;
  
  jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector

  make3.state = jS.pot_value/100; // convert to an integer from 0 to 9
  if (make3.state != old_state)
      make3.initialize=true;
  old_state = make3.state;
  state_setup(make3);  // check if state changed

  #if SERIALOUT
    Serial.print("ms,");
    Serial.print(make3.prior_mst);
    Serial.print(", STATE,");
    Serial.print(make3.state);
    Serial.print(", N,"); // command line
    Serial.print(make3.n);
    Serial.print(", CMD,");
    Serial.print(cmd_array[make3.n][0]); 
  #endif
  
   switch (make3.state) {
    case 1:  // DO NOTHING STATES
    case 3:
    case 5:
    case 7:
      break;
    case 0: // COMMAND CONTROL  LINE
      commands_loop(make3,line2Cmds);
      
      pointC =  clawToC(make3.at_ptG, make3.alphaC, make3.alphaD, S_CG_X, S_CG_Y);

      angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC); 
      jA.desired_angle = angles[0]; // global A = local A
      jB.desired_angle = angles[1];  // local B
      alphaB = angles[0]+angles[1];  // global B
      jT.desired_angle = angles[2];  // global T
      jC.desired_angle = make3.alphaC-alphaB; // convert to a local C
      jD.desired_angle = make3.alphaD -  jT.desired_angle; // convert to a local D
      jCLAW.desired_angle = make3.angClaw;
      
      break;
    case 2: // COMMAND CONTROL  LINE
      commands_loop(make3,lineCmds);
      
      pointC =  clawToC(make3.at_ptG, make3.alphaC, make3.alphaD, S_CG_X, S_CG_Y);

      angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC); 
      jA.desired_angle = angles[0]; // global A = local A
      jB.desired_angle = angles[1];  // local B
      alphaB = angles[0]+angles[1];  // global B
      jT.desired_angle = angles[2];  // global T
      jC.desired_angle = make3.alphaC-alphaB; // convert to a local C
//      jD.desired_angle = make3.alphaD -  jT.desired_angle; // convert to a local D
      jD.desired_angle = make3.alphaD; // convert to a local D
      jCLAW.desired_angle = make3.angClaw;
      
      break;
    case 4: // COMMAND CONTROL  C = -90  SAME AS CASE 2 PRESENTLY
      commands_loop(make3,cmd_array);

      pointC =  clawToC(make3.at_ptG, make3.alphaC, make3.alphaD, S_CG_X, S_CG_Y);

      angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC); 
      jA.desired_angle = angles[0]; // global A = local A
      jB.desired_angle = angles[1];  // local B
      alphaB = angles[0]+angles[1];  // global B
      jT.desired_angle = angles[2];  // global T
      jC.desired_angle = make3.alphaC-alphaB; // convert to a local C
//      jD.desired_angle = make3.alphaD -  jT.desired_angle; // convert to a local D
      jD.desired_angle = make3.alphaD; // convert to a local D
      jCLAW.desired_angle = make3.angClaw;
      
      break;
    case 6:  // MANUAL CONTROL  DIRECT FULL SPEED   C = -90
      jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
      jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
      jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
      jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
      jCLAW.pot_value = analogRead(jCLAW.pot.analog_pin);  // read joint Claw
    
      // Direct (full speed) method
      pot_map(jA);
      pot_map(jB);
      pot_map(jCLAW); 
      jC.desired_angle = -jA.desired_angle - jB.desired_angle-90.0/RADIAN;
      make3.angClaw = jC.desired_angle;
      pot_map(jD); // Wrist  TO DO  subtract turntable?
      pot_map(jT); // Turntable
      
      break;
    case 8:  // MANUAL CONTROL  DIRECT FULL SPEED   C = 0
      jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
      jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
      jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
      jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
      jCLAW.pot_value = analogRead(jCLAW.pot.analog_pin);  // read joint Claw
    
      // Direct (full speed) method
      pot_map(jA);
      pot_map(jB);
      pot_map(jCLAW); 
      jC.desired_angle = -jA.desired_angle - jB.desired_angle;   // C is zero
      make3.angClaw = jC.desired_angle;
      pot_map(jD); // Wrist  TO DO  subtract turntable?
      pot_map(jT); // Turntable
      
      break;
    case 9:  // MANUAL CONTROL  WITH GOVENOR    C = -90
        jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
        jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
        jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
        jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
        jCLAW.pot_value = analogRead(jCLAW.pot.analog_pin);  // read joint Claw
      
        // Direct (full speed) method
        pot_map(jA);
        pot_map(jB);
        pot_map(jCLAW); 
        jC.desired_angle = -jA.desired_angle - jB.desired_angle-90.0/RADIAN;
        make3.angClaw = jC.desired_angle;
        pot_map(jD); // Wrist  TO DO  subtract turntable?
        pot_map(jT); // Turntable
        
        lineCG = anglesToG(jA.desired_angle,jB.desired_angle,jT.desired_angle,jC.desired_angle,jD.desired_angle, LEN_AB, LEN_BC,S_CG_X,S_CG_Y);
        govenorDone = machineGovenor(make3, lineCG.p2, 200, jC.desired_angle, jD.desired_angle); 
  
        pointC = clawToC(make3.at_ptG, make3.alphaC, make3.alphaD,S_CG_X,S_CG_Y);
        angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC); // find partial angles  TO DO  need goverened at point c
        jA.desired_angle = angles[0]; // global A = local A
        jB.desired_angle = angles[1];  // local B
        //alphaB = angles[0]+angles[1];  // global B
        jT.desired_angle = angles[2];  // global T
        jC.desired_angle = make3.alphaC; //-alphaB; // convert to a local C
        jD.desired_angle = make3.alphaD; // goverened d
        //jD.desired_angle = make3.alphaD -  jT.desired_angle; // convert to a local D  TO DO  APPLY COMPENSATION ANGLES
        //jCLAW.desired_angle = make3.angClaw;
  
      break;
  }
  // GET SERVO Pulse width VALUES FROM ARM OUTPUT ANGLE
  servo_map(jA);
  servo_map(jB);  
  servo_map(jC); 
  servo_map(jD); 
  servo_map(jCLAW); 
  servo_map(jT); 

  pwm.writeMicroseconds(jA.svo.digital_pin, jA.servo_ms); // Adafruit servo library
  pwm.writeMicroseconds(jB.svo.digital_pin, jB.servo_ms); // Adafruit servo library
  pwm.writeMicroseconds(jC.svo.digital_pin, jC.servo_ms); // Adafruit servo library
  pwm.writeMicroseconds(jD.svo.digital_pin, jD.servo_ms); // Adafruit servo library
  pwm.writeMicroseconds(jCLAW.svo.digital_pin, jCLAW.servo_ms); // Adafruit servo library
  pwm.writeMicroseconds(jT.svo.digital_pin, jT.servo_ms); // Adafruit servo library

  #if SERIALOUT
    Serial.print(",G,");
    Serial.print(make3.at_ptG.x);
    Serial.print(",");
    Serial.print(make3.at_ptG.y);
    Serial.print(",");
    Serial.print(make3.at_ptG.z);
  
//    logData(jA,'A');
//    logData(jB,'B');
//    logData(jC,'C');
//    logData(jD,'D');
    logData(jT,'T');
//    logData(jCLAW,'X');
//      logData(jS,'S');
    Serial.println(", END");
  #endif
}
//
