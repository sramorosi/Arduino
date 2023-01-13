/* ROBOT ARM CONTROL SOFTWARE FOR MAKE3 ROBOT ARM
 *  By, SrAmo, January 2023
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERIALOUT true  // Controlls SERIAL output. Set true when debugging. 

#define RADIAN 57.2957795  // number of degrees in one radian

#define SIZE_CMD_ARRAY 7  // NUMBER OF VALUES PER COMMAND
/*
 *  COMMAND CODES: SEE ARM SHEET
 *  Feed rate is in mm per second.
 */
#define C_TIME 0   // command code for timer
#define C_LINE 1   // command code for line move
#define C_CLAW 2   // command code for claw move

 /*
 *  STRUCTURES FOR MATH, SERVOS, POTENTIOMETERS, JOINTS, and ARM
 */
struct point {float x,y,z;}; 

struct line {struct point p1 , p2; };

struct potentiometer {
  int analog_pin; // Arduino analog pin number (0 - 5)
  int low_mv; // low voltage value, from 0 (0 Volts) to 1023 (5 Volts)
  float low_ang; // corresponding angle at low voltage, in radians
  int high_mv; // high voltage value, from 0 (0 Volts) to 1023 (5 Volts)
  float high_ang; // corresponding angle at high voltage, in radians
  // Note: low and high values do not need to be  at the extreems
  //  Tip:  Pick low and high angles that are easy to read/set/measure
  //      Values outside of the low and high will be extrapolated
};

struct arm_servo {
  // Values are pulse width in microseconds (~400 to ~2400--SERVOS SHOULD BE TESTED)
  // 270 deg servo, gets 290 deg motion with 400 to 2500 microsecond range
  // 180 deg servo, gets 160 deg motion with 500 to 2800 microsecond range
  // 180 deg SAVOX, gets 170 deg motion with 500 to 2800 microsecond range
  uint8_t digital_pin; // Adafruit digital pin number
  int low_ms; // low microsecond value, from about 500 to 2400
  float low_ang; // corresponding angle at low microsecond, in radians
  int high_ms; // high microsecond value, from about 500 to 2400
  float high_ang; // corresponding angle at high microsecond, in radians
}; 

struct joint {
  potentiometer pot; // Sub structure which contains static pot information
  arm_servo svo; // Sub structure which contains the static servo information
  int pot_value;  // current potentiometer value in millivolts 
  float pot_angle; // Potentiometer arm angle, if used, in radians
  float current_angle; // last issued command for angl, in radians
  float target_angle;  // where the angle should end up, in radians
  float target_velocity;  // radians per second
};

struct commands {  
  int num_cmds;
  int cmds[][SIZE_CMD_ARRAY];
};

struct arm {  // as in Robot Arm
  int state; // The different ways that the arm can be controlled. Set by S potentiometer
  boolean initialize; // =true if one needs to initialize state
  //int cmd_size; // size of the command array
  int n; // current active index in the command array
  unsigned long prior_mst; // clock time (microseconds) prior loop 
  unsigned long dt; // delta loop time in ms
  unsigned long timerStart; // timer, for delays
  float moveDist; // distance to travel in current move
  point current_pt; // current (C or G) point 
  joint jA;
  joint jB;
  joint jC;
  joint jD;
  joint jCLAW;
  joint jT;
};

// SETTERS,  INITIALIZERS
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
  // Sets initial joint values
  // Note: can't read physical servo angle, so can jump on startup
  //jt.pot_value = 500;  // middle ish
  jt.current_angle = initial_angle;
  jt.target_angle = initial_angle;
  jt.target_velocity = 2.0;  // default initial velocity, radians per second
}

arm set_arm(point pt) { // starting point for C or G on arm
  // initialize arm structure
  struct arm ms;
  ms.state = 0;
  ms.initialize = true;
  ms.n = 0;  // first command
  //ms.cmd_size = 0;
  ms.prior_mst = millis();
  ms.current_pt = pt;
  return ms;
}

float set_C_Abs(arm the_arm, float fixed_angle) { // returns joint angle C, using A and B
  return -the_arm.jA.current_angle - the_arm.jB.current_angle + fixed_angle;
}

//  ###############  MATH FUNCTIONS ###########################
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

point rot_pt_x(point p1, float a) { // Rotate p1 about X axis by a radians
  static point p2;
  p2.x = p1.x;
  p2.y = p1.y*cos(a)-p1.z*sin(a);
  p2.z = p1.y*sin(a)+p1.z*cos(a);
  return p2;
}

point rot_pt_y(point p1, float a) { // Rotate p1 about Y axis by a radians
  static point p2;
  p2.x = p1.x*cos(a)-p1.z*sin(a);
  p2.y = p1.y;
  p2.z = p1.x*sin(a)+p1.z*cos(a);
  return p2;
}

point rot_pt_z(point p1, float a) { // Rotate p1 about Z axis by a radians
  point p2;
  p2.x = p1.x*cos(a)-p1.y*sin(a);
  p2.y = p1.x*sin(a)+p1.y*cos(a);
  p2.z = p1.z;
  return p2;
}

point add_pts(point p1, point p2) {
  static point p3;
  p3.x = p1.x+p2.x;   p3.y = p1.y+p2.y;   p3.z = p1.z+p2.z;
  return p3;
}
float ptpt_dist(point p1, point p2) {
  static float dist;
  dist = sqrt(pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0)+pow(p1.z-p2.z,2.0) ); 
  return dist;
}

point pt_on_line(float s, float dist, point p1, point p2) { // point along line s from p1
  static point new_pt;
  static float ratio;
  ratio = s / dist; 
  new_pt.x = ratio*(p2.x-p1.x) + p1.x;
  new_pt.y = ratio*(p2.y-p1.y) + p1.y;
  new_pt.z = ratio*(p2.z-p1.z) + p1.z;
  return new_pt;
}

float interpolate(float s, float dist, float p1, float p2) { // dist along s from p1
  static float new_pt;
  static float ratio;
  ratio = s / dist; 
  new_pt = ratio*(p2-p1) + p1;
  return new_pt;
}

float vec_mag(point p) {
  return sqrt(pow((p.x),2)+pow(p.y,2)+pow(p.z,2));
}

float dot_prod (point p1, point p2) {
  return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z;
}

point cross_prod (point p1, point p2) {
  static point cross_p;
  cross_p.x = p1.y*p2.z - p1.z*p2.y;
  cross_p.y = p1.z*p2.x - p1.x*p2.z;
  cross_p.z = p1.x*p2.y - p1.y*p2.x;
  return cross_p;
}

float * inverse_arm_kinematics(point c, float l_ab, float l_bc, float aOffset) {
  // Given arm system GroundT-TA-AB-BC-CD, where T (turntable) and A are [0,0,0]
  // The location of joint C (c[])  AND CX IS POSITIVE ONLY - TO DO SOLVE NEGATIVES
  // The lengths Length AB (l_ab) and Length BC (l_bc) are specified
  //   aOffset is the distance in X from the Turtable axis to the A axis
  // The joints A,B are on a turntable with rotation T parallel to Z through A
  // With T_angle = 0, then joints A & B are parallel to the Y axis
  // calculate the angles given pt C ***Inverse Kinematics***
  // returns an array with [A_angle,B_angle,T_angle] in radians
  
  float xy_len, c_len, sub_angle1, sub_angle2;
  static float angles[3] = {0.0,0.0,0.0};  // [ A , B , T ]
  static point c_new;
  static point z_vec = {0.0,0.0,1.0};
  float dot;

  c_len = vec_mag(c);
  c_new = cross_prod(z_vec,c);
  dot = acos(dot_prod(z_vec,c)/c_len);  // check for angle using dot, and if zero, arm is straight up

  // This method prevents math errors when the AB arm is straight up
  if (dot < 0.001) {
    angles[2] = 0.0;  // arm is straight up, turntable = 0
  } else {
    angles[2] = atan2(abs(c_new.y),c_new.x) - 90.0/RADIAN; // normal
  }

  if (c.x < 0.0) {  // reverse the turntable angle if c.x is negative
    angles[2] = -angles[2];
  }
  c_new = rot_pt_z(c,-angles[2]); // rotate the point c onto the XZ plane using turntable angle
  c_len = sqrt(pow((c_new.x-aOffset),2)+pow(c_new.z,2));   // XZ plane

  if (c_len < l_ab+l_bc) {
    // case where robot arm can reach
    sub_angle1 = atan2(c_new.z,(c_new.x-aOffset));
    sub_angle2 = acos((pow(c_len,2)+pow(l_ab,2)-pow(l_bc,2))/(2*c_len*l_ab));
    angles[0] = sub_angle1 + sub_angle2;
    angles[1] = acos((pow(l_bc,2)+pow(l_ab,2)-pow(c_len,2))/(2*l_bc*l_ab))-180.0/RADIAN;
  } else {
    // case where robot arm can not reach point... 
    angles[0] = atan2(c_new.z,(c_new.x-aOffset)); // a angle point in direction to go
    angles[1] = 0.0; // b is straight
  } 
  return angles;  // return the angles
}

point clawToC(point g, float absC, float thetaD, float s_CG_x, float s_CG_y, float s_CG_z) {
  // Claw pickup point is g.  Determine c based on joint c and d angles.
  static point c, c1, c2, d;
  static float thetaT;
  c.x = -s_CG_x;  c.y = 0.0;    c.z = -s_CG_z;
  d.x = 0.0;      d.y = s_CG_y; d.z = 0.0;
  c1 = rot_pt_x(c,thetaD);  // rotate D
  c2 = add_pts(c1,d);
  c1 = rot_pt_y(c2,absC); // rotate C
  thetaT = atan2((g.y+s_CG_y),g.x); // expected turntable angle
  c2 = rot_pt_z(c1,thetaT); // correct for turntable angle
  c1 = add_pts(c2,g);
  return c1;
}
point anglesToC(float a, float b, float t, float l_ab, float l_bc){
  // Apply translations and rotations to get the C point
  //   Forward Kinematics
  static point pB,pC, pTemp;
  
  pB.x = l_ab;   pB.y = 0.0;  pB.z = 0.0;
  pC.x = l_bc;   pC.y = 0.0;  pC.z = 0.0;
  
  pTemp = rot_pt_y(pC,b);  // rotate b
  pC = add_pts(pB,pTemp);  // add to AB arm
  pTemp = rot_pt_y(pC,a); // rotate a
  pC = rot_pt_z(pTemp,t);  // rotate turntable

  return pC;
}
line anglesToG(float a, float b, float t, float c, float d, float l_ab, float l_bc, float sCGx, float sCGy) {
  //  Apply translations and rotations to get from point C to G
  //   Forward Kinematics
  static point pB,pC, pG, pTemp;
  static line lCG;
  
  pB.x = l_ab;   pB.y = 0.0;  pB.z = 0.0;
  pC.x = l_bc;   pC.y = 0.0;  pC.z = 0.0;
  pG.x = sCGx;   pG.y = 0.0;  pG.z = sCGy;

  pTemp = rot_pt_x(pG,d);  // rotate d
  pG = rot_pt_y(pTemp,c);  // rotate c
  pTemp = add_pts(pC,pG);  // add G to BC arm
  pG = rot_pt_y(pTemp,b);  // rotate b
  pTemp = add_pts(pB,pG);     // add to AB arm
  pG = rot_pt_y(pTemp,a); // rotate a
  lCG.p2 = rot_pt_z(pG,t);  // rotate turntable

  pTemp = rot_pt_y(pC,b);  // rotate b
  pC = add_pts(pB,pTemp);  // add to AB arm
  pTemp = rot_pt_y(pC,a); // rotate a
  lCG.p1 = rot_pt_z(pTemp,t);  // rotate turntable

  return lCG;
}
void pot_map(joint & jt) {
  // Map a potentiometer value in millivolts to an angle
  // map(value, fromLow, fromHigh, toLow, toHigh), USES INTEGER MATH
  // NOTE: SCALE ANGLES (RADIANS) UP THEN DOWN TO GET PRECISION FROM POT VALUES
  jt.pot_angle = map(jt.pot_value, jt.pot.low_mv, jt.pot.high_mv, jt.pot.low_ang*1000, jt.pot.high_ang*1000) / 1000.0; 
  
  jt.target_angle = jt.pot_angle;  // assume that the two are equal for now, RADIANS
}

float servo_map(joint & jt) { // Map current joint angle to the servo microsecond value
  return floatMap(jt.current_angle, jt.svo.low_ang, jt.svo.high_ang, jt.svo.low_ms, jt.svo.high_ms);
}

boolean lineMoveG(arm & the_arm, point to_G, int mmps, float to_C, float to_D) { 
  // limits movement given feed rate (mmps)
  // updates .current_pt based on speed limit
  // RETURNS true if done moving, false if not
  static unsigned long mst;
  static float line_len, newC, newD;
  static point newG;
  
  mst = millis();
  the_arm.dt = mst - the_arm.prior_mst; // delta time
  the_arm.prior_mst = mst;
  the_arm.moveDist = mmps * (the_arm.dt/1000.0); // feed rate (mm/sec)*sec = mm
  
  line_len = ptpt_dist(the_arm.current_pt,to_G);
  if (the_arm.moveDist < line_len) { // PARTIAL MOVE
    newG = pt_on_line(the_arm.moveDist,line_len, the_arm.current_pt,to_G);
    the_arm.current_pt = newG;
    newC = interpolate(the_arm.moveDist,line_len, the_arm.jC.target_angle,to_C);
    the_arm.jC.current_angle = newC;
    newD = interpolate(the_arm.moveDist,line_len, the_arm.jD.target_angle,to_D);
    the_arm.jD.current_angle = newD;
    return false;
  } else { // FULL MOVE
    the_arm.current_pt = to_G;
    the_arm.jC.current_angle = to_C;
    the_arm.jD.current_angle = to_D;
    return true;
  }
}

void lineMoveC(arm & the_arm, point to_C, int mmps) { 
  // Move to point C
  // limits movement given feed rate (mmps)
  // updates .current_pt based on speed limit
  // RETURNS true if done moving, false if not
  static unsigned long mst;
  static float line_len;
  static point newG;
  
  mst = millis();
  the_arm.dt = mst - the_arm.prior_mst; // delta time
  the_arm.prior_mst = mst;
  the_arm.moveDist = mmps * (the_arm.dt/1000.0); // feed rate (mm/sec)*sec = mm
  
  line_len = ptpt_dist(the_arm.current_pt,to_C);
  if (the_arm.moveDist < line_len) { // PARTIAL MOVE
    newG = pt_on_line(the_arm.moveDist,line_len, the_arm.current_pt,to_C);
    the_arm.current_pt = newG;
  } else { // FULL MOVE
    the_arm.current_pt = to_C;
  }
}

void go_to_next_cmd(arm & the_arm, commands & the_cmds) {
  the_arm.n += 1; // go to the next command line
  the_arm.moveDist = 0.0;
  the_arm.timerStart = millis();
  if (the_arm.n >= the_cmds.num_cmds) { // end of command lines, stop
    the_arm.n = 9999; // signals end of commands
  }
}

void commands_loop(arm & the_arm, commands & the_cmds, float scale) { 
  // scale is used to scale the MMPS
  static point toCpt; 
  static float toabsC, toAlphaD;
  static boolean govenorDone;

  // read type of command
  if (the_arm.n != 9999) {
    switch (the_cmds.cmds[the_arm.n][0]) {
      case C_TIME: // DELAY (timer)
        if ((millis()-the_arm.timerStart) > the_cmds.cmds[the_arm.n][1]) {
          the_arm.prior_mst = millis();
          go_to_next_cmd(the_arm, the_cmds);        
        }
        break;
      case C_LINE: // line move
        if (the_arm.moveDist == 0.0) { // initialize line
            toCpt.x = the_cmds.cmds[the_arm.n][2];
            toCpt.y = the_cmds.cmds[the_arm.n][3];
            toCpt.z = the_cmds.cmds[the_arm.n][4];
            toabsC = the_cmds.cmds[the_arm.n][5]/RADIAN;
            toAlphaD = the_cmds.cmds[the_arm.n][6]/RADIAN;
            govenorDone = lineMoveG(the_arm, toCpt, the_cmds.cmds[the_arm.n][1]*scale,toabsC,toAlphaD); 

         } else {  // moving
            govenorDone = lineMoveG(the_arm, toCpt, the_cmds.cmds[the_arm.n][1]*scale,toabsC,toAlphaD); 
            if (govenorDone) {  
              go_to_next_cmd(the_arm, the_cmds);
            }
         }
         break;
       case C_CLAW: // Claw move, with timer to give claw time to move.
         the_arm.jCLAW.target_angle = the_cmds.cmds[the_arm.n][2]/RADIAN;  // set the claw angle
         if ((millis()-the_arm.timerStart) > the_cmds.cmds[the_arm.n][1]) {
            the_arm.prior_mst = millis();
            go_to_next_cmd(the_arm, the_cmds);        
          }
         break;
    } // end switch   
  }
}

void state_setup(arm & the_arm) { // Call every loop, to check for a state change
  if (the_arm.initialize) { 
    the_arm.n = 0; // reset array pointer
    the_arm.moveDist = 0.0;  // reset govenor distance
    //the_arm.cmd_size = 0;  // sizeof array.
    the_arm.initialize = false;
  }  
}

void logData(joint jt,char jt_letter) {
  Serial.print(",");
  Serial.print(jt_letter);
//  Serial.print(", p_value,");
//  Serial.print(jt.pot_value);
//  Serial.print(", Pang,");
//  Serial.print(jt.pot_angle*RADIAN,1);
  Serial.print(",");
//  Serial.print(",dsr_ang,");
  Serial.print(jt.target_angle*RADIAN,1);
} 

// <<<<<<<<<<<<<<<<<<<<<<<<END MOTION CONTROL CODE HERE

// BEGIN GLOBAL CODE

// GEOMETRY OF ARM:
#define LEN_AB 350.0     // Length of Input AB arm in mm
#define LEN_BC 380.0     // Length of Input BC arm in mm
#define S_TA 10.0       // offset in X of A axis from Turtable Axis
#define S_CG_X 180.0    // offset from joint C to G in X
#define S_CG_Y 38.0     // offset from joint C to G in Y
#define S_CG_Z 40.0     // offset from joint C to G in Z

struct arm make3; 
struct joint jS;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Command Values for picking 5 stack PowerPlay cones and placing on Mid height Junction
#define MMPS 700 // mm per second
#define X_PP 254 // y location for pick and place in mm
#define Y_MV 700 // x cone pick location in mm
#define Y_MV_NEG -400 // x cone place location in mm
#define FLOORH -60 // z of floor for picking cone from floor
#define MIDJUNTH 600 // z height of Mid Juction for placing
#define CONEH 32 // DELTA cone height STACKED mm
#define HORIZONTAL 0 // global C angle, all moves
#define ALPHADPICK 0 // global D for pick
#define ALPHADPLACE 0 // global D for place
#define CLAWCLOSE -50
#define CLAWOPEN 30
#define LINEDANG 0
#define LINEZ -50

static commands lineCmds = 
  {8,  // size of the array.  It gets set correct on startup
   {{C_CLAW,200,CLAWOPEN,0,0,0,0},
   {C_LINE,2000, 160,-S_CG_Z,LINEZ,  -90,LINEDANG}, // ready
   {C_CLAW,1000,CLAWOPEN,0,0,0,0}, // pause to pick 
   {C_CLAW,1000,CLAWCLOSE,0,0,0,0}, // close to pick camera
   {C_LINE,100, 160,-S_CG_Z,LINEZ+50,   -90,LINEDANG}, // line over
   {C_LINE,100, 700,-S_CG_Z,LINEZ+50,   -90,LINEDANG}, // line over
   {C_TIME,1000,0,0,0,0,0},    // pause
   {C_LINE,100, 160,-S_CG_Z,LINEZ+50,   -90,LINEDANG}}}; // line back
                           
static commands test2cmds =
   {7,
   {{C_CLAW,1000,CLAWOPEN,0,0,0,0},  // open the claw, wherever it is, 1 second
   {C_LINE,MMPS, X_PP,Y_MV,FLOORH+CONEH*7,  HORIZONTAL,ALPHADPICK}, // ready over cone 1
   {C_LINE,MMPS, X_PP,Y_MV,FLOORH+CONEH*5,  HORIZONTAL,ALPHADPICK}, // down to cone 1
   {C_CLAW,500,CLAWCLOSE,0,0,0,0}, // pick cone 1
   {C_LINE,MMPS, X_PP,Y_MV,FLOORH+CONEH*9,   HORIZONTAL,ALPHADPLACE}, // up with cone 1 
   {C_LINE,MMPS, X_PP,Y_MV_NEG,MIDJUNTH,     HORIZONTAL,ALPHADPLACE}, // Move over Junction
   {C_CLAW,500,CLAWOPEN,0,0,0,0}}}; // drop cone 1

void setup() {  // put your setup code here, to run once:
  make3 = set_arm({LEN_BC, 0.0, LEN_AB});
  make3.state = 1; 

  #if SERIALOUT  
    Serial.begin(9600); // baud rate
    //while (!Serial) {
    //  ; // wait for serial port to connect. Needed for native USB port only
    //} 
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
  make3.jA.pot = set_pot(0 ,134,  0/RADIAN, 895, 178/RADIAN); // good
  make3.jB.pot = set_pot(1 ,500,-90/RADIAN, 908,  0/RADIAN); // good
  //jC.pot = set_pot(3 , 116,-90/RADIAN, 903, 90/RADIAN); // C will not have a pot on Make3
  make3.jD.pot = set_pot(2 ,250, 60/RADIAN, 747, -60/RADIAN); 
  make3.jT.pot = set_pot(4 ,166, -90/RADIAN, 960, 90/RADIAN); // better
  make3.jCLAW.pot = set_pot(3 , 250,-50/RADIAN, 750, 50/RADIAN);  // input arm is limited to 250 to 750
  jS.pot = set_pot(5 , 0, 0/RADIAN, 1023, 280/RADIAN);  // to tune

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  make3.jA.svo = set_servo(0,  2.2/RADIAN, 478, 176.8/RADIAN, 2390); // good
  make3.jB.svo = set_servo(1, 0.0/RADIAN, 500,-175.0/RADIAN, 2320); // good
  make3.jC.svo = set_servo(2, -153.5/RADIAN, 475, 5.0/RADIAN, 2156); // good.  TO DO REPLACE WITH 270 DEG SERVO
  make3.jD.svo = set_servo(3,  -90.0/RADIAN,  811, 90.0/RADIAN, 2054); // good
  make3.jCLAW.svo = set_servo(4, -50.0/RADIAN,  900, 50.0/RADIAN, 1900); // 45 DEG = FULL OPEN, -45 = FULL CLOSE
  make3.jT.svo = set_servo(5,  0.0/RADIAN,  1650, 90.0/RADIAN, 450); // good, SERVO RANGE: -57 TO +92 DEG

  // INITIALIZE ANGLES FOR ARM
  set_joint(make3.jA, 130.0/RADIAN);  
  set_joint(make3.jB,  -130.0/RADIAN);
  set_joint(make3.jC,  -70.0/RADIAN);
  set_joint(make3.jD,   0.0/RADIAN);
  set_joint(make3.jT,    -10.0/RADIAN); 
  set_joint(make3.jCLAW,  45.0/RADIAN); 

  // INITIALIZE COMMAND ARRAYS
  //test2cmds.num_cmds= sizeof(test2cmds.cmds)/(SIZE_CMD_ARRAY*2); 
  //lineCmds.num_cmds = sizeof(lineCmds.cmds)/(SIZE_CMD_ARRAY*2); 
}

void loop() {  //########### MAIN LOOP ############
  // put your main code here, to run repeatedly:
  static int old_state = -1;
  static float *angles;
  static float alphaB;
  static point pointC;
  static float zFloor; 
  static float mmps_scale;
  
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
    //Serial.print(",cmd_size,");
    //Serial.print(make3.cmd_size);
    Serial.print(", N,"); // command line
    Serial.print(make3.n);
    //Serial.print(", CMD,");
    //Serial.print(cmd_array[make3.n][0]); 
  #endif
  
   switch (make3.state) {
    case 3: // DO NOTHING STATES
    case 5:
    case 10:
      //make3.prior_mst = millis();
      //  These commands should smoothly move the arm to the last saved position
      lineMoveC(make3, pointC, 500);   // Limits the speed of movement
      angles = inverse_arm_kinematics(make3.current_pt,LEN_AB,LEN_BC,S_TA); // find A, B, T angles
      make3.jA.current_angle = angles[0]; // global A = local A
      make3.jB.current_angle = angles[1];  // local B
      make3.jT.current_angle = angles[2];  // global T

      break;
    case 0:  
    case 1:
    case 2: // COMMAND CONTROL -- LINE
      
      if (make3.state == 0) mmps_scale = 2.0;
      if (make3.state == 1) mmps_scale = 0.5;
      if (make3.state == 2) mmps_scale = 1.0;
      
      commands_loop(make3,    lineCmds    ,mmps_scale); // get and calculate the moves
      
      pointC =  clawToC(make3.current_pt, make3.jC.current_angle, make3.jC.current_angle, S_CG_X, S_CG_Y,S_CG_Z);

      angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC,S_TA); 
      make3.jA.current_angle = angles[0]; // global A = local A
      make3.jB.current_angle = angles[1];  // local B
      make3.jT.current_angle = angles[2];  // global T
      make3.jC.current_angle = set_C_Abs(make3,make3.jC.target_angle);
      make3.jD.current_angle = make3.jD.target_angle; 
      
      break;
    case 4: // COMMAND CONTROL -- C = -90  WITH xxx
      mmps_scale = 1.0;
      commands_loop(make3,    test2cmds   ,mmps_scale); // get and calculate the moves

      pointC =  clawToC(make3.current_pt, make3.jC.current_angle, make3.jD.current_angle, S_CG_X, S_CG_Y,S_CG_Z);

      angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC,S_TA); 
      make3.jA.current_angle = angles[0]; // global A = local A
      make3.jB.current_angle = angles[1];  // local B
      make3.jT.current_angle = angles[2];  // global T
      make3.jC.current_angle = set_C_Abs(make3,-90.0/RADIAN);
      make3.jD.current_angle = make3.jD.target_angle; 
      
      break;
    case 6:  
    case 7:
    case 8: // MANUAL CONTROL WITH GOVENOR
      make3.jA.pot_value = analogRead(make3.jA.pot.analog_pin);  // read joint A
      make3.jB.pot_value = analogRead(make3.jB.pot.analog_pin);  // read joint B
      make3.jD.pot_value = analogRead(make3.jD.pot.analog_pin);  // read D wrist
      make3.jT.pot_value = analogRead(make3.jT.pot.analog_pin);  // read the turntable
      make3.jCLAW.pot_value = analogRead(make3.jCLAW.pot.analog_pin);  // read joint Claw
    
      pot_map(make3.jA);  // get A angle
      pot_map(make3.jB);  // get B angle
      pot_map(make3.jD); // Wrist
      pot_map(make3.jT); // Get Turntable angle
      pot_map(make3.jCLAW);   // get Claw angle

      pointC = anglesToC(make3.jA.current_angle,make3.jB.current_angle,make3.jT.current_angle,LEN_AB,LEN_BC);
      lineMoveC(make3, pointC, 300);   // Limits the speed of movement
      
      if (make3.state == 6) {
        make3.jC.current_angle = set_C_Abs(make3,-90.0/RADIAN);
        zFloor = 100.0;
      }
      if (make3.state == 7) {
        make3.jC.current_angle = set_C_Abs(make3,-45.0/RADIAN);
        zFloor = 50.0;
      }
      if (make3.state == 8) {
        make3.jC.current_angle = set_C_Abs(make3, 0.0);
        zFloor = 30.0;
      }
      if (pointC.z < zFloor) {
        //pointC.z = zFloor;  // this keeps the arm from doing a pushup and frying a servo
        make3.current_pt.z = zFloor;
      }
      angles = inverse_arm_kinematics(make3.current_pt,LEN_AB,LEN_BC,S_TA); // find A, B, T angles
      make3.jA.current_angle = angles[0]; // global A = local A
      make3.jB.current_angle = angles[1];  // local B
      make3.jT.current_angle = angles[2];  // global T

      break;
    case 9:  // MANUAL CONTROL  
      make3.jA.pot_value = analogRead(make3.jA.pot.analog_pin);  // read joint A
      make3.jB.pot_value = analogRead(make3.jB.pot.analog_pin);  // read joint B
      make3.jD.pot_value = analogRead(make3.jD.pot.analog_pin);  // read D wrist
      make3.jT.pot_value = analogRead(make3.jT.pot.analog_pin);  // read the turntable
      make3.jCLAW.pot_value = analogRead(make3.jCLAW.pot.analog_pin);  // read joint Claw
    
      // Direct (full speed) method
      pot_map(make3.jA);
      pot_map(make3.jB);
      pot_map(make3.jCLAW); 
      make3.jC.current_angle = set_C_Abs(make3,-90.0/RADIAN);
      pot_map(make3.jD); // Wrist  TO DO  subtract turntable?
      pot_map(make3.jT); // Turntable

      // The following lines are called to store the C point
      pointC = anglesToC(make3.jA.current_angle,make3.jB.current_angle,make3.jT.current_angle,LEN_AB,LEN_BC);
      lineMoveC(make3, pointC, 400);   // Limits the speed of movement
    
      break;
  }
  
  // HARD LIMITS
  if (make3.jA.current_angle > 150.0/RADIAN) make3.jA.current_angle = 150.0/RADIAN; // keep arm from going into baby arm
  if (make3.jCLAW.current_angle > 18.0/RADIAN) make3.jCLAW.current_angle = 18.0/RADIAN;
  if (make3.jCLAW.current_angle < -49.0/RADIAN) make3.jCLAW.current_angle = -49.0/RADIAN;

  // Convert CURRENT angle to PWM signal and send 
  pwm.writeMicroseconds(make3.jA.svo.digital_pin, servo_map(make3.jA)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jB.svo.digital_pin, servo_map(make3.jB)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jC.svo.digital_pin, servo_map(make3.jC)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jD.svo.digital_pin, servo_map(make3.jD)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jCLAW.svo.digital_pin, servo_map(make3.jCLAW)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jT.svo.digital_pin, servo_map(make3.jT)); // Adafruit servo library

  #if SERIALOUT
    Serial.print(",G,");
    Serial.print(make3.current_pt.x);
    Serial.print(",");
    Serial.print(make3.current_pt.y);
    Serial.print(",");
    Serial.print(make3.current_pt.z);
  
    logData(make3.jA,'A');
//    logData(make3.jB,'B');
//    logData(make3.jC,'C');
//    logData(make3.jD,'D');
//    logData(make3.jT,'T');
//    logData(make3.jCLAW,'X');
//      logData(make3.jS,'S');
    Serial.println(", END");
  #endif
}
//
