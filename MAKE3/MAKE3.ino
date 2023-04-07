/* ROBOT ARM CONTROL SOFTWARE FOR MAKE3 ROBOT ARM
 *  By, SrAmo, April 2023
 */
//#include <Wire.h>  // does not seem to be needed 
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates, from Adafruit
#define RADIAN 57.2957795  // number of degrees in one radian
#define RAMP_START_DIST 50 // distance at which velocity ramp-down begins (mm) used in velocity control
#define ZERO_DIST 0.5     // value used to determine if distance is zero (mm)

// STATES (or programs)  Controlled by the selector potentiometer.
#define S_SERIAL 0      // enter a serial command from serial.input
#define S_TELEOP_1 1    // C at -90 absolute, D at 0 absolute
#define S_AUTO_LINE_Y 2 // Back and forth in Y direction, with Aim point at +x, y=0
#define S_TELEOP_3 3    // C at 90 absolute, D at 0 absolute
#define S_TELEOP_4 4    // C at -90 absolute, D relative (controlled)
#define S_AUTO_LINE_X 5 // Back and forth in X direction, with Aim point at y=?, x=mid
#define S_TELEOP_6 6    // C at -90 absolute, D relative (controlled)
#define S_AUTO_ARC_XY 7  // Arc in Y direction, with Aim point in middle of arc
#define S_TELEOP_8 8    // C at -90 absolute, D relative (controlled)
#define S_TELEOP_9 9    // C at 45 absolute
#define S_TELEOP_10 10  // C at 0 absolute
// UNUSED STATES
#define S_AUTO_LINE_Z 13 // C at 90 abs, D loc from last
#define S_AUTO_CONES 17  // WAS 7

// MODES in which the arm end effector can operate (joints C and D)
// Note: For each loop the arm C point is obtained (from multiple methods) and angles A, B and T are solved (inverse kinematics)
#define M_CD_LOC 0
#define M_CD_ABS 1
#define M_C_LOC_D_ABS 2
#define M_C_ABS_D_LOC 3
#define M_CD_AIM 4

/*
 *  COMMAND CODES: All codes are integers
 *  Feed rate is in mm per second.
 *  Angle rate is in Degrees per second.
 *  Angles given in Degrees.  Locations in mm.
 */
// Synchronous commands (completes command before next command is sent)
#define K_TIMER 1    // timer {millisconds}
#define K_LINE_C 2   // line move to new C point   {feed rate, x,y,z}
//#define K_LINE_G 3   // line move to new G point {feed rate, x,y,z}
#define K_ARC_Z 4  // moves in a arc on a Z plane, from current pt (x,y,z) around (x,0,z), radius = abs(y) {feed rate, CW-0 or CCW-1}
// Asynchronous commands (does not wait to be completed)
#define K_C_LOC 12    // C local move           {angle rate, target angle}
#define K_C_ABS 22    // C Absolute Angle Mode, {angle rate, target angle}
#define K_D_LOC 13    // D local move           {angle rate, target angle}
#define K_D_ABS 23     // D Absolute Angle Mode {angle rate, target angle}
#define K_CLAW 14   // claw local move      {angle rate, target angle}
#define K_AIM 5     // Use AIM mode for given point {x,y,z}
#define K_GOTO 9    // Go back to given command line {command line number} 

// GEOMETRY OF ARM:
#define LEN_AB 350.0     // Length of Input AB arm in mm
#define LEN_BC 380.0     // Length of Input BC arm in mm
#define FLOOR 0.0     // Distance from A joint to Floor in mm (-150 mm without camera, -50 with camera)
#define S_CG_X 90.0    // offset from joint C to G in X.  180 is a claw value,  90 is GoPro
#define S_CG_Y -38.0     // offset from joint C to G in Y.  -38 is GoPro
#define S_CG_Z 0.0     // offset from joint C to G in Z.  40 is Claw.  0 is GoPro

// Command Values for picking 5 stack PowerPlay cones and placing on Mid height Junction
#define MMPS 600 // mm per second
#define X_PP 254 // y location for pick and place in mm
#define Y_MV 300 // x cone pick location in mm (was 700)
#define Y_MV_NEG -200 // x cone place location in mm (was -400)
#define FLOORH -40 // z of floor for picking cone from floor
#define MIDJUNTH 400 // z height of Mid Juction for placing (was 600)
#define CONEH 32 // DELTA cone height STACKED mm
#define CLAWCLOSE -50
#define CLAWOPEN 10

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

struct servo {
  // Values are pulse width in microseconds (~400 to ~2400--SERVOS SHOULD BE TESTED)
  // 270 deg servo, gets 290 deg motion with 400 to 2500 microsecond range
  // 180 deg servo, gets 160 deg motion with 500 to 2800 microsecond range
  uint8_t digital_pin; // Adafruit digital pin number
  int low_ms; // low microsecond value, from about 500 to 2400
  float low_ang; // corresponding angle at low microsecond, in radians
  int high_ms; // high microsecond value, from about 500 to 2400
  float high_ang; // corresponding angle at high microsecond, in radians
}; 

struct joint {
  potentiometer pot; // Sub structure which contains static pot information
  servo svo; // Sub structure which contains the static servo information
  int pot_value;  // current potentiometer value in millivolts
  float pot_angle; // Potentiometer arm angle, if used, in radians
  float current_angle; // last issued command for angl, in radians
  float target_angle;  // where the angle should end up, in radians
  float target_velocity;  // radians per millisecond
};

struct arm {  // as in Robot Arm
  int state; // The different ways that the arm can be controlled. Set by S potentiometer
  int mode; // The mode in which the arm runs, primarily the end effector (C and D joints)
  int n; // current active index in the command array
  unsigned long prior_mst; // clock time (microseconds) prior loop 
  unsigned long dt; // delta loop time in ms
  unsigned long timerStart; // timer, for delays
  float feedRate; // feed rate for point moves, in mm/sec (converted to /microsec)
  boolean rampDown;  // controlls if a velocity ramp down should be used
  point current_pt; // current (C or G) point 
  point target_pt;  // target C or G point
  point aim_pt;   // aiming point 
  float line_len;  // calculated distance from current_pt to target_pt
  joint jA;
  joint jB;
  joint jC;
  joint jD;
  joint jCLAW;
  joint jT;
};

#define SIZE_CMD_ARRAY 5  // NUMBER OF VALUES PER COMMAND. Command array is 2 dimensional. This is number of commands per row.
struct command {
  int arg[SIZE_CMD_ARRAY];
};

struct sequence {  // An array of commands
  int num_cmds;
  command cmd[];  // undefined array length
};

// Path of Servo Angles.  Stored as integer to minimize memory usage.  Scale by at least 1000 for smooth results
#define PATH_SIZE 20
#define ANGLE_SCALE 1000   // used to store radians in integer format (radian*SCALE)
struct pathAngles {
  int a[PATH_SIZE];
  int b[PATH_SIZE];
  int c[PATH_SIZE];
  int d[PATH_SIZE];
  int t[PATH_SIZE];        
};

//   GOBAL VARIABLES 
arm make3; 
joint jS;  // selector pot
pathAngles pathA; // for storing pre-computed servo angles

sequence sixCmds = {6,{
   {K_TIMER,1000,0,0,0},    // pause
   {K_LINE_C,100, 160,150,150}, // point 1
   {K_TIMER,1000,0,0,0},    // pause
   {K_LINE_C,100, 300,150,150}, // point 2
   {K_TIMER,1000,0,0,0},    // pause
   {K_LINE_C,100, 160,150,150}, // point 1
   }}; 
                           
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// SETTERS,  INITIALIZERS
potentiometer initPot(int pin, int lowmv, float lowang, int highmv, float highang) {
  // initPot(pin,lowmv,lowang,highmv,highang)
  // Note: low and high values do not need to be  at the extreems
  struct potentiometer pot;
  pot.analog_pin = pin;
  pot.low_mv = lowmv;
  pot.low_ang = lowang;
  pot.high_mv = highmv;
  pot.high_ang = highang;
  return pot;
}

servo initServo(int pin, float lowang, int lowms, float highang, int highms) {
  // initServo(pin,lowang,lowms,highang,highmv)
  // Note: low and high values do not need to be  at the extreems
  struct servo svo;
  svo.digital_pin = pin;
  svo.low_ang = lowang;
  svo.low_ms = lowms;
  svo.high_ang = highang;
  svo.high_ms = highms;
  return svo;
}

void initJoint(joint & jt, float initial_angle) {
  // Sets initial joint values
  // Note: can't read physical servo angle, so servos can jump on startup
  jt.current_angle = initial_angle;
  jt.target_angle = initial_angle;
  jt.target_velocity = 60.0/RADIAN/1000.0;  // default initial velocity, radians per microsecond
}

void initArm(arm & the_arm, point pt) { // starting point for arm
  // initialize arm structure
  the_arm.state = 0;
  the_arm.mode = M_CD_ABS;  
  the_arm.n = -1;  // first command, USE -1 TO INDICATE NOT RUNNING A COMMAND
  the_arm.prior_mst = millis();
  the_arm.dt = 10;  
  the_arm.timerStart = millis();
  the_arm.feedRate = 300.0;  
  the_arm.rampDown = true; 
  the_arm.current_pt = pt;
  the_arm.target_pt = pt;
  the_arm.aim_pt = pt;
  the_arm.aim_pt.x = the_arm.aim_pt.x + 300.0;
  the_arm.line_len = 0.0;
}

float getCang(float a_angle, float b_angle, float fixed_c_angle) { // returns joint angle C, using A and B
  // Assumes that you want an absolute (fixed) C angle
  return -a_angle - b_angle + fixed_c_angle;
}

//  ###############  MATH FUNCTIONS ###########################
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

point rot_pt_x(point p1, float a) { // Rotate p1 about X axis by a radians
  point p2;
  p2.x = p1.x;
  p2.y = p1.y*cos(a)-p1.z*sin(a);
  p2.z = p1.y*sin(a)+p1.z*cos(a);
  return p2;
}

point rot_pt_y(point p1, float a) { // Rotate p1 about Y axis by a radians
  point p2;
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
  point p3;
  p3.x = p1.x+p2.x;   p3.y = p1.y+p2.y;   p3.z = p1.z+p2.z;
  return p3;
}
float ptpt_dist(point p1, point p2) {
  float dist;
  dist = sqrt(pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0)+pow(p1.z-p2.z,2.0) ); 
  return dist;
}

point pt_on_line(float s, float dist, point p1, point p2) { // point along line s from p1
  point new_pt;
  float ratio;
  ratio = s / dist; 
  new_pt.x = ratio*(p2.x-p1.x) + p1.x;
  new_pt.y = ratio*(p2.y-p1.y) + p1.y;
  new_pt.z = ratio*(p2.z-p1.z) + p1.z;
  return new_pt;
}

float interpolate(float s, float dist, float p1, float p2) { // dist along s from p1
  float new_pt;
  float ratio;
  ratio = s / dist; 
  new_pt = ratio*(p2-p1) + p1;
  return new_pt;
}

float * inverse_arm_kinematics(point c, float l_ab, float l_bc) {
  // Given arm system GroundT-TA-AB-BC, where T (turntable) and A are [0,0,0]
  // The location of joint C (point c) and the lengths AB (l_ab) and BC (l_bc) are specified
  // The joints A,B are on a turntable with rotation T parallel to Z through A
  // With T_angle = 0, then joints A & B are parallel to the Y axis
  // Calculate the angles given pt C ***Inverse Kinematics***
  // returns an array with [A_angle,B_angle,T_angle] in radians
  
  float c_len, sub_angle1, sub_angle2;
  float angles[3] = {0.0,0.0,0.0};  // [ A , B , T ]
  point c_new;
  static float tAngMemory = 0.0;  // THIS NEEDS TO BE STATIC

  c_len = sqrt(pow((c.x),2)+pow(c.y,2));   // C on XY plane

  // This method prevents math errors when the AB arm is straight up
  if (c_len < ZERO_DIST) {
    angles[2] = tAngMemory;  // arm is straight up, use the prior T angle
  } else {
    angles[2] = atan2(c.y,abs(c.x));   // turntable angle, compute with pos x
    tAngMemory = angles[2];
  }

  if (c.x < 0.0) {  // reverse the turntable angle if c.x is negative
    angles[2] = -angles[2];
  }
  c_new = rot_pt_z(c,-angles[2]); // rotate the point c onto the XZ plane using turntable angle
  c_len = sqrt(pow((c_new.x),2)+pow(c_new.z,2));   // XZ plane, reuse variable c_len

  if (c_len < l_ab+l_bc) {
    // case where robot arm can reach
    sub_angle1 = atan2(c_new.z,c_new.x);
    sub_angle2 = acos((pow(c_len,2)+pow(l_ab,2)-pow(l_bc,2))/(2*c_len*l_ab));
    angles[0] = sub_angle1 + sub_angle2;
    angles[1] = acos((pow(l_bc,2)+pow(l_ab,2)-pow(c_len,2))/(2*l_bc*l_ab))-180.0/RADIAN;
  } else {
    // case where robot arm can not reach point... 
    angles[0] = atan2(c_new.z,c_new.x); // a angle point in direction to go
    angles[1] = 0.0; // b is straight
  } 
  return angles;  // return the angles
}

point anglesToC(float a, float b, float t, float l_ab, float l_bc){  // Forward Kinematics
  // Apply translations and rotations to get the C point
  point pB,pC, pTemp;
  
  pB.x = l_ab;   pB.y = 0.0;  pB.z = 0.0;
  pC.x = l_bc;   pC.y = 0.0;  pC.z = 0.0;
  
  pTemp = rot_pt_y(pC,b);  // rotate b
  pC = add_pts(pB,pTemp);  // add to AB arm
  pTemp = rot_pt_y(pC,a); // rotate a
  pC = rot_pt_z(pTemp,t);  // rotate turntable

  return pC;
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

void updateArmPtC(arm & the_arm) { // Moves current point C toward target_pt at given feed rate (mmps)
  float moveDist,newFeedRate,distC;
  the_arm.line_len = ptpt_dist(the_arm.current_pt,the_arm.target_pt);
  // cut the feedrate when the current pt x,y component is close to zero.  To stop the rocking dynamics. 
  distC = sqrt(pow(the_arm.current_pt.x,2)+pow(the_arm.current_pt.y,2));
  newFeedRate = the_arm.feedRate;
  if (the_arm.rampDown) {
    if (distC < 100) {
      newFeedRate = newFeedRate*((distC+50)/150);
    };
    // ADDED RAMP DOWN ON 3/18/2023  
    if (the_arm.line_len < RAMP_START_DIST) {
      newFeedRate = the_arm.feedRate*(the_arm.line_len/RAMP_START_DIST);  // scale down based on distance
    } else {
      newFeedRate = the_arm.feedRate;  // keep using the current feedrate
    };
  };
  moveDist = newFeedRate/1000.0 * (the_arm.dt); // dist(mm) = feed rate(mm/ms)*dt(ms)  
  the_arm.current_pt = pt_on_line(moveDist,the_arm.line_len, the_arm.current_pt,the_arm.target_pt);
}

void logPoint(arm & the_arm) {
  Serial.print(make3.state);
  Serial.print(",cmdNo=");
  Serial.print(make3.n);        
  Serial.print(",");
  Serial.print(make3.prior_mst);        
  Serial.print(",");
  Serial.print(make3.dt);        
  Serial.print(",");
  Serial.print("CRNT PT,");
  Serial.print(the_arm.current_pt.x);
  Serial.print(",");
  Serial.print(the_arm.current_pt.y);
  Serial.print(",");
  Serial.print(the_arm.current_pt.z);
  Serial.print(",AIM PT,");
  Serial.print(the_arm.aim_pt.x);
  Serial.print(",");
  Serial.print(the_arm.aim_pt.y);
  Serial.print(",");
  Serial.print(the_arm.aim_pt.z);
  Serial.print(",jA_CRNT_ANG,");
  Serial.print(the_arm.jA.current_angle*RADIAN,1);
  Serial.print(",jB_CRNT_ANG,");
  Serial.print(the_arm.jB.current_angle*RADIAN,1);
  Serial.print(",jC_CRNT_ANG,");
  Serial.print(the_arm.jC.current_angle*RADIAN,1);
  Serial.print(",jD_CRNT_ANG,");
  Serial.print(the_arm.jD.current_angle*RADIAN,1);
  //Serial.print(",TRGT PT,");
  //Serial.print(the_arm.target_pt.x);
  //Serial.print(",");
  //Serial.print(the_arm.target_pt.y);
  //Serial.print(",");
  //Serial.print(the_arm.target_pt.z);
  Serial.println(",");
}
void logCmds(sequence & the_cmds, char *string) {
  Serial.println("logCmds");
  int i;
  for (int i=0;i<the_cmds.num_cmds;++i) {
    Serial.print(string);
    Serial.print(i); 
    for (int j=0;j<SIZE_CMD_ARRAY;++j) {
      Serial.print(",");
      Serial.print(the_cmds.cmd[i].arg[j]); 
    }
    Serial.println("."); 
  }
} 
void logData(joint jt,char jt_letter) {
  Serial.print(make3.state);
  Serial.print(",cmdNo=");
  Serial.print(make3.n);        
  Serial.print(",");
  Serial.print(make3.prior_mst);        
  Serial.print(",");
  Serial.print(make3.dt);        
  Serial.print(",");
  Serial.print(jt_letter);
  //Serial.print(", p_value,");
  //Serial.print(jt.pot_value);
  //Serial.print(", Pang,");
  //Serial.print(jt.pot_angle*RADIAN,1);
  Serial.print(",crnt_ang,");
  Serial.print(jt.current_angle*RADIAN,1);
  Serial.print(",trgt_ang,");
  Serial.print(jt.target_angle*RADIAN,1);
  Serial.print(",A_crnt_ang,");
  Serial.print(make3.jA.current_angle*RADIAN,1);
  Serial.print(",B_crnt_ang,");
  Serial.print(make3.jB.current_angle*RADIAN,1);

  //Serial.print(",Servo (microsec),");
  //Serial.print(servo_map(jt));

  //Serial.print(",target_vel (DEG/SEC),"); // ONLY USED FOR JOINTS C AND D
  //Serial.print(jt.target_velocity*RADIAN*1000.0,1);
  Serial.println(".");
} 

boolean runCommand(arm & the_arm, command  & the_cmd) { // TRUE if DONE, FALSE is NOT DONE
  // read type of command
  switch (the_cmd.arg[0]) {
    case 0:
      return true;
      break;
    case K_TIMER: // DELAY (timer)
      if ((millis()-the_arm.timerStart) > the_cmd.arg[1]) {
        return true;
      } else
        return false;
      break;
    case K_C_LOC:  // Move C servo to a local angle
      // arg[1] is the velocity C, DEG per Second
      if (the_cmd.arg[1] < 10) the_cmd.arg[1] = 10; // NO Negatives or Zeros
      the_arm.jC.target_velocity = the_cmd.arg[1]/RADIAN/1000.0;
      the_arm.jC.target_angle = the_cmd.arg[2]/RADIAN;  // arg[2] is NEW ANGLE FOR C
      the_arm.mode = M_CD_LOC;      
      return true;
      break;
    case K_C_ABS:  // move C servo relative to ground (absolute)
      // arg[1] is the velocity C, DEG per Second
      if (the_cmd.arg[1] < 10) the_cmd.arg[1] = 10; // NO Negatives or Zeros
      the_arm.jC.target_velocity = the_cmd.arg[1]/RADIAN/1000.0;
      the_arm.jC.target_angle = the_cmd.arg[2]/RADIAN;  // arg[2] is NEW ANGLE FOR C
      the_arm.mode = M_C_ABS_D_LOC;      
      return true;
      break;
    case K_D_LOC:  // D servo
      // arg[1] is the velocity D, DEG per Second
      if (the_cmd.arg[1] < 10) the_cmd.arg[1] = 10; // NO Negatives or Zeros
      the_arm.jD.target_velocity = the_cmd.arg[1]/RADIAN/1000.0;
      the_arm.jD.target_angle = the_cmd.arg[2]/RADIAN;  // arg[2] is NEW ANGLE FOR D
      the_arm.mode = M_C_ABS_D_LOC;      
      return true;
      break;
    case K_D_ABS:  // move D servo relative to ground (absolute)
      // arg[1] is the velocity D, DEG per Second
      if (the_cmd.arg[1] < 10) the_cmd.arg[1] = 10; // NO Negatives or Zeros
      the_arm.jD.target_velocity = the_cmd.arg[1]/RADIAN/1000.0;
      the_arm.jD.target_angle = the_cmd.arg[2]/RADIAN;  // arg[2] is NEW ANGLE FOR D
      the_arm.mode = M_CD_ABS;      
      return true;
      break;
    case K_CLAW:  // CLAW servo
      // arg[1] is the claw velocity, DEG per Second
      if (the_cmd.arg[1] < 10) the_cmd.arg[1] = 10; // NO Negatives or Zeros
      the_arm.jCLAW.target_velocity = the_cmd.arg[1]/RADIAN/1000.0;
      the_arm.jCLAW.target_angle = the_cmd.arg[2]/RADIAN;  // arg[2] is NEW ANGLE FOR Claw
      return true;
      break;
    case K_LINE_C: // line move, from C current_pt to given C target_pt
      the_arm.feedRate = the_cmd.arg[1];
      the_arm.target_pt.x = the_cmd.arg[2];
      the_arm.target_pt.y = the_cmd.arg[3];
      the_arm.target_pt.z = the_cmd.arg[4];
      
      if (the_arm.line_len < ZERO_DIST) {  // Distance from current to target is small
        return true;
      } else { // return false to keep the next command from running
        return false;
      }
      break;
    case K_ARC_Z:  // move in circle on z plane
      /*   REPLACE WITH NEW METOD
      static int rad;
      static point center = {100,0,100};
      static float current_angle; // radians
      static int angle_sign = 1;
      static boolean circleInitialize = true;
      //float dist;
      // first time reading the arc command, set variables
      if (circleInitialize) {  // setup circle move
        the_arm.feedRate = the_cmd.arg[1];  // feed rate in mm/sec
        rad = abs(the_arm.target_pt.y); 
        if (rad < 10) rad = 10;  // mm
        // store circle center, only x in needed
        center.x = the_arm.target_pt.x;
        if (the_cmd.arg[2] == 1) angle_sign = -1; // reverse direction if requested
        else angle_sign = 1;
        rad = rad*angle_sign;        
        current_angle = 0.0; 
        the_arm.target_pt.x = center.x;
        the_arm.target_pt.y = center.y + rad;
        circleInitialize = false;  // don't execute setup again
      }
      //the_arm.line_len = ptpt_dist(the_arm.current_pt,the_arm.target_pt);  // SHOULDN'T DO THIS TWICE??
      if (abs(current_angle) > PI) {  // do half circle
        circleInitialize = true;
        current_angle = 0.0; 
        return true;  // done with command
      } else if (the_arm.line_len < ZERO_DIST) {  // move to next segment of circle
        the_arm.target_pt.x = center.x - rad*sin(current_angle);
        if (the_arm.target_pt.x < 10.0) the_arm.target_pt.x = 10.0; // Clip -x! keep the turntable from fast reversing
        the_arm.target_pt.y = center.y + rad*cos(current_angle);
        //current_angle = current_angle + ANGLE_INC*angle_sign; 
        return false; 
      }  else { // let the Arm Loop Update
        return false;  // stay on current segment of circle      
      } */
      break;
    case K_GOTO:  // to be made more robust
      the_arm.n = the_cmd.arg[1]-1;  //  set the command pointer to the first value
      return true;
      break;
    case K_AIM:
      the_arm.mode = M_CD_AIM;      
      the_arm.aim_pt.x = the_cmd.arg[1];
      the_arm.aim_pt.y = the_cmd.arg[2];
      the_arm.aim_pt.z = the_cmd.arg[3];
      return true;      
      break;
    default:
      return true;
      break;          
  } // end switch   
}

void readCommands(arm & the_arm, sequence & the_cmds, char *string) {
  // step through commands
  command cmdArray;
  //static boolean newCmd = true;
  if (the_arm.n > the_cmds.num_cmds) {  // DONE RUNNING ARRAY OF COMMANDS
    return;  
  } else {
    // pull out the one command from the sequence
    cmdArray = the_cmds.cmd[the_arm.n];
    if (runCommand(the_arm, cmdArray)) { // true = go to the next command
      the_arm.n += 1; // go to the next command line
      the_arm.line_len = 0.0; // this tells arcs that its the start of an arc
      the_arm.timerStart = millis();  // for timed commands
    };
    return;
  };
}

void setCmd(command & cmd, int v0, int v1, int v2, int v3, int v4) {
  cmd.arg[0] = v0;
  cmd.arg[1] = v1;
  cmd.arg[2] = v2;
  cmd.arg[3] = v3;
  cmd.arg[4] = v4;
}

void getCmdSerial(command & cmd) { // read command from serial port
  // If cmd[0] = 0 then command has NOT been read
  int i;
  
  cmd.arg[0] = 0;
  
  if (Serial.available() > 0){

    for (i=0;i<SIZE_CMD_ARRAY;++i) 
      cmd.arg[i] = 0;  // initialize

    i=0;
    while (Serial.available() > 0) {
      cmd.arg[i] = Serial.parseInt(); // get available ints
      ++i;
    }    
    // print the command
    Serial.print("Command: ");
    for (i=0;i<SIZE_CMD_ARRAY;++i) {
      Serial.print(cmd.arg[i]);
      Serial.print(",");
    }
    Serial.println("END");
  }
}
void path_orbit_xy(pathAngles & the_pathA,point center,int rad) {  // build 180 deg arc path about center, symetric about y=0 plane
  int i;  
  static float current_angle; // radians
  float angle_inc; // radians  
  static float *angles;
  point c,d;
  float d_len;
  float dc_vect_ang;
  current_angle = 90.0 / RADIAN; 
  angle_inc = 180.0/ RADIAN / (PATH_SIZE-1);
  d.z = center.z;
  c.z = d.z + S_CG_X;  // since c joint is rotated -90, then x becomes z
  for(i=0;i<PATH_SIZE;++i) {
    d.x = center.x + rad*cos(current_angle);  // point d traces the arc
    d.y = center.y + rad*sin(current_angle);
    // compute the DC vector angle.  DC vector mag is S_CG_Y. 
    d_len = ptpt_dist({0,0,0},{d.x,d.y,0});
    dc_vect_ang = asin(d.y/d_len) + asin(-S_CG_Y/d_len) + 90.0/RADIAN;
    c.x = d.x - S_CG_Y*cos(dc_vect_ang); // compute the C point
    c.y = d.y - S_CG_Y*sin(dc_vect_ang);   
    angles = inverse_arm_kinematics(c,LEN_AB,LEN_BC); // feed c to the inverse function
    the_pathA.a[i] = angles[0]*ANGLE_SCALE;   // CONVERT TO INTEGER AND DEGREES FOR OPENSCAD SIMULATION
    the_pathA.b[i] = angles[1]*ANGLE_SCALE;
    the_pathA.c[i] = getCang(angles[0],angles[1],-90.0/RADIAN)*ANGLE_SCALE;
    //the_pathA.d[i] = current_angle*ANGLE_SCALE;
    the_pathA.d[i] = (angles[2] - current_angle)*ANGLE_SCALE;
    //the_pathA.t[i] = dc_vect_ang*ANGLE_SCALE;
    the_pathA.t[i] = angles[2]*ANGLE_SCALE;
    current_angle = current_angle + angle_inc;
  }
}
void print_pathAngles(pathAngles & the_pathA) {  // Print servo angle path for OpenSCAD
  int i;  
  Serial.print("[");
  for(i=0;i<PATH_SIZE;++i) {
    Serial.print("[");
    Serial.print((float)the_pathA.a[i]/ANGLE_SCALE*RADIAN,1);
    Serial.print(",");
    Serial.print((float)the_pathA.b[i]/ANGLE_SCALE*RADIAN,1);
    Serial.print(",");
    Serial.print((float)the_pathA.c[i]/ANGLE_SCALE*RADIAN,1);
    Serial.print(",");
    Serial.print((float)the_pathA.d[i]/ANGLE_SCALE*RADIAN,1);
    Serial.print(",");
    Serial.print((float)the_pathA.t[i]/ANGLE_SCALE*RADIAN,1);
    Serial.println("],");
  }
}
void setup() {  // setup code here, to run once:
  pinMode(13,OUTPUT); // LED on Arduino board
  digitalWrite(13,HIGH); // Turn the LED on
    
  Serial.begin(115200); // baud rate
  delay(2500);  // serial output needs a delay

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
  // initPot(pin,lowmv,lowang,highmv,highang)
  make3.jA.pot = initPot(0 ,134,  0/RADIAN, 895, 178/RADIAN); // good
  make3.jB.pot = initPot(1 ,500,-90/RADIAN, 908,  0/RADIAN); // good
  //jC.pot = initPot(3 , 116,-90/RADIAN, 903, 90/RADIAN); // C will not have a pot on Make3
  make3.jD.pot = initPot(2 ,250, 90/RADIAN, 747, -90/RADIAN); 
  make3.jT.pot = initPot(4 ,348, -45/RADIAN, 797, 45.0/RADIAN); // 3/18/2023, fixed sign
  make3.jCLAW.pot = initPot(3 , 250,-50/RADIAN, 750, 50/RADIAN);  // input arm is limited to 250 to 750
  jS.pot = initPot(5 , 0, 0/RADIAN, 1023, 280/RADIAN);  // to tune

  // TUNE SERVO LOW AND HIGH VALUES
  // initServo(pin,lowang,lowms,highang,highms)
  make3.jA.svo = initServo(0,  45.0/RADIAN, 990, 135.0/RADIAN, 2010); // set 3/7/2023
  make3.jB.svo = initServo(1, 0.0/RADIAN, 500,-175.0/RADIAN, 2320); // good
  make3.jC.svo = initServo(2, -90.0/RADIAN, 927, 0.0/RADIAN, 1410); // set 3/7/2023
  make3.jD.svo = initServo(3,  -90.0/RADIAN,  811, 90.0/RADIAN, 2054); // good.  Can travel from -135 to 135 deg
  make3.jCLAW.svo = initServo(4, -50.0/RADIAN,  900, 50.0/RADIAN, 1900); // 45 DEG = FULL OPEN, -45 = FULL CLOSE
  make3.jT.svo = initServo(5,  60.0/RADIAN,  2320, -60.0/RADIAN, 480); // 3/18/2023 fixed sign.  Servo range is about -60 to 60 deg.

  // INITIALIZE ANGLES FOR ARM
  initJoint(make3.jA, 130.0/RADIAN);  
  initJoint(make3.jB,  -130.0/RADIAN);
  initJoint(make3.jC,  -70.0/RADIAN);
  initJoint(make3.jD,   0.0/RADIAN);
  initJoint(make3.jT,    -10.0/RADIAN); 
  initJoint(make3.jCLAW,  17.0/RADIAN); 

  initArm(make3,{LEN_BC/2, 0.0, LEN_AB});
  make3.state = 1; 
  
  digitalWrite(13,LOW); // Turn the LED off
  
  path_orbit_xy(pathA,{400,0,200},250);
  print_pathAngles(pathA);
}

void stateLoop(arm & the_arm) { // Call this Function at the top of main loop()
  // This is "setup" for each state
  // checks for a change in state and runs the setup for that state  
  static int old_state = -1;  // should force initialize first pass
  static int new_x, new_y, new_z;  
  
  jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector
  the_arm.state = jS.pot_value/100; // convert to an integer from 0 to 9

  if (the_arm.state != old_state){
    switch (the_arm.state) {
      case S_SERIAL: 
        the_arm.n = 0; // reset command pointer
        the_arm.rampDown = false;
        break;
      case S_AUTO_LINE_X:  // dance
        the_arm.n = 0; // reset command pointer
        the_arm.mode = M_CD_AIM;  // Aim at point that is on y=500 plane
        the_arm.rampDown = false;
        new_z = make3.current_pt.z;   // was target_pt... should be the same
        if (new_z < FLOOR)  new_z = FLOOR;
        if (new_z > 500)  new_z = 500;
        the_arm.aim_pt.x = make3.current_pt.x;  // current point y
        new_y = make3.current_pt.y;
        the_arm.aim_pt.y = new_y+500.0;
        the_arm.aim_pt.z = new_z;
        setCmd(sixCmds.cmd[0],K_LINE_C,300,the_arm.aim_pt.x-100,new_y,new_z);  // move to start
        setCmd(sixCmds.cmd[1],K_LINE_C,200,the_arm.aim_pt.x,new_y,new_z);
        setCmd(sixCmds.cmd[2],K_LINE_C,200,the_arm.aim_pt.x,new_y,new_z+200);
        setCmd(sixCmds.cmd[3],K_LINE_C,400,the_arm.aim_pt.x+100,new_y,new_z);
        setCmd(sixCmds.cmd[4],K_LINE_C,300,the_arm.aim_pt.x,new_y,new_z+150);
        setCmd(sixCmds.cmd[5],K_GOTO,0,0,0,0);  // loop back to start
        break;
      case S_AUTO_LINE_Y:
        the_arm.n = 0; // reset command pointer
        the_arm.mode = M_CD_AIM;  // Aim at point that is on y=0 plane
        the_arm.rampDown = false;
       new_x = make3.target_pt.x;
        if (new_x < 250) new_x = 250;  // can't be too close
        if (new_x > 400) new_x = 400;  // can't be too far
        new_z = make3.target_pt.z;
        if (new_z < FLOOR)  new_z = FLOOR;
        if (new_z > 500)  new_z = 500;
        the_arm.aim_pt.x = new_x + 300.0;
        the_arm.aim_pt.y = 0.0;
        the_arm.aim_pt.z = new_z;
        setCmd(sixCmds.cmd[0],K_LINE_C,160,new_x,LEN_AB+100,new_z);
        setCmd(sixCmds.cmd[1],K_TIMER,500,0,0,0);
        setCmd(sixCmds.cmd[2],K_LINE_C,100,new_x,-LEN_AB-100,new_z);
        setCmd(sixCmds.cmd[3],K_TIMER,500,0,0,0);
        setCmd(sixCmds.cmd[4],K_LINE_C,100,new_x,LEN_AB+100,new_z);
        setCmd(sixCmds.cmd[5],K_LINE_C,100,new_x,0,new_z);  // stop in the middle for alignment
        break;
      case S_AUTO_LINE_Z: // NEEDS REWRITE
        the_arm.n = 0; // reset command pointer
        the_arm.mode = M_C_LOC_D_ABS;
        the_arm.rampDown = false;
        setCmd(sixCmds.cmd[0],K_D_ABS,300,make3.jD.target_angle*RADIAN,0,0);
        setCmd(sixCmds.cmd[1],K_LINE_C,100,make3.target_pt.x,0,0);
        setCmd(sixCmds.cmd[2],K_TIMER,1000,0,0,0);
        setCmd(sixCmds.cmd[3],K_LINE_C,100,make3.target_pt.x,0,LEN_AB+LEN_BC-100);
        setCmd(sixCmds.cmd[4],K_TIMER,1000,0,0,0);
        setCmd(sixCmds.cmd[5],K_LINE_C,100,make3.target_pt.x,0,0);
        break;
      case S_AUTO_ARC_XY:
        the_arm.n = 0; // reset command pointer
        the_arm.mode = M_CD_AIM;  // Aim at center of circle
        the_arm.rampDown = false;
        new_x = 500;
        new_z = make3.target_pt.z;
        //if (new_z < FLOOR)  new_z = FLOOR;
        if (new_z > 400)  new_z = 400;
        the_arm.aim_pt.x = new_x+10;  // small offset to keep from flipping C joint
        the_arm.aim_pt.y = 0.0;
        the_arm.aim_pt.z = new_z;
        setCmd(sixCmds.cmd[0],K_LINE_C,80,new_x,260,new_z);
        setCmd(sixCmds.cmd[1],K_ARC_Z,80,0,0,0);
        setCmd(sixCmds.cmd[2],K_TIMER,500,0,0,0);
        setCmd(sixCmds.cmd[3],K_ARC_Z,120,1,0,0);
        setCmd(sixCmds.cmd[4],K_TIMER,500,0,0,0);
        setCmd(sixCmds.cmd[5],K_ARC_Z,140,0,0,0);
        break;
      case S_AUTO_CONES:
        the_arm.n = 0; // reset command pointer
        break;
      case S_TELEOP_1:
        the_arm.n = -1; // reset command pointer
        the_arm.mode = M_CD_ABS;
        the_arm.feedRate = 400.0;  // mm per second
        the_arm.rampDown = true;
        break;      
      case S_TELEOP_3:
        the_arm.n = -1; // reset command pointer
        the_arm.mode = M_CD_AIM;  // Aim at center of circle
        the_arm.aim_pt.x = make3.target_pt.z + 250;
        the_arm.aim_pt.y = 0.0;
        the_arm.aim_pt.z = make3.target_pt.z;
        the_arm.feedRate = 400.0;  // mm per second
        the_arm.rampDown = true;
        break;
      case S_TELEOP_4:
        the_arm.n = -1; // reset command pointer
        the_arm.mode = M_CD_ABS;
        the_arm.feedRate = 400.0;  // mm per second
        the_arm.rampDown = true;
        break;      
      case S_TELEOP_6:
        the_arm.n = -1; // reset command pointer
        the_arm.mode = M_CD_ABS;
        the_arm.feedRate = 400.0;  // mm per second
        the_arm.rampDown = true;
        break;      
      case S_TELEOP_8:
        the_arm.n = -1; // reset command pointer
        the_arm.mode = M_C_ABS_D_LOC;
        the_arm.feedRate = 400.0;  // mm per second
        the_arm.rampDown = true;
        break;      
      case S_TELEOP_9:
        the_arm.n = -1; // reset command pointer
        the_arm.mode = M_C_ABS_D_LOC;
        the_arm.feedRate = 400.0;  // mm per second
        the_arm.rampDown = true;
        break;      
      case S_TELEOP_10:
        the_arm.n = -1; // reset command pointer
        the_arm.mode = M_C_ABS_D_LOC;
        the_arm.feedRate = 400.0;  // mm per second
        the_arm.rampDown = true;
        break;      
    }
  }
  old_state = the_arm.state;
}

void updateJointBySpeed(joint & jt, unsigned long dt) {
  // NOTE dt is in milliseconds
  float radps;  // radians per millisecond
  radps = (jt.target_angle-jt.current_angle)/dt;  // full move speed
  if (radps > jt.target_velocity) {
    jt.current_angle += jt.target_velocity*dt;
  } else  if (radps < -jt.target_velocity) {
    jt.current_angle -= jt.target_velocity*dt;
  } else {
    jt.current_angle = jt.target_angle; // done
  }
}

void loopUpdateArm (arm & the_arm) { // Call this Function at the bottom of main loop()
  unsigned long mst;  // Keep track of Loop time.  Used for regulating servo speeds.
  mst = millis();
  the_arm.dt = mst - the_arm.prior_mst; // loop time, milliseconds
  the_arm.prior_mst = mst;
  if (the_arm.dt > 30) {   // Check that dt (delta time) is in reasonable range
    the_arm.dt = 30;
  } else if (the_arm.dt < 2) {
    the_arm.dt = 1;
  }
   
  // JOINTS A B T ARE COMPUTED BY KNOWING POINT C
  static float *angles;
  updateArmPtC(the_arm);   // Moves current_pt C toward target_pt at given feed rate (mmps)
  angles = inverse_arm_kinematics(the_arm.current_pt,LEN_AB,LEN_BC); 
  the_arm.jA.current_angle = angles[0]; // global A = local A
  the_arm.jB.current_angle = angles[1];  // local B
  the_arm.jT.current_angle = angles[2];  // global T

  // ARM MODES.  Solve for joints C and D
  static point ctemp = {0,-S_CG_Y,0};  // G CENTER (CAMERA) RELATIVE TO C, USED IN AIM
  point c1;    // USED IN AIM
  //float d1;    // USED IN AIM
  switch(the_arm.mode) {
    case M_CD_LOC:
      updateJointBySpeed(the_arm.jC, the_arm.dt);  
      updateJointBySpeed(the_arm.jD, the_arm.dt);  
    break;
    case M_CD_ABS:
      the_arm.jC.current_angle = getCang(the_arm.jA.current_angle,the_arm.jB.current_angle,the_arm.jC.target_angle);
      the_arm.jD.current_angle = the_arm.jT.current_angle - the_arm.jD.target_angle;     
    break;
    case M_C_LOC_D_ABS:
      updateJointBySpeed(the_arm.jC, the_arm.dt);  
      the_arm.jD.current_angle = the_arm.jT.current_angle - the_arm.jD.target_angle;     
    break;
    case M_C_ABS_D_LOC:
      the_arm.jC.current_angle = getCang(the_arm.jA.current_angle,the_arm.jB.current_angle,the_arm.jC.target_angle);
      updateJointBySpeed(the_arm.jD, the_arm.dt);  
    break;
    case M_CD_AIM: // computes C and D from current and aim points and uses angles as absolute 
      if (the_arm.current_pt.z < LEN_AB) the_arm.jC.target_angle = - 90.0/RADIAN; 
      else the_arm.jC.target_angle = 90.0/RADIAN;
      //  c1 point represents the camera offset from the c point
      c1 = rot_pt_z(ctemp,the_arm.jT.current_angle-90.0/RADIAN);  // ROTATE CAMERA/G POINT BY THE ARM/TURNTABLE ANGLE
      the_arm.jD.target_angle = atan2(the_arm.aim_pt.y-the_arm.current_pt.y-c1.y,the_arm.aim_pt.x-the_arm.current_pt.x-c1.x);
      the_arm.jC.current_angle = getCang(the_arm.jA.current_angle,the_arm.jB.current_angle,the_arm.jC.target_angle);
      the_arm.jD.current_angle = the_arm.jT.current_angle - the_arm.jD.target_angle;     
    break;
  }

  updateJointBySpeed(the_arm.jCLAW, the_arm.dt);  // update Claw joint
  
  // Blink the built in LED to measure the HeartBeat (speed) of the Loop
  // 3/9/3023 stopwatch measurements found loop speed ~ 75 hz (13.3 millisec per loop). Increase to 40 counts per blink to make easier to count
  static int counterLEDblink = 0;
  if (counterLEDblink < 20) {
    digitalWrite(13,HIGH); // Turn the LED on
  } else if (counterLEDblink < 40) { 
    digitalWrite(13,LOW); // Turn the LED off
  } else {
    counterLEDblink = 0; // reset counter
  }
  counterLEDblink ++;    

}

void loop() {  //########### MAIN LOOP ############
  static float mmps_scale;
  static command cmd = {0,0,0,0,0};  // used in Serial Read

  stateLoop(make3);  // checks for state change
  
  switch (make3.state) {
    case S_SERIAL: 
      if (runCommand(make3, cmd)) getCmdSerial(cmd);  // returns zeros if no command
      if (cmd.arg[0] != 0) make3.timerStart=millis();  // start timer if there is a command
      break;
    case S_AUTO_LINE_X:
    case S_AUTO_LINE_Y:
    case S_AUTO_LINE_Z:
    case S_AUTO_ARC_XY:
      readCommands(make3,    sixCmds , "sixCmds,"); // get and calculate the moves
      break;
    case S_AUTO_CONES:  
      //readCommands(make3,    coneCmds ,"coneCmds,"); // get and calculate the moves
      break;
    case S_TELEOP_1: 
    case S_TELEOP_3:
    case S_TELEOP_4: 
    case S_TELEOP_6: 
    case S_TELEOP_8:
    case S_TELEOP_9:
    case S_TELEOP_10:
      make3.jC.target_angle = -90.0/RADIAN;     
      //if (make3.state == S_TELEOP_3) make3.jC.target_angle = 90.0/RADIAN;
      if (make3.state == S_TELEOP_9) make3.jC.target_angle = -45.0/RADIAN;
      if (make3.state == S_TELEOP_10) make3.jC.target_angle = 0.0;
      
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
      make3.target_pt = anglesToC(make3.jA.target_angle,make3.jB.target_angle,make3.jT.target_angle,LEN_AB,LEN_BC);
      break;
  }

  // PRE UPDATE, FLOOR HARD LIMITS
  //float clawZ;
  //if (make3.jC.current_angle < 0.0 ) clawZ = -sin(make3.jC.current_angle)*S_CG_X;
  //else clawZ = 0.0;
  if (make3.current_pt.z < (FLOOR) ) {  // this keeps the arm from doing a pushup and frying a servo
    make3.current_pt.z = FLOOR;
  }
  
  // HARD LIMITS
  if (make3.jA.target_angle > 150.0/RADIAN) make3.jA.target_angle = 150.0/RADIAN; // keep arm from going into baby arm
  if (make3.jA.target_angle < 12.0/RADIAN) make3.jA.target_angle = 12.0/RADIAN; // A joint physical limit.  Needs design fix.  3/7/2023
  if (make3.jCLAW.target_angle > 18.0/RADIAN) make3.jCLAW.target_angle = 18.0/RADIAN;
  if (make3.jCLAW.target_angle < -49.0/RADIAN) make3.jCLAW.target_angle = -49.0/RADIAN;  // WAS -49?

  loopUpdateArm(make3);  // ARM UPDATE (MOVES CURRENT TOWARD TARGETS)

  //logData(make3.jC,'C');
  //logPoint(make3);

  // Convert CURRENT angle to PWM signal and send 
  pwm.writeMicroseconds(make3.jA.svo.digital_pin, servo_map(make3.jA)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jB.svo.digital_pin, servo_map(make3.jB)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jC.svo.digital_pin, servo_map(make3.jC)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jD.svo.digital_pin, servo_map(make3.jD)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jCLAW.svo.digital_pin, servo_map(make3.jCLAW)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jT.svo.digital_pin, servo_map(make3.jT)); // Adafruit servo library
} // END OF MAIN LOOP
