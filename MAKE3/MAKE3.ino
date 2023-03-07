/* ROBOT ARM CONTROL SOFTWARE FOR MAKE3 ROBOT ARM
 *  By, SrAmo, January 2023
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define RADIAN 57.2957795  // number of degrees in one radian
//#define RADIAN 180.0/PI // number of degrees in one radian
#define SIZE_CMD_ARRAY 5  // NUMBER OF VALUES PER COMMAND
/*
 *  COMMAND CODES: All codes are integers
 *  Feed rate is in mm per second.
 *  Angle rate is in Degrees per second.
 *  Angles given in Degrees.  Location in mm.
 */
// Synchronous commands (completes command before next command is sent)
#define K_TIMER 1    // timer {millisconds}
#define K_LINE_C 2   // line move to new C point   {feed rate, x,y,z}
//#define K_LINE_G 3   // line move to new G point {feed rate, x,y,z}
#define K_CIRCLE_Z 4  // moves in a circle in the Z plane, from current pt (x,y,z) around (x,0,z), radius = abs(y) {feed rate, CW-0 or CCW-1}
// Asynchronous commands (does not wait to be completed)
#define K_C_LOC 12    // C local move           {angle rate, target angle}
#define K_C_ABS 22    // C Absolute Angle Mode, {angle rate, target angle}
#define K_D_LOC 13    // D local move           {angle rate, target angle}
#define K_D_ABS 23     // D Absolute Angle Mode {angle rate, target angle}
#define K_CLAW_LOC 14   // claw local move      {angle rate, target angle}

// STATES
#define S_SERIAL 0
#define S_TELEOP_1 1
#define S_AUTO_LINE_Y 2 // C at 90 abs, D abs from last
#define S_AUTO_LINE_Z 3 // C at 90 abs, D loc from last
#define S_TELEOP_4 4
#define S_AUTO_LINE_X 5 // C at 90 abs, D loc from last
#define S_TELEOP_6 6
#define S_AUTO_CIRCLE_Z 7
#define S_AUTO_CONES 17  // WAS 7
#define S_TELEOP_8 8
#define S_TELEOP_9 9  // C at 45 absolute
#define S_TELEOP_10 10  // C at 0 absolute

// GEOMETRY OF ARM:
#define LEN_AB 350.0     // Length of Input AB arm in mm
#define LEN_BC 380.0     // Length of Input BC arm in mm
#define FLOOR -152.0     // Distance from A joint to Floor in mm
#define S_TA 0.0       // offset in X of A axis from Turtable Axis
#define S_CG_X 180.0    // offset from joint C to G in X
#define S_CG_Y 38.0     // offset from joint C to G in Y
#define S_CG_Z 40.0     // offset from joint C to G in Z

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
  // 180 deg SAVOX, gets 170 deg motion with 500 to 2800 microsecond range
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
  boolean global;  // used for special cases where joint moves based on formula
  float pot_angle; // Potentiometer arm angle, if used, in radians
  float current_angle; // last issued command for angl, in radians
  float target_angle;  // where the angle should end up, in radians
  float target_velocity;  // radians per millisecond
};

struct arm {  // as in Robot Arm
  int state; // The different ways that the arm can be controlled. Set by S potentiometer
  int n; // current active index in the command array
  unsigned long prior_mst; // clock time (microseconds) prior loop 
  unsigned long dt; // delta loop time in ms
  unsigned long timerStart; // timer, for delays
  float feedRate; // feed rate for point moves, in mm/sec (converted to /microsec)
  point current_pt; // current (C or G) point 
  point target_pt;  // target C or G point
  float line_len;  // calculated distance from current_pt to target_pt
  joint jA;
  joint jB;
  joint jC;
  joint jD;
  joint jCLAW;
  joint jT;
};

struct command {
  int arg[SIZE_CMD_ARRAY];
};

struct sequence {
  int num_cmds;
  command cmd[];  // undefined array length
};

//   GOBAL VARIABLES 
arm make3; 
joint jS;  // selector pot

sequence lineCmds = {6,{
   {K_TIMER,1000,0,0,0},    // pause
   {K_LINE_C,100, 160,150,150}, // point 1
   {K_TIMER,1000,0,0,0},    // pause
   {K_LINE_C,100, 300,150,150}, // point 2
   {K_TIMER,1000,0,0,0},    // pause
   {K_LINE_C,100, 160,150,150}, // point 1
   }}; 
                           
sequence coneCmds ={7,{
   {K_CLAW_LOC,100,CLAWOPEN,0,0},  // open the claw
   {K_LINE_C,MMPS, X_PP,Y_MV,FLOORH+CONEH*7}, // ready over cone 1
   {K_LINE_C,MMPS, X_PP,Y_MV,FLOORH+CONEH*5}, // down to cone 1
   {K_CLAW_LOC,500,CLAWCLOSE,0,0}, // pick cone 1
   {K_LINE_C,MMPS, X_PP,Y_MV,FLOORH+CONEH*9}, // up with cone 1 
   {K_LINE_C,MMPS, X_PP,Y_MV_NEG,MIDJUNTH}, // Move over Junction
   {K_CLAW_LOC,500,CLAWOPEN,0,0}  // drop cone 1
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
  // Note: can't read physical servo angle, so can jump on startup
  jt.global = false;
  jt.current_angle = initial_angle;
  jt.target_angle = initial_angle;
  jt.target_velocity = 60.0/RADIAN/1000.0;  // default initial velocity, radians per microsecond
}

void initArm(arm & the_arm, point pt) { // starting point for C or G on arm
  // initialize arm structure
  the_arm.state = 0;
  the_arm.n = 0;  // first command
  the_arm.prior_mst = millis();
  the_arm.dt = 10;  //  TO BE TUNED?
  the_arm.timerStart = millis();
  the_arm.feedRate = 300.0;  // TO BE TUNED?
  the_arm.current_pt = pt;
  the_arm.target_pt = pt;
  the_arm.line_len = 0.0;
}

float getCang(arm the_arm, float fixed_angle) { // returns joint angle C, using A and B
  // Assumes that you want an absolute (fixed) C angle
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
  // Given arm system GroundT-TA-AB-BC, where T (turntable) and A are [0,0,0]
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
/*   NOT USED FOR NOW
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
}  */
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
/*  NOT USED FOR NOW
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
} */
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
/* NOT USED FOR NOW
boolean updateArmPtG(arm & the_arm, float to_C, float to_D) { 
  // Moves current point G toward target_pt at given feed rate (mmps)
  // updates .current_pt based on speed limit
  // RETURNS true if done moving, false if not
  float moveDist;
   
  moveDist = the_arm.feedRate * (the_arm.dt/1000.0); // feed rate (mm/sec)*sec = mm  
  the_arm.line_len = ptpt_dist(the_arm.current_pt,the_arm.target_pt);
  if (moveDist < the_arm.line_len) { // PARTIAL MOVE
    the_arm.current_pt = pt_on_line(moveDist,the_arm.line_len, the_arm.current_pt,the_arm.target_pt);
    Serial.print("PARTIAL MOVE PT G (mm),");
    Serial.println(moveDist,1);
 
    the_arm.jC.current_angle = interpolate(moveDist,the_arm.line_len, the_arm.jC.target_angle,to_C);
 
    the_arm.jD.current_angle = interpolate(moveDist,the_arm.line_len, the_arm.jD.target_angle,to_D);
    return false;
  } else { // FULL MOVE
    the_arm.current_pt = the_arm.target_pt;
    the_arm.jC.current_angle = to_C;
    the_arm.jD.current_angle = to_D;
    return true;
  }
} */

void updateArmPtC(arm & the_arm) { 
  // Moves current point C toward target_pt at given feed rate (mmps)
  // updates .current_pt based on speed limit
  float moveDist;
  
  moveDist = the_arm.feedRate/1000.0 * (the_arm.dt); // feed rate (mm/sec)*sec = mm  
  the_arm.line_len = ptpt_dist(the_arm.current_pt,the_arm.target_pt);
  if (moveDist < the_arm.line_len) { // PARTIAL MOVE
    the_arm.current_pt = pt_on_line(moveDist,the_arm.line_len, the_arm.current_pt,the_arm.target_pt);
    //Serial.print("PARTIAL MOVE PT C (mm),");
    //Serial.print(moveDist,1);
    //Serial.print(",   ");
    //logPoint(the_arm);
   } else { // FULL MOVE
    the_arm.current_pt = the_arm.target_pt;
    //logPoint(the_arm);
  }
}
void logPoint(arm & the_arm) {
  Serial.print("CURRENT PT,");
  Serial.print(the_arm.current_pt.x);
  Serial.print(",");
  Serial.print(the_arm.current_pt.y);
  Serial.print(",");
  Serial.print(the_arm.current_pt.z);
  Serial.print(",   TARGET PT,");
  Serial.print(the_arm.target_pt.x);
  Serial.print(",");
  Serial.print(the_arm.target_pt.y);
  Serial.print(",");
  Serial.println(the_arm.target_pt.z);
}
 
void logData(joint jt,char jt_letter) {
  Serial.print(make3.state);
  Serial.print(",");
  Serial.print(jt_letter);
  Serial.print(", p_value,");
  Serial.print(jt.pot_value);
  Serial.print(", Pang,");
  Serial.print(jt.pot_angle*RADIAN,1);
//  Serial.print(",");
  Serial.print(",current_ang (DEG),");
  Serial.print(jt.current_angle*RADIAN,1);
  Serial.print(",target_ang (DEG),");
  Serial.print(jt.target_angle*RADIAN,1);

  Serial.print(",Servo (microsec),");
  Serial.print(servo_map(jt));

  //Serial.print(",target_vel (DEG/SEC),");
  //Serial.print(jt.target_velocity*RADIAN*1000.0,1);
  Serial.println(" . ");
} 

boolean runCommand(arm & the_arm, command  & the_cmd) { 
  // TRUE if DONE, FALSE is NOT DONE

  // read type of command
  switch (the_cmd.arg[0]) {
    case 0:
      return true;
      break;
    case K_TIMER: // DELAY (timer)
      if ((millis()-the_arm.timerStart) > the_cmd.arg[1]) {
        Serial.print("TIMER DONE, ");
        Serial.println((millis()-the_arm.timerStart));
        return true;
      } else
        return false;
      break;
    case K_C_LOC:  // C servo
    case K_C_ABS:  // move C servo relative to ground (absolute)
      // second argument is NEW VELO FOR C, DEG per Second
      if (the_cmd.arg[1] < 10) the_cmd.arg[1] = 10; // NO Negatives or Zeros
      the_arm.jC.target_velocity = the_cmd.arg[1]/RADIAN/1000.0;
      the_arm.jC.target_angle = the_cmd.arg[2]/RADIAN;  // Third arg is NEW ANGLE FOR C

      the_arm.jC.global = false;
      if (the_cmd.arg[0] == K_C_ABS) the_arm.jC.global = true;

      //logData(the_arm.jC,'C');
      return true;
      break;
    case K_D_LOC:  // D servo
    case K_D_ABS:  // move D servo relative to ground (absolute)
      // second argument is NEW VELO FOR D, DEG per Second
      if (the_cmd.arg[1] < 10) the_cmd.arg[1] = 10; // NO Negatives or Zeros
      the_arm.jD.target_velocity = the_cmd.arg[1]/RADIAN/1000.0;
      the_arm.jD.target_angle = the_cmd.arg[2]/RADIAN;  // Third arg is NEW ANGLE FOR D

      the_arm.jD.global = false;
      if (the_cmd.arg[0] == K_D_ABS) the_arm.jD.global = true;
      
      //logData(the_arm.jD,'D');
      return true;
      break;
    case K_CLAW_LOC:  // CLAW servo
      // second argument is NEW VELO FOR C, DEG per Second
      if (the_cmd.arg[1] < 10) the_cmd.arg[1] = 10; // NO Negatives or Zeros
      the_arm.jCLAW.target_velocity = the_cmd.arg[1]/RADIAN/1000.0;
      the_arm.jCLAW.target_angle = the_cmd.arg[2]/RADIAN;  // Third arg is NEW ANGLE FOR Claw
      the_arm.jCLAW.global = false;
      //logData(the_arm.jCLAW,'X');
      return true;
      break;
    case K_LINE_C: // line move to C point
    //case K_LINE_G: // line move to G point (TO BE COMPLETED-SAME A C FOR NOW)
      the_arm.feedRate = the_cmd.arg[1];
      the_arm.target_pt.x = the_cmd.arg[2];
      the_arm.target_pt.y = the_cmd.arg[3];
      the_arm.target_pt.z = the_cmd.arg[4];
      the_arm.line_len = ptpt_dist(the_arm.current_pt,the_arm.target_pt); // also calculated in loop update arm... shouldn't do it twice
      
      if (the_arm.line_len < 0.5) {  // Distance from current to target is small
        Serial.println("LINE MOVE DONE, ");
        return true;
      } else { // return false to keep the next command from running
        //Serial.println("LINE MOVE ");
        return false;
      }
      break;
    case K_CIRCLE_Z:  // move in circle on z plane
      static int rad;
      static point center = {100,0,100};
      static float current_angle; // radians
      static float angle_inc = 0.1570796;  // radians (40 secments per circle)
      static int angle_sign = 1;
      static boolean circleInitialize = true;
      float dist;
      // first time reading the arc command, set variables
      if (circleInitialize) {  // setup circle move
        Serial.println("Circle Initialize");
        the_arm.feedRate = the_cmd.arg[1];  // feed rate in mm/sec
        rad = abs(the_arm.target_pt.y); 
        if (rad < 10) rad = 10;  // mm
        // store circle center, only x in needed
        center.x = the_arm.target_pt.x;
        if (the_cmd.arg[2] == 1) angle_sign = -1; // reverse direction if requested
        else angle_sign = 1;

        current_angle = 0.0; 
        the_arm.target_pt.x = center.x;
        the_arm.target_pt.y = center.y + rad;
        circleInitialize = false;  // don't execute setup again
      }
      dist = ptpt_dist(the_arm.current_pt,the_arm.target_pt);
      if (abs(current_angle) > 2.0*PI+angle_inc) {
        circleInitialize = true;
        current_angle = 0.0; 
        return true;  // done with command
      } else if (dist < 0.2) {  // move to next segment of circle
        the_arm.target_pt.x = center.x + rad*sin(current_angle);
        if (the_arm.target_pt.x < 10.0) the_arm.target_pt.x = 10.0; // Clip -x! keep the turntable from fast reversing
        the_arm.target_pt.y = center.y + rad*cos(current_angle);
        current_angle = current_angle + angle_inc*angle_sign; 
        return false; 
      }  else { // let the Arm Loop Update
        return false;  // stay on current segment of circle      
      }

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
    //for (int i=0;i<SIZE_CMD_ARRAY;i++) {
    //  cmdArray[i]=the_cmds[the_arm.n][i];//if (newCmd) {  // command initialization
    //}
    if (runCommand(the_arm, cmdArray)) { // true = go to the next command
      Serial.print("JUST RAN ");
      Serial.print(string);
      Serial.println(the_arm.n);
      the_arm.n += 1; // go to the next command line
      the_arm.line_len = 0.0; // this tells arcs that its the start of an arc
      the_arm.timerStart = millis();  // for timed commands
      //newCmd = true;    
    } else { // false = keep looping on the current command
      //newCmd = false;
      };
  };
}

void logCmds(sequence & the_cmds, char *string) {
  Serial.println("logCmds");
  int i;
  for (int i=0;i<the_cmds.num_cmds;i++) {
    Serial.print(string);
    Serial.print(i); 
    for (int j=0;j<SIZE_CMD_ARRAY;j++) {
      Serial.print(",");
      Serial.print(the_cmds.cmd[i].arg[j]); 
    }
    Serial.println("."); 
  }
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

    for (i=1;i<SIZE_CMD_ARRAY;i++) 
      cmd.arg[i] = 0;  // initialize

    i=0;
    while (Serial.available() > 0) {
      cmd.arg[i] = Serial.parseInt(); // get available ints
      i++;
    }    
    // print the command
    Serial.print("Command: ");
    for (i=0;i<SIZE_CMD_ARRAY;i++) {
      Serial.print(cmd.arg[i]);
      Serial.print(",");
    }
    Serial.println("END");
  }
  //return(cmd); 
}

void setup() {  // setup code here, to run once:

  Serial.begin(115200); // baud rate
  delay(1500);  // serial output needs a delay

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
  make3.jD.pot = initPot(2 ,250, 60/RADIAN, 747, -60/RADIAN); 
  make3.jT.pot = initPot(4 ,348, 45/RADIAN, 797, -45.0/RADIAN); // set 3/7/2023
  make3.jCLAW.pot = initPot(3 , 250,-50/RADIAN, 750, 50/RADIAN);  // input arm is limited to 250 to 750
  jS.pot = initPot(5 , 0, 0/RADIAN, 1023, 280/RADIAN);  // to tune

  // TUNE SERVO LOW AND HIGH VALUES
  // initServo(pin,lowang,lowms,highang,highms)
  make3.jA.svo = initServo(0,  45.0/RADIAN, 990, 135.0/RADIAN, 2010); // set 3/7/2023
  make3.jB.svo = initServo(1, 0.0/RADIAN, 500,-175.0/RADIAN, 2320); // good
  make3.jC.svo = initServo(2, -90.0/RADIAN, 927, 0.0/RADIAN, 1410); // set 3/7/2023
  make3.jD.svo = initServo(3,  -90.0/RADIAN,  811, 90.0/RADIAN, 2054); // good
  make3.jCLAW.svo = initServo(4, -50.0/RADIAN,  900, 50.0/RADIAN, 1900); // 45 DEG = FULL OPEN, -45 = FULL CLOSE
  make3.jT.svo = initServo(5,  -45.0/RADIAN,  2038, 45.0/RADIAN, 610); // set 3/7/2023

  // INITIALIZE ANGLES FOR ARM
  initJoint(make3.jA, 130.0/RADIAN);  
  initJoint(make3.jB,  -130.0/RADIAN);
  initJoint(make3.jC,  -70.0/RADIAN);
  initJoint(make3.jD,   0.0/RADIAN);
  initJoint(make3.jT,    -10.0/RADIAN); 
  initJoint(make3.jCLAW,  17.0/RADIAN); 

  initArm(make3,{LEN_BC/2, 0.0, LEN_AB});
  make3.state = 1; 

  // Echo sequences
  logCmds(lineCmds, "lineCmds "); 
  logCmds(coneCmds, "coneCmds ");
}

void stateLoop(arm & the_arm) {
  // This is "setup" for each state
  // called at the begining of loop()
  // checks for a change in state and runs the setup for that state  
  static int old_state = -1;  // should force initialize first pass
  // jS (joint Selector) is a global variable
  
  jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector
  the_arm.state = jS.pot_value/100; // convert to an integer from 0 to 9
  
  if (the_arm.state != old_state){
    the_arm.n = 0; // reset array pointer
    Serial.print("NEW STATE,");
    Serial.print(the_arm.state);
    switch (the_arm.state) {
      case S_SERIAL: 
        Serial.println(", Serial Read");
        break;
      case S_AUTO_LINE_X:
        Serial.println(", AUTO LINE X");
        make3.jD.global = false;
        setCmd(lineCmds.cmd[1],K_LINE_C,100,100,0,make3.target_pt.z);
        setCmd(lineCmds.cmd[3],K_LINE_C,100,LEN_AB+LEN_BC-50,0,make3.target_pt.z);
        setCmd(lineCmds.cmd[5],K_LINE_C,100,100,0,make3.target_pt.z);
         logCmds(lineCmds, "lineCmds "); 
        break;
      case S_AUTO_LINE_Y:
        Serial.println(", AUTO LINE Y");
        make3.jD.global = true;
        setCmd(lineCmds.cmd[1],K_LINE_C,100,make3.target_pt.x,300,make3.target_pt.z);
        setCmd(lineCmds.cmd[3],K_LINE_C,100,make3.target_pt.x,-300,make3.target_pt.z);
        setCmd(lineCmds.cmd[5],K_LINE_C,100,make3.target_pt.x,300,make3.target_pt.z);
        logCmds(lineCmds, "lineCmds "); 
        break;
      case S_AUTO_LINE_Z:
        Serial.println(", AUTO LINE Z");
        make3.jD.global = false;
        setCmd(lineCmds.cmd[1],K_LINE_C,100,make3.target_pt.x,0,0);
        setCmd(lineCmds.cmd[3],K_LINE_C,100,make3.target_pt.x,0,500);
        setCmd(lineCmds.cmd[5],K_LINE_C,100,make3.target_pt.x,0,0);
        logCmds(lineCmds, "lineCmds "); 
        break;
      case S_AUTO_CIRCLE_Z:
        Serial.println(", AUTO CIRCLE Z");
        make3.jD.global = false;
        setCmd(lineCmds.cmd[1],K_CIRCLE_Z,200,0,0,0);
        setCmd(lineCmds.cmd[2],K_D_ABS,100,0,0,0);
        setCmd(lineCmds.cmd[3],K_CIRCLE_Z,300,1,0,0);
        setCmd(lineCmds.cmd[4],K_D_LOC,100,0,0,0);
        setCmd(lineCmds.cmd[5],K_CIRCLE_Z,400,0,0,0);
        logCmds(lineCmds, "lineCmds "); 
        break;
        
      case S_AUTO_CONES:
        Serial.println(", AUTO CONES");
        break;
      case S_TELEOP_1:
      case S_TELEOP_4:
      case S_TELEOP_6:
      case S_TELEOP_8:
      case S_TELEOP_9
    :
      case S_TELEOP_10
    :
        make3.jC.global = true;
        make3.feedRate = 400.0;  // mm per second
        Serial.println(", TELEOP");
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
    //Serial.print(jt.svo.digital_pin);
    //logData(jt,'+');
  } else  if (radps < -jt.target_velocity) {
    jt.current_angle -= jt.target_velocity*dt;
    //Serial.print(jt.svo.digital_pin);
    //logData(jt,'-');
  } else {
    jt.current_angle = jt.target_angle; // done
    //logData(jt,'D');
  }
}

void loopUpdateArm (arm & the_arm) {
  // Loop time.  Used for speeds.
  unsigned long mst;
  mst = millis();
  the_arm.dt = mst - the_arm.prior_mst; // loop time, milliseconds
  the_arm.prior_mst = mst;
  // Keep dt (delta time) in reasonable range
  if (the_arm.dt > 30) {
    the_arm.dt = 10;
    Serial.println(">30 DT");
  } else if (the_arm.dt < 2) {
    the_arm.dt = 1;
    Serial.println("<2 DT");    
  }
   
  // NEED TO IMPLEMENT C VS G
  // JOINTS A B T ARE DRIVEN BY POINT C
  static float *angles;
  updateArmPtC(the_arm);
  angles = inverse_arm_kinematics(the_arm.current_pt,LEN_AB,LEN_BC,S_TA); 
  the_arm.jA.current_angle = angles[0]; // global A = local A
  the_arm.jB.current_angle = angles[1];  // local B
  the_arm.jT.current_angle = angles[2];  // global T

  // JOINT C CAN BE GLOBAL OR LOCAL
  if (the_arm.jC.global) {
    the_arm.jC.current_angle = getCang(the_arm,the_arm.jC.target_angle);
  } else {
    updateJointBySpeed(the_arm.jC, the_arm.dt);  
  }

  // JOINT D CAN BE GLOBAL OR LOCAL  OR AIM AT A POINT
  // TARGET POINT CODE:
  // assume that the aim point is on the y=0 line, and arm is moving in an xy plane
  // aim point is given as an x value
  // use atan2(y,x) to four quadrants of angle
  // first transform the aim point so that the current point is (0,0)
  // newx = aim.x-cp.x
  // newy = aim.y-cp.y
  // theta = atan2(newy,newx)
  // the_arm.jD.current_angle = the_arm.jT.current_angle - theta;
  if (the_arm.jD.global) {
    the_arm.jD.current_angle = the_arm.jT.current_angle - the_arm.jD.target_angle;     
  } else {
    updateJointBySpeed(the_arm.jD, the_arm.dt);  
  }

  updateJointBySpeed(the_arm.jCLAW, the_arm.dt);  // update Claw joint
}


void loop() {  //########### MAIN LOOP ############
  static float mmps_scale;
  static command cmd;  // used in Serial Read

  stateLoop(make3);  // checks for state change
  
  switch (make3.state) {
    case S_SERIAL: 
      getCmdSerial(cmd);  // returns zeros if no command
      if (cmd.arg[0] != 0) make3.timerStart=millis();  // start timer if there is a command
      while (!runCommand(make3, cmd)) {
        //Serial.println("NOT TRUE");  // for timer (HALTS LOOP!)
      }
      break;
    case S_AUTO_LINE_X:
    case S_AUTO_LINE_Y:
    case S_AUTO_LINE_Z:
    case S_AUTO_CIRCLE_Z:
      readCommands(make3,    lineCmds, "lineCmds,"); // get and calculate the moves
      break;
    case S_AUTO_CONES:  
      readCommands(make3,    coneCmds ,"coneCmds,"); // get and calculate the moves
      break;
    case S_TELEOP_1: 
    case S_TELEOP_4: 
    case S_TELEOP_6: 
    case S_TELEOP_8:
    case S_TELEOP_9
  :
    case S_TELEOP_10
  :
      make3.jC.target_angle = -90.0/RADIAN;     
      if (make3.state == S_TELEOP_9
    ) make3.jC.target_angle = -45.0/RADIAN;
      if (make3.state == S_TELEOP_10
    ) make3.jC.target_angle = 0.0;
      
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
  float clawZ;
  if (make3.jC.current_angle < 0.0 ) clawZ = -sin(make3.jC.current_angle)*S_CG_X;
  else clawZ = 0.0;
  if (make3.current_pt.z < (FLOOR+clawZ) ) {  // this keeps the arm from doing a pushup and frying a servo
    //Serial.print("FLOOR (mm),");
    //Serial.println(make3.current_pt.z,1);
    make3.current_pt.z = FLOOR+clawZ;
  }

  loopUpdateArm(make3);  // ARM UPDATE (MOVES CURRENT TOWARD TARGETS)
  
  // HARD LIMITS
  if (make3.jA.current_angle > 150.0/RADIAN) make3.jA.current_angle = 150.0/RADIAN; // keep arm from going into baby arm
  if (make3.jA.current_angle < 12.0/RADIAN) make3.jA.current_angle = 12.0/RADIAN; // A joint physical limit.  Needs design fix.  3/7/2023
  if (make3.jCLAW.current_angle > 18.0/RADIAN) make3.jCLAW.current_angle = 18.0/RADIAN;
  if (make3.jCLAW.current_angle < -49.0/RADIAN) make3.jCLAW.current_angle = -49.0/RADIAN;  // WAS -49?

  logData(make3.jT,'T');

  // Convert CURRENT angle to PWM signal and send 
  pwm.writeMicroseconds(make3.jA.svo.digital_pin, servo_map(make3.jA)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jB.svo.digital_pin, servo_map(make3.jB)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jC.svo.digital_pin, servo_map(make3.jC)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jD.svo.digital_pin, servo_map(make3.jD)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jCLAW.svo.digital_pin, servo_map(make3.jCLAW)); // Adafruit servo library
  pwm.writeMicroseconds(make3.jT.svo.digital_pin, servo_map(make3.jT)); // Adafruit servo library
} // END OF MAIN LOOP
