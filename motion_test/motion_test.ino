/*
 * Robot Arm Motion Control Include file
 * By SrAmo,  July 2022
 * 
 *  The bottom of this sketch (setup and loop functions) should be
 *  commented-out when saves as an include file.
 */
#define RADIAN 57.2957795  // number of degrees in one radian

struct point {float x,y,z;}; 

struct line {struct point p1 , p2; };

struct machine_state {
  int state; // 0=TBD,1=code_array,2=manual: CHANGED BY S POTENTIOMETER
  boolean initialize; // =true if one needs to initialize state
  int n; // current active index in the command array
  int cmd_size; // size of the command array
  unsigned long prior_mst; // clock time (microseconds) prior loop 
  unsigned long dt; // delta loop time in ms
  unsigned long timerStart; // timer, for delays
  float dist; // distance traveled in a move
  point at_ptC; // current C point 
  point at_ptD; // current D point 
  float angClaw;
};

#define SIZE_CMD_ARRAY 8  // NUMBER OF VALUES PER COMMAND
/*
 *  CODES:
 *  0 = DELAY, followed by the delay in milliseconds
 *  1 = LINE MOVE, followed by feed rate, then C go-to point, F go-to point
 *  2 = ARC MOVE, followed by feed rate, then go-to pointS
 *  4 = Claw Move, followed by feed rate, then claw position
 *  
 *  Feed rate is in mm per second.
 *  
 *  array     = {{code,feed rate/delay, Cx-to, Cy-to, Cz-to, Fx-to, Fy-to, Fz-to},{},{}};
 *  loc in array ={  0,   1           , 2, 3, 4, 5   ,  6  ,  7  } 
 */
/*  Example of a command array and getting the size
static int cmd_array[][SIZE_CMD_ARRAY]={{1,100,300,0,250,0,0,0},
                           {0,3000,0,0,0,0,0,0},
                           {1,100,200,-200,250,0,0,0},
                           {1,50,300,300,200,0,0,0}};
 */
 /*
 *  STRUCTURES FOR SERVOS, POTENTIOMETERS, AND JOINTS
 */
struct potentiometer {
  int analog_pin; // Arduino analog pin number (0 - 5)
  int low_mv; // low voltage point, from 0 (0 Volts) to 1023 (5 Volts)
  float low_ang; // corresponding angle at low voltage, in radians
  int high_mv; // high voltage point, from 0 (0 Volts) to 1023 (5 Volts)
  float high_ang; // corresponding angle at high voltage, in radians
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
  float low_ang; // corresponding angle at low microsecond, in radians
  int high_ms; // high microsecond point, from about 500 to 2400
  float high_ang; // corresponding angle at high microsecond, in radians
}; 

struct joint {
  potentiometer pot; // Sub structure which contains static pot information
  arm_servo svo; // Sub structure which contains the static servo information
  int pot_value;  // current potentiometer value in millivolts 
  float pot_angle; // Potentiometer arm angle, if used, in radians
  unsigned long previous_millis; // used to find servo rate of change
  float desired_angle;  // angle from input device or array, in radians
  int servo_ms;   // servo command in microseconds
  //float previous_velo;  // previous velocity, used to find acceleration
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
  // Converts initial_angle to Servo Microseconds
  jt.pot_value = 500;  // middle ish
  jt.desired_angle = initial_angle;
  jt.servo_ms = map(initial_angle,jt.svo.low_ang,jt.svo.high_ang,jt.svo.low_ms,jt.svo.high_ms);
  jt.previous_millis = millis();
}

machine_state setup_ms(float xC, float yC, float zC, int cmd_size) { // starting point C on arm
  struct machine_state ms;
  ms.state = 0;
  ms.initialize = true;
  ms.n = 0;  // first command
  ms.cmd_size = cmd_size;
  ms.prior_mst = millis();
  ms.at_ptC.x = xC;  ms.at_ptC.y = yC;  ms.at_ptC.z = zC;
  ms.at_ptD.x = xC+1000.0;  ms.at_ptD.y = yC;  ms.at_ptD.z = zC;
  ms.angClaw = 45;  // full open
  return ms;
}

//  ###############  MATH FUNCTIONS ###########################
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
float * inverse_arm_kinematics(point c, float l_ab, float l_bc, point d) {
  // Given arm system GroundT-TA-AB-BC-CD, where T (turntable) and A are [0,0,0]
  // The location of joint C (c[])  AND CX IS POSITIVE ONLY
  // The location of D (focal point) (where the claw should point)
  // The lengths Length AB (l_ab) and Length BC (l_bc) are specified
  // The joints A,B are on a turntable with rotation T parallel to Z through A
  // With T_angle = 0, then joints A & B are parallel to the Y axis
  // calculate the angles given pt C ***Inverse Kinematics***
  // returns an array with [A_angle,B_angle,T_angle,D_angle] in radians
  
  float xy_len, c_len, sub_angle1, sub_angle2;
  static float angles[4] = {0.0,0.0,0.0,0.0};  // [ A , B , T , D]
  static point c_new;
  
  if (c.x > 0) { //The math only works for positive Cx
    angles[2] = atan2(c.y,c.x);  // note: atan2 order; y,x

    // compute the D angle based on where C and D are on the xy plane
    xy_len = sqrt(pow((d.x-c.x),2.0)+pow((d.y-c.y),2.0)); 
    if (xy_len > 0) {  
      angles[3] = atan2((d.y-c.y),(d.x-c.x)) - angles[2];  // CD angle minus the turntable angle
    } else {
      angles[3] = 0.0; 
    }
  
    c_new = rot_pt_z(c,-angles[2]); // rotate the point c onto the XZ plane using turntable angle
    c_len = sqrt(pow(c_new.x,2)+pow(c_new.z,2));   // XZ plane
  
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
  }
  return angles;  // return the angles
}
line forward_arm_kinematics(float a, float b, float d, float t, float l_ab, float l_bc, float l_cd){
  // Start at the D end of the arm, and apply translations and rotations to get the final points
  //   assume that angle c is the claw, and is not relavent here
  //   assume that angle d (wrist) rotates part cd about the Z axis through c
  static line lineCD;  // [Cx, Cy, Cz,  Dx, Dy, Dz]
  static point pB,pC,pD;
  static point pC2, pC3, pD2, pD3;  // temporary points for calculations 
  
  pD.x = l_cd;   pD.y = 0.0;  pD.z = 0.0;
  pC.x = l_bc;   pC.y = 0.0;  pC.z = 0.0;
  pB.x = l_ab;   pB.y = 0.0;  pB.z = 0.0;
  
  pC2 = rot_pt_y(pC,b);
  pC3 = add_pts(pB,pC2);
  pC2 = rot_pt_y(pC3,a); // reuse pC2
  pC3 = rot_pt_z(pC2,t);  // reuse pC3

  pD2 = rot_pt_z(pD,d);
  pD3 = add_pts(pC3,pD2);

  lineCD.p1 = pC3;   lineCD.p2 = pD3;
  return lineCD;
}

void pot_map(joint & jt) {
  // Map a potentiometer value in millivolts to an angle
  // map(value, fromLow, fromHigh, toLow, toHigh), USES INTEGER MATH
  // NOTE: SCALE ANGLES (RADIANS) UP THEN DOWN TO GET PRECISION FROM POT VALUES
  jt.pot_angle = map(jt.pot_value, jt.pot.low_mv, jt.pot.high_mv, jt.pot.low_ang*1000, jt.pot.high_ang*1000) / 1000.0; 
  
  jt.desired_angle = jt.pot_angle;  // assume that the two are equal for now, RADIANS
}

void servo_map(joint & jt) {
  // Map an arm angle to servo microseconds
  // PLAIN, No Rate Limiting
  // Map joint angle to the servo microsecond value
  jt.servo_ms = floatMap(jt.desired_angle, jt.svo.low_ang, jt.svo.high_ang, jt.svo.low_ms, jt.svo.high_ms);
}

void go_to_next_cmd(machine_state & machine) {
  machine.n += 1; // go to the next command line
  machine.dist = 0.0;
  machine.timerStart = millis();
  if (machine.n >= machine.cmd_size) { // end of command lines, stop
    machine.n = 9999; // signals end of commands
  }
}

void commands_loop(machine_state & machine, int cmds[][SIZE_CMD_ARRAY]) { 
  static float ds, line_len;
  static point toCpt,fromCpt, toDpt, fromDpt; 

  // read type of command
  if (machine.n != 9999) {
    switch (cmds[machine.n][0]) {
      case 0: // DELAY and C joint move
       machine.angClaw = cmds[machine.n][2]/RADIAN;  // set the claw angle
       if ((millis()-machine.timerStart) > cmds[machine.n][1]) {
          go_to_next_cmd(machine);        
        }
        break;
      case 1: // line move
        ds = cmds[machine.n][1] * (machine.dt/1000.0); // feed rate (mm/sec)*sec = mm
        if (machine.dist == 0.0) { // initialize line
          toCpt.x = cmds[machine.n][2];
          toCpt.y = cmds[machine.n][3];
          toCpt.z = cmds[machine.n][4];
          toDpt.x = cmds[machine.n][5];
          toDpt.y = cmds[machine.n][6];
          toDpt.z = cmds[machine.n][7];
          line_len = ptpt_dist(machine.at_ptC,toCpt);
          fromCpt = machine.at_ptC;
          fromDpt = machine.at_ptD;
          machine.dist += ds;
        } else {  // moving
            if (machine.dist <= line_len) {  // move along line 
              machine.at_ptC = pt_on_line(machine.dist,line_len, fromCpt,toCpt);
              machine.at_ptD = pt_on_line(machine.dist,line_len, fromDpt,toDpt);
              machine.dist += ds;
            } else {  // reached end point
              machine.at_ptC = toCpt;
              machine.at_ptD = toDpt;
              go_to_next_cmd(machine);
            }
         }
         break;
       case 2: // TBD
         break;
    } // end switch   
  }
}

void inputArmLoop(machine_state & machine, line to_CD, int mmps) { // limits movement given feed rate
  static float line_len;
  static point partC, partD;
  machine.dist = mmps * (machine.dt/1000.0); // feed rate (mm/sec)*sec = mm
  line_len = ptpt_dist(machine.at_ptC,to_CD.p1);
  if (machine.dist < line_len) { // PARTIAL MOVE
    partC = pt_on_line(machine.dist,line_len, machine.at_ptC,to_CD.p1);
    partD = pt_on_line(machine.dist,line_len, machine.at_ptD,to_CD.p2);
    machine.at_ptC = partC;
    machine.at_ptD = partD;
  } else { // FULL MOVE
    machine.at_ptC = to_CD.p1;
    machine.at_ptD = to_CD.p2;
  }
}

void state_setup(machine_state & machine) {
  // This is called every loop, to check for a state change
  if (machine.initialize) { // initialize
    machine.n = 0; // array pointer
    machine.dist = 0.0;  // could use a static variable?
    machine.initialize = false;
  }  
}
// #####################################################################################33
//  COMMENT OUT BELOW HERE WHEN SAVING AS AN INCLUDE (.h) FILE
//
// todo: pass paramaters to turn things on and off
void logData(joint jt,char jt_letter) {
  Serial.print(",");
  Serial.print(jt_letter);
//  Serial.print(", p_value,");
//  Serial.print(jt.pot_value);
//  Serial.print(", Pang,");
//  Serial.print(jt.pot_angle*RADIAN,1);
  Serial.print(",dsr_ang,");
  Serial.print(jt.desired_angle*RADIAN,1);
  Serial.print(", servo_ms,");
  Serial.print(jt.servo_ms);
} 

#define LEN_AB 320.0     // Length of Input AB arm in mm
#define LEN_BC 320.0     // Length of Input BC arm in mm
#define LEN_CD 140.0

struct machine_state test_machine; 
struct joint jA,jB,jC,jD,jT,jS;

#define MMPS 200 // mm per second
#define X_PP 280 // x mm for pick and place
#define Y_MV 200 // y swing in mm
#define FLOORH -80 // z of floor for picking
#define BLOCKH 100 // block height mm

static int cmd_array[][SIZE_CMD_ARRAY]={{1,MMPS, X_PP,Y_MV,FLOORH+BLOCKH,  X_PP,1000,0}, // ready block 1
                           {0,2000,45,0,0,0,0,0}, // pause to pick block 1 - UNIQUE IN SEQUENCE
                           {1,MMPS, X_PP,Y_MV,FLOORH,             X_PP,1000,0}, // down to block 2
                           {0,500,-45,0,0,0,0,0}, // pick block 1
                           {1,MMPS, X_PP,Y_MV,FLOORH+2*BLOCKH,    1000, Y_MV,0}, // up block 1 
                           {1,MMPS, X_PP,-Y_MV,FLOORH+2*BLOCKH,   1000, -Y_MV,0}, // over block 1 
                           {1,MMPS, X_PP,-Y_MV,FLOORH,            1000, -Y_MV,0}, // place block 1 
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

void setup() {
  static int cmd_size = sizeof(cmd_array)/(SIZE_CMD_ARRAY*2);  // sizeof array.  2 bytes per int
//  static line line1;
//  line1 = forward_arm_kinematics(0/RADIAN, 0/RADIAN, 0/RADIAN, 75/RADIAN, LEN_AB, LEN_BC, LEN_CD);
  // put your setup code here, to run once:
  Serial.begin(9600); // baud rate
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 
  test_machine = setup_ms(LEN_BC, 0.0, LEN_AB,cmd_size);
  test_machine.state = 1; 
  Serial.print("cmd_size,");
  Serial.println(cmd_size);
//  Serial.println(line1.p1.x,1);
//  Serial.println(line1.p1.y,1);
//  Serial.println(line1.p1.z,1);
//  Serial.println(line1.p2.x,1);
//  Serial.println(line1.p2.y,1);
//  Serial.println(line1.p2.z,1); 
  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(0 ,134,  0/RADIAN, 895, 178/RADIAN); 
  jB.pot = set_pot(1 ,500,-90/RADIAN, 908,  0/RADIAN); 
  jC.pot = set_pot(3 , 116,-90/RADIAN, 903, 90/RADIAN); 
  jD.pot = set_pot(2 ,250, 60/RADIAN, 747, -60/RADIAN); 
  jT.pot = set_pot(4 ,160, -90/RADIAN, 510, 0/RADIAN); 
  jS.pot = set_pot(5 , 0, 0/RADIAN, 1023, 280/RADIAN);  // to tune

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  jA.svo = set_servo(0,  0.0/RADIAN, 960, 170.0/RADIAN, 2200);
  jB.svo = set_servo(1, 0.0/RADIAN, 1500, 80.0/RADIAN, 1070); // high to low

  jC.desired_angle = 0.0;  // initialize
  test_machine.angClaw = 45.0/RADIAN;
  delay(10); // not sure why, but adafruit did it.
}

void loop() {
  // put your main code here, to run repeatedly:
  static int old_state = 2;
  static float *angles;
  static unsigned long mst;
  static line lineCD;

  mst = millis();
  test_machine.dt = mst - test_machine.prior_mst; // delta time
  test_machine.prior_mst = mst;
  
  jS.pot_value = analogRead(jS.pot.analog_pin);  // read the selector
  if (jS.pot_value < 600) {
    test_machine.state = 1;  // read command list
    if (old_state != 1) {
      test_machine.initialize = true;
    }
  } else {
    test_machine.state = 2;  // Manually control using input arm
    if (old_state != 2) {
      test_machine.initialize = true;
    }
  }
  old_state = test_machine.state;
  state_setup(test_machine);  // check if state changed at t   

  Serial.print("ms,");
  Serial.print(mst);
  Serial.print(", STATE,");
  Serial.print(test_machine.state);
  Serial.print(", N,"); // command line
  Serial.print(test_machine.n);
  Serial.print(", CMD,");
  Serial.print(cmd_array[test_machine.n][0]); 

   switch (test_machine.state) {
    case 0:  // DO NOTHING STATE
      break;
    case 1: // COMMAND CONTROL
      commands_loop(test_machine,cmd_array);
      // GET OUTPUT ANGLES FROM INPUTS
      angles = inverse_arm_kinematics(test_machine.at_ptC,LEN_AB,LEN_BC,test_machine.at_ptD); 
      break;
    case 2:  // MANUAL CONTROL
      jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
      jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
      jC.pot_value = analogRead(jC.pot.analog_pin);  // read joint Claw
      jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
      jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
    
      pot_map(jA);
      pot_map(jB);
      pot_map(jC);  // Claw
      test_machine.angClaw = jC.desired_angle;
      pot_map(jD); // Wrist
      pot_map(jT); // Turntable
      lineCD = forward_arm_kinematics(jA.pot_angle/RADIAN,jB.pot_angle/RADIAN,jD.pot_angle/RADIAN,jT.pot_angle/RADIAN, LEN_AB, LEN_BC, LEN_CD);
      inputArmLoop(test_machine, lineCD, 600);  // limit movement to 100 mm per second
      angles = inverse_arm_kinematics(test_machine.at_ptC,LEN_AB,LEN_BC,test_machine.at_ptD); 
      break;
  }
  jA.desired_angle = angles[0];
  jB.desired_angle = angles[1];
  jT.desired_angle = angles[2];
  jD.desired_angle = angles[3];
  jC.desired_angle = test_machine.angClaw;

  // ADD CODE HERE TO DRIVE SERVOS
  // GET SERVO Pulse width VALUES FROM ARM OUTPUT ANGLE
  servo_map(jA);
  servo_map(jB);  

  Serial.print(",C,");
  Serial.print(test_machine.at_ptC.x);
  Serial.print(",");
  Serial.print(test_machine.at_ptC.y);
  Serial.print(",");
  Serial.print(test_machine.at_ptC.z);

  logData(jA,'A');
  logData(jB,'B');
  //logData(jC,'C');
  //logData(jD,'D');
  //logData(jT,'T');
  //logData(jS,'S');
  Serial.println(", END");
}
//
