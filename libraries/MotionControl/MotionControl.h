/*
 * Robot Arm Motion Control Include file
 * By SrAmo,  August 2022
 * 
 *  The bottom of this sketch (setup and loop functions) should be
 *  commented-out when saved as an include (.h) file.
 *  alpha angles are global.  theta angles are local.
 */
#define RADIAN 57.2957795  // number of degrees in one radian

struct point {float x,y,z;}; 

struct line {struct point p1 , p2; };

struct machine_state {
  int state; // 0=TBD,1=code_array,2=manual: Controlled by S potentiometer
  boolean initialize; // =true if one needs to initialize state
  int n; // current active index in the command array
  int cmd_size; // size of the command array
  unsigned long prior_mst; // clock time (microseconds) prior loop 
  unsigned long dt; // delta loop time in ms
  unsigned long timerStart; // timer, for delays
  float moveDist; // distance to travel in current move
  point at_ptG; // current G (grabber) point 
  float alphaC; // current global C angle
  float alphaD; // current global D angle
  float angClaw;
};

#define SIZE_CMD_ARRAY 7  // NUMBER OF VALUES PER COMMAND
/*
 *  CODES:
 *  0 = TIMER, followed by the time delay in milliseconds (keeps looping)
 *  1 = LINE MOVE, followed by feed rate, then G (grabber) go-to point, alphaC, alphaD
 *  2 = Claw Move, followed by time delay in milliseconds, then claw position (degrees)
 *  TBD = ARC MOVE, followed by feed rate, then go-to pointS
 *  
 *  Feed rate is in mm per second.
 *  
 *  array     = {{code,feed rate/delay, Gx-to, Gy-to, Gz-to, alphaC, alphaD},{},{}};
 *  loc in array ={  0,   1           , 2    , 3    ,  4   , 5     ,  6    } 
 */
/*  Example of a command array and getting the size
static int cmd_array[][SIZE_CMD_ARRAY]={{1,100,300,0,250,0,0},
                           {0,3000,0,0,0,0,0}, // 3 second delay
                           {1,100,200,-200,250,0,0},
                           {2,500,30,0,0,0,0}}; // 1/2 second 30 deg claw move
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

machine_state setup_ms(float xG, float yG, float zG, int cmd_size) { // starting point G on arm
  struct machine_state ms;
  ms.state = 0;
  ms.initialize = true;
  ms.n = 0;  // first command
  ms.cmd_size = cmd_size;
  ms.prior_mst = millis();
  ms.at_ptG.x = xG;  ms.at_ptG.y = yG;  ms.at_ptG.z = zG;
  ms.alphaC = 0.0;
  ms.alphaD = 0.0;
  ms.angClaw = 45;  // full open
  return ms;
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

float * inverse_arm_kinematics(point c, float l_ab, float l_bc) {
  // Given arm system GroundT-TA-AB-BC-CD, where T (turntable) and A are [0,0,0]
  // The location of joint C (c[])  AND CX IS POSITIVE ONLY - TO DO SOLVE NEGATIVES
  // The lengths Length AB (l_ab) and Length BC (l_bc) are specified
  // The joints A,B are on a turntable with rotation T parallel to Z through A
  // With T_angle = 0, then joints A & B are parallel to the Y axis
  // calculate the angles given pt C ***Inverse Kinematics***
  // returns an array with [A_angle,B_angle,T_angle] in radians
  
  float xy_len, c_len, sub_angle1, sub_angle2;
  static float angles[3] = {0.0,0.0,0.0};  // [ A , B , T ]
  static point c_new;
  
  if (c.x > 0) { // This math only works for positive Cx
    angles[2] = atan2(c.y,c.x);  // note: atan2 order; y,x

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

point clawToC(point g, float alphaC, float alphaD, float s_CG_x, float s_CG_y) {
  // Claw pickup point is g.  Determine c based on joint c and d angles.
  static point c, c1, c2;
  static float thetaT;
  c.x = -s_CG_x;  c.y = 0.0;  c.z = -s_CG_y;
  c1 = rot_pt_x(c,alphaD);  // rotate D
  c2 = rot_pt_y(c1,alphaC); // rotate C
  thetaT = atan2(g.y,g.x); // expected turntable angle
  c1 = rot_pt_z(c2,thetaT); // correct for turntable angle
  c2 = add_pts(c1,g);
  return c2;
}
point anglesToC(float a, float b, float t, float l_ab, float l_bc){
  // Apply translations and rotations to get the C point
  //   Forward Kinematics
  static point pB,pC;
  static point pC2, pC3;  // temporary points for calculations 
  
  pC.x = l_bc;   pC.y = 0.0;  pC.z = 0.0;
  pB.x = l_ab;   pB.y = 0.0;  pB.z = 0.0;
  
  pC2 = rot_pt_y(pC,b);
  pC3 = add_pts(pB,pC2);
  pC2 = rot_pt_y(pC3,a); // reuse pC2
  pC3 = rot_pt_z(pC2,t);  // reuse pC3

  return pC3;
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

boolean machineGovenor(machine_state & machine, point to_G, int mmps, float to_C, float to_D) { // limits movement given feed rate
  // updates .at_ptG based on speed limit
  static unsigned long mst;
  static float line_len, newC, newD;
  static point newG;
  
  mst = millis();
  machine.dt = mst - machine.prior_mst; // delta time
  machine.prior_mst = mst;
  machine.moveDist = mmps * (machine.dt/1000.0); // feed rate (mm/sec)*sec = mm
  
  line_len = ptpt_dist(machine.at_ptG,to_G);
  if (machine.moveDist < line_len) { // PARTIAL MOVE
    newG = pt_on_line(machine.moveDist,line_len, machine.at_ptG,to_G);
    machine.at_ptG = newG;
    newC = interpolate(machine.moveDist,line_len, machine.alphaC,to_C);
    machine.alphaC = newC;
    newD = interpolate(machine.moveDist,line_len, machine.alphaD,to_D);
    machine.alphaD = newD;
    return false;
  } else { // FULL MOVE
    machine.at_ptG = to_G;
    machine.alphaC = to_C;
    machine.alphaD = to_D;
    return true;
  }
}

void go_to_next_cmd(machine_state & machine) {
  machine.n += 1; // go to the next command line
  machine.moveDist = 0.0;
  machine.timerStart = millis();
  if (machine.n >= machine.cmd_size) { // end of command lines, stop
    machine.n = 9999; // signals end of commands
  }
}

void commands_loop(machine_state & machine, int cmds[][SIZE_CMD_ARRAY]) { 
  static point toCpt; 
  static float toAlphaC, toAlphaD;
  static boolean govenorDone;

  // read type of command
  if (machine.n != 9999) {
    switch (cmds[machine.n][0]) {
      case 0: // DELAY (timer)
        if ((millis()-machine.timerStart) > cmds[machine.n][1]) {
          go_to_next_cmd(machine);        
        }
        break;
      case 1: // line move
        if (machine.moveDist == 0.0) { // initialize line
            toCpt.x = cmds[machine.n][2];
            toCpt.y = cmds[machine.n][3];
            toCpt.z = cmds[machine.n][4];
            toAlphaC = cmds[machine.n][5]/RADIAN;
            toAlphaD = cmds[machine.n][6]/RADIAN;
            govenorDone = machineGovenor(machine, toCpt, cmds[machine.n][1],toAlphaC,toAlphaD); 

         } else {  // moving
            govenorDone = machineGovenor(machine, toCpt, cmds[machine.n][1],toAlphaC,toAlphaD); 
            if (govenorDone) {  
              go_to_next_cmd(machine);
            }
         }
         break;
       case 2: // Claw move, with timer to give claw time to move.
         machine.angClaw = cmds[machine.n][2]/RADIAN;  // set the claw angle
         if ((millis()-machine.timerStart) > cmds[machine.n][1]) {
            go_to_next_cmd(machine);        
          }
         break;
    } // end switch   
  }
}

void state_setup(machine_state & machine) { // Call every loop, to check for a state change
  if (machine.initialize) { 
    machine.n = 0; // reset array pointer
    machine.moveDist = 0.0;  // reset govenor distance
    machine.initialize = false;
  }  
}

// #####################################################################################33
//  COMMENT OUT BELOW HERE WHEN SAVING AS AN INCLUDE (.h) FILE
/*
// todo: pass paramaters to turn things on and off
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
//  Serial.print(", servo_ms,");
//  Serial.print(jt.servo_ms);
} 

#define LEN_AB 320.0     // Length of Input AB arm in mm
#define LEN_BC 320.0     // Length of Input BC arm in mm
#define S_CG_X 180.0 
#define S_CG_Y 40.0

struct machine_state test_machine; 
struct joint jA,jB,jC,jD,jCLAW,jT,jS;

#define MMPS 200 // mm per second
#define X_PP 280 // x mm for pick and place
#define Y_MV 200 // y swing in mm
#define FLOORH -80 // z of floor for picking
#define BLOCKH 100 // block height mm
#define ALPHAC -90 // global C angle
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

void setup() {
  static int cmd_size = sizeof(cmd_array)/(SIZE_CMD_ARRAY*2);  // sizeof array.  2 bytes per int
  // put your setup code here, to run once:
  Serial.begin(9600); // baud rate
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 
  test_machine = setup_ms(LEN_BC, 0.0, LEN_AB,cmd_size);
  test_machine.state = 1; 
  Serial.print("cmd_size,");
  Serial.println(cmd_size);
  // TUNE POT LOW AND HIGH VALUES
  // set_pot(pin,lowmv,lowang,highmv,highang)
  jA.pot = set_pot(0 ,134,  0/RADIAN, 895, 178/RADIAN); 
  jB.pot = set_pot(1 ,500,-90/RADIAN, 908,  0/RADIAN); 
  //jC.pot = set_pot(3 , 116,-90/RADIAN, 903, 90/RADIAN); // C will not have a pot on Make3
  jD.pot = set_pot(2 ,250, 60/RADIAN, 747, -60/RADIAN); 
  jT.pot = set_pot(4 ,160, -90/RADIAN, 510, 0/RADIAN); 
  jCLAW.pot = set_pot(3 , 116,-90/RADIAN, 903, 90/RADIAN);
  jS.pot = set_pot(5 , 0, 0/RADIAN, 1023, 280/RADIAN);  // to tune

  // TUNE SERVO LOW AND HIGH VALUES
  // set_servo(pin,lowang,lowms,highang,highms)
  //jA.svo = set_servo(0,  0.0/RADIAN, 960, 170.0/RADIAN, 2200);
  //jB.svo = set_servo(1, 0.0/RADIAN, 1500, 80.0/RADIAN, 1070); // high to low

  jC.desired_angle = 0.0;  // initialize
  test_machine.angClaw = 45.0/RADIAN;
  delay(10); // not sure why, but adafruit did it.
}

void loop() {
  // put your main code here, to run repeatedly:
  static int old_state = 2;
  static float *angles;
  static float alphaB;
  static point pointC;
  static boolean govenorDone;
  
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
  state_setup(test_machine);  // check if state changed

  Serial.print("ms,");
  Serial.print(test_machine.prior_mst);
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
      
      pointC =  clawToC(test_machine.at_ptG, test_machine.alphaC, test_machine.alphaD, S_CG_X, S_CG_Y);

      angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC); 
      
      break;
    case 2:  // MANUAL CONTROL  TO DO FIGURE OUT HOW TO MAP C AND D
      jA.pot_value = analogRead(jA.pot.analog_pin);  // read joint A
      jB.pot_value = analogRead(jB.pot.analog_pin);  // read joint B
      jC.pot_value = analogRead(jC.pot.analog_pin);  // read joint Claw
      jD.pot_value = analogRead(jD.pot.analog_pin);  // read D wrist
      jT.pot_value = analogRead(jT.pot.analog_pin);  // read the turntable
    
      pot_map(jA);
      pot_map(jB);
      pot_map(jC); 
      test_machine.angClaw = jC.desired_angle;
      pot_map(jD); // Wrist
      pot_map(jT); // Turntable
      pointC = anglesToC(jA.pot_angle/RADIAN,jB.pot_angle/RADIAN,jT.pot_angle/RADIAN, LEN_AB, LEN_BC);
      govenorDone = machineGovenor(test_machine, pointC, 600, jC.pot_value, jD.pot_value); 
      angles = inverse_arm_kinematics(pointC,LEN_AB,LEN_BC); 
      break;
  }
  jA.desired_angle = angles[0]; // global A = local A
  jB.desired_angle = angles[1];  // local B
  alphaB = angles[0]+angles[1];  // global B
  jT.desired_angle = angles[2];  // global T
  jC.desired_angle = test_machine.alphaC-alphaB; // convert to a local C
  jD.desired_angle = test_machine.alphaD -  jT.desired_angle; // convert to a local D
  jCLAW.desired_angle = test_machine.angClaw;

  // ADD CODE HERE TO DRIVE SERVOS
  // GET SERVO Pulse width VALUES FROM ARM OUTPUT ANGLE
  servo_map(jA);
  servo_map(jB);  

  Serial.print(",G,");
  Serial.print(test_machine.at_ptG.x);
  Serial.print(",");
  Serial.print(test_machine.at_ptG.y);
  Serial.print(",");
  Serial.print(test_machine.at_ptG.z);

  logData(jA,'A');
  logData(jB,'B');
  logData(jC,'C');
  logData(jD,'D');
  logData(jT,'T');
  logData(jCLAW,'X');
  //logData(jS,'S');
  Serial.println(", END");
}
*/
