/*
 * Robot Arm Motion Control Include file
 * By SrAmo,  July 2022
 * 
 *  The bottom of this sketch (setup and loop functions) should be
 *  commented-out when saves as an include file.
 */
#define RADIAN 57.2957795  // number of degrees in one radian

struct machine_state {
  int state; // 0=TBD,1=code_array,2=manual: CHANGED BY S POT
  boolean initialize; // =true if one needs to initialize state
  int n; // current active index in the command array
  int cmd_size; // size of the command array
  unsigned long prior_mst; // clock time (microseconds) prior loop 
  unsigned long dt; // delta loop time in ms
  float dist;
  float fr_ptC[3]; // from C point 
  float at_ptC[3]; // at (current) C point 
  float fr_ptF[3]; // from Focal point 
  float at_ptF[3]; // at (current) Focal point 
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
/* 
 *  Example of a command array and getting the size
static int cmd_array[][SIZE_CMD_ARRAY]={{1,100,300,0,250,0,0,0},
                           {0,3000,0,0,0,0,0,0},
                           {1,100,200,-200,250,0,0,0},
                           {1,50,300,300,200,0,0,0}};
 */
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

machine_state setup_ms(float xC, float yC, float zC, int cmd_size) { // starting point C on arm
  struct machine_state ms;
  ms.state = 0;
  ms.initialize = true;
  ms.n = 0;  // first command
  ms.cmd_size = cmd_size;
  ms.prior_mst = millis();
  ms.fr_ptC[0] = xC;
  ms.fr_ptC[1] = yC;
  ms.fr_ptC[2] = zC;
  ms.at_ptC[0] = xC;
  ms.at_ptC[1] = yC;
  ms.at_ptC[2] = zC;
  
  ms.fr_ptF[0] = xC+1000.0;
  ms.fr_ptF[1] = yC;
  ms.fr_ptF[2] = zC;
  ms.at_ptF[0] = xC+1000.0;
  ms.at_ptF[1] = yC;
  ms.at_ptF[2] = zC;
  return ms;
}

float rot_x(float x, float y, float a) {
  return x*cos(a)-y*sin(a);
}

float rot_y(float x, float y, float a) {
  return x*sin(a)+y*cos(a);
}
/*
float * arc_pt(float s, float rad, float rot_cent_x, float w, float h) { // rtn point on arc
  // s is normalized arc length, w is arc width (Y dir), h is Z
  static float pt[3] = {0.0,0.0,0.0}; // [ X , Y , Z ]
  b = sqrt(pow(rad,2.0)+pow(w/2.0,2.0));   
  beta = acos(rad/b);  // half arc angle
  gamma = beta * (s-0.5); // from -beta to +beta
  // rotate the arc point by gamma
  pt[0] = b*cos(gamma)+rot_cent_x;
  pt[1] = b*sin(gamma);
  pt[2] = h;
  return pt;
}
*/
float ptpt_dist(float p1[3], float p2[3]) {
  static float dist;
  dist = sqrt(pow(p1[0]-p2[0],2.0)+pow(p1[1]-p2[1],2.0)+pow(p1[2]-p2[2],2.0) ); 
  return dist;
}
float * pt_on_line(float s, float dist, float p1[3], float p2[3]) {
  static float new_pt[3],ratio;
  ratio = s / dist; 
  new_pt[0] = ratio*(p2[0]-p1[0]) + p1[0];
  new_pt[1] = ratio*(p2[1]-p1[1]) + p1[1];
  new_pt[2] = ratio*(p2[2]-p1[2]) + p1[2];
  return new_pt;
}
float * inverse_arm_kinematics(float c[3], float l_ab, float l_bc, float f[3]) {
  // Given arm system GroundT-TA-AB-BC-CD, where T (turntable) and A are [0,0,0]
  // The location of joint C (c[])
  // The location of F (focal point) (where the claw should point)
  // The lengths Length AB (l_ab) and Length BC (l_bc) are specified
  // The joints A,B are on a turntable with rotation T parallel to Z through A
  // With T_angle = 0, then joints A & B are parallel to the Y axis
  // calculate the angles given pt C ***Inverse Kinematics***
  // returns an array with [A_angle,B_angle,T_angle,D_angle] 
  
  float xy_len, c_len, sub_angle1, sub_angle2;
  static float t_limit = 120.0 / RADIAN;  // Turntable limits, should be an argument??
  static float angles[4] = {0.0,0.0,0.0,0.0};  // [ A , B , T , D]
  static float c_new[3];
  
  // compute the turntable angle
  xy_len = sqrt(pow(c[0],2.0)+pow(c[1],2.0)); 
  if (xy_len > 0) {  
    angles[2] = atan2(c[1],c[0]);  //  y,x
  } else {
    if (c[2] == 0.0) {
      // case where c = [0,0,0]
      return angles;
    } else {
      // the robot arm could be pointing along the z axis
      angles[2] = 0.0; // turntable = 0
    }
  }

  // compute the D angle based on where C and F are on the xy plane
  xy_len = sqrt(pow((f[0]-c[0]),2.0)+pow((f[1]-c[1]),2.0)); 
  if (xy_len > 0) {  
    angles[3] = atan2((f[1]-c[1]),(f[0]-c[0]));  //  y,x
  } else {
    angles[3] = 0.0; 
  }

  // rotate the point c onto the XZ plane
  c_new[0] = rot_x(c[0],c[1],-angles[2]);
  c_new[1] = rot_y(c[0],c[1],-angles[2]);
  c_new[2] = c[2];
  c_len = sqrt(pow(c_new[0],2)+pow(c_new[2],2));   // XZ plane

  if (c_len < l_ab+l_bc) {
    // case where robot arm can reach
    sub_angle1 = atan2(c_new[2],c_new[0]);
    sub_angle2 = acos((pow(c_len,2)+pow(l_ab,2)-pow(l_bc,2))/(2*c_len*l_ab));
    angles[0] = sub_angle1 + sub_angle2;
    angles[1] = acos((pow(l_bc,2)+pow(l_ab,2)-pow(c_len,2))/(2*l_bc*l_ab))-180.0/RADIAN;
  } else {
    // case where robot arm can not reach point... 
    angles[0] = atan2(c_new[2],c_new[0]);
    angles[1] = 0.0;
  }
  
  // limit the turntable range, so as not to break things
  if (angles[2] > t_limit) {
    angles[2] = t_limit;
  } else if (angles[2] < -t_limit) {
    angles[2] = -t_limit;
  }

  return angles;  // return the angles
}
void loop_circle() { // NOT DONE
/*  CIRCLE TEST
#define R_TEST 150.0   // radius of motion circle
#define XC_TEST 200.0  // X offset of the motion circle
      ds = cmd_array[n][1] * (test_machine.dt/1000.0); // s = feed distance in mm
      dist += ds;
      //test_machine.dist += ds;
      ang += ds/R_TEST;  // feed angle in radians

      if (ang < 1.0) {
        ptC[0] = XC_TEST + R_TEST * cos(ang);
        ptC[1] = R_TEST * sin(ang);
        
        Serial.print(" , dist,");
        Serial.print(dist,3);
        Serial.print(" , ang,");
        Serial.print(ang,2); 
        */
  
}
/*
float * arc_pt(float s, float rad, float rot_cent_x, float w, float h) { // rtn point on arc
  // s is normalized arc length, w is arc width (Y dir), h is Z
  static float pt[3] = {0.0,0.0,0.0}; // [ X , Y , Z ]
  static float b, beta, gamma;
  b = sqrt(pow(rad,2.0)+pow(w/2.0,2.0));   
  beta = acos(rad/b);  // half arc angle
  gamma = beta * (s-0.5); // from -beta to +beta
  // rotate the arc point by gamma
  pt[0] = b*cos(gamma)+rot_cent_x;
  pt[1] = b*sin(gamma);
  pt[2] = h;
  return pt;
}

void path1_loop() {
  static float *angles; // pointer to angles array
  static float time_ang,s;
  static float focal_pt[3] = {500.0,0.0,Z_PATH}; 
  static float *ptC;  // pointer to points array
  if (path1_init) {
    // first time in this function, do initialize
    //  and set the other function to require initialize
    path1_init=false; 
    path2_init=true;
    hold_init=true;  
    input_arm_init=true;
    
     // initialize joints
    jA.desired_angle = 80.0;
    jB.desired_angle = 20.0;
    jC.desired_angle = -70.0;
    jD.desired_angle = 80.0;
    jT.desired_angle = 0.0;  

    //main_ang_velo = 0.02;

  } else {
      millisTime = millis();
      time_ang = millisTime*0.001;

      s = time_ang;  //  HOW??
      ptC = arc_pt(s,R,XC,LEN_AB*1.5,Z_PATH);
      
      ptC[0] = XC + R * cos(time_ang);
      ptC[1] = R * sin(time_ang);
      ptC[2] = Z_PATH;   
    
      angles = inverse_arm_kinematics(ptC,LEN_AB,LEN_BC);
      jA.desired_angle = angles[0]*RADIAN;
      jB.desired_angle = -angles[1]*RADIAN - jA.desired_angle;
      jT.desired_angle = angles[2]*RADIAN;  

      //  wrist angle
      //jD.desired_angle = ??(ptC,focal_pt);  atan2(ptC[1],(focal_pt[0]-ptC[0]) );  // y,x

      #if SERIALOUT
        Serial.print(", A,");
        Serial.print(jA.desired_angle);
        Serial.print(", B,");
        Serial.print(jB.desired_angle);
        Serial.print(", Table,");
        Serial.print(jT.desired_angle);
        Serial.print(", Wrist,");
        Serial.print(jD.desired_angle);
      #endif
      }
    }
*/
void state_setup(machine_state & machine) {
  // This is called every loop, to check for a state change
  if (machine.initialize) { // initialize
    machine.n = 0; // array pointer
    machine.dist = 0.0;  // could use a static variable?
    machine.initialize = false;
  }  
}

void go_to_next_cmd(machine_state & machine) {
  machine.n += 1; // go to the next command line
  machine.dist = 0.0;
  if (machine.n >= machine.cmd_size) { // end of command lines, stop
    machine.state = 0;
    machine.initialize = true; // tell next state to initialize
  }
}

void commands_loop(machine_state & machine, int cmds[][SIZE_CMD_ARRAY]) { 
  static float ds, line_len, to_pt[3];
  static float *at_pointC; // pointer to a point!

  // read type of command
  switch (cmds[machine.n][0]) {
    case 0: // DELAY
      delay(cmds[machine.n][1]);
      machine.prior_mst = millis();
      go_to_next_cmd(machine);
      break;
    case 1: // line move
      ds = cmds[machine.n][1] * (machine.dt/1000.0); // feed rate (mm/sec)*sec = mm
      if (machine.dist == 0.0) { // initialize line
        to_pt[0] = cmds[machine.n][2];
        to_pt[1] = cmds[machine.n][3];
        to_pt[2] = cmds[machine.n][4];
        line_len = ptpt_dist(machine.fr_ptC,to_pt);
        at_pointC = pt_on_line(machine.dist,line_len, machine.fr_ptC,to_pt);
        machine.dist += ds;
      } else {  // moving
          if (machine.dist <= line_len) {  // move along line 
            at_pointC = pt_on_line(machine.dist,line_len, machine.fr_ptC,to_pt);
            machine.dist += ds;
          } else {  // reached end point
            machine.fr_ptC[0] = to_pt[0];
            machine.fr_ptC[1] = to_pt[1];
            machine.fr_ptC[2] = to_pt[2];
            go_to_next_cmd(machine);
          }
       }
       machine.at_ptC[0] = at_pointC[0];
       machine.at_ptC[1] = at_pointC[1];
       machine.at_ptC[2] = at_pointC[2];
       break;
     case 2: // TBD
       break;
  } // end switch
}

/*
 *  STRUCTURES FOR SERVOS, POTENTIOMETERS, AND JOINTS
 */


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
  // Converts initial_angle to Servo Microseconds
  jt.pot_value = 500;  // middle ish
  jt.desired_angle = initial_angle;
  jt.previous_angle = initial_angle;
  jt.servo_ms = map(initial_angle,jt.svo.low_ang,jt.svo.high_ang,jt.svo.low_ms,jt.svo.high_ms);
  jt.previous_millis = millis();
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
// SERVO Map With RATE LIMITING, for smooth operation and to prevent damage
// Limit how much a servo angle can change in a unit time

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
    Serial.print(",");
    Serial.print(jt_letter);
    Serial.print(", p_value,");
    Serial.print(jt.pot_value);
    Serial.print(", Pang,");
    Serial.print(jt.pot_angle,1);
    //Serial.print(", PrevAngle,");
    //Serial.print(jt.previous_angle,1);
    Serial.print(", DESAngle,");
    Serial.print(jt.desired_angle,1);
    Serial.print(", servo_ms,");
    Serial.print(jt.servo_ms);
}


//  COMMENT OUT BELOW HERE WHEN SAVING AS AN INCLUDE (.h) FILE
//
#define LEN_AB 195.0     // Length of Input AB arm in mm
#define LEN_BC 240.0     // Length of Input BC arm in mm

struct machine_state test_machine; 

static int cmd_array[][SIZE_CMD_ARRAY]={{1,100,LEN_BC-20,0,LEN_AB,0,0,0},
                           {0,3000,0,0,0,0,0,0},
                           {1,100,LEN_BC-40,-200,LEN_AB,0,0,0},
                           {1,50,LEN_BC,LEN_BC,LEN_AB,0,0,0}};

void setup() {
  static int cmd_size = sizeof(cmd_array)/(SIZE_CMD_ARRAY*2);  // sizeof array.  2 bytes per int
  // put your setup code here, to run once:
  Serial.begin(9600); // baud rate, slower is easier to read
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 
  test_machine = setup_ms(LEN_BC, 0.0, LEN_AB,cmd_size);
  test_machine.state = 1; 
  Serial.print("cmd_size,");
  Serial.println(cmd_size);

  delay(10); // not sure why, but adafruit did it.
}

void loop() {
  // put your main code here, to run repeatedly:
  static float *angles;
  static unsigned long mst;

  mst = millis();
  test_machine.dt = mst - test_machine.prior_mst; // delta time
  test_machine.prior_mst = mst;
  
  Serial.print("ms,");
  Serial.print(mst);
  Serial.print(", STATE,");
  Serial.print(test_machine.state);
  Serial.print(", N,"); // command line
  Serial.print(test_machine.n);
  Serial.print(", CMD,");
  Serial.print(cmd_array[test_machine.n][0]);

   switch (test_machine.state) {
    case 0:
      state_setup(test_machine);
      break;
    case 1:
      state_setup(test_machine);
      commands_loop(test_machine,cmd_array);
      // GET OUTPUT ANGLES FROM INPUTS
      angles = inverse_arm_kinematics(test_machine.at_ptC,LEN_AB,LEN_BC,test_machine.at_ptF); 
      break;
    case 2:
      state_setup(test_machine);
      //loop_circle();
      break;
  }
  
  // ADD CODE HERE TO DRIVE SERVOS

  Serial.print(", Cx,");
  Serial.print(test_machine.at_ptC[0]);
  Serial.print(" ,Cy,");
  Serial.print(test_machine.at_ptC[1]);
  Serial.print(" ,Cz,");
  Serial.print(test_machine.at_ptC[2]);
  Serial.print(" ,A,");
  Serial.print(angles[0]*RADIAN);
  Serial.print(" ,B,");
  Serial.print(angles[1]*RADIAN);
  Serial.print(" ,T,");
  Serial.print(angles[2]*RADIAN);
  Serial.print(" ,D,");
  Serial.print(angles[3]*RADIAN);
  Serial.println(", END");
}
//
