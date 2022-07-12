#define LEN_AB 195.0     // Length of Input AB arm in mm
#define LEN_BC 240.0     // Length of Input BC arm in mm
#define RADIAN 57.2957795  // number of degrees in one radian
#define MMPS 30.0 // Feed rate, mm per second
#define R_TEST 150.0   // radius of motion circle
#define XC_TEST 200.0  // X offset of the motion circle

struct machine_state {
  int state; // 0=TBD,1=code_array_line,2=TBD: CHANGED BY S POT
  boolean initialize; // true=need to initialize state
  int n; // current active index in the code_array
  //int code; // motion command 0=delay,1=line,2=arc
  unsigned long prior_mst; // clock time (microseconds) prior loop 
  unsigned long dt; // delta loop time in ms
  float dist;
  float prior_pt[3]; // x,y,z C point, prior 
};

struct machine_state test_machine;

static float t_limit = 120.0 / RADIAN;  // Turntable limits

static float *ptC;  

#define SIZE_CMD_ARRAY 8
// {code,feed rate/delay, x, y, z, TBD1, TBD2, TBD3}
// {  0 ,   1           , 2, 3, 4, 5   ,  6  ,  7  }
static int cmd_array[][SIZE_CMD_ARRAY]={{1,100,LEN_BC,0,LEN_AB,0,0,0},
                           {0,3000,0,0,0,0,0,0},
                           {1,100,LEN_BC,-200,LEN_AB,0,0,0},
                           {1,50,LEN_BC,LEN_BC,LEN_AB,0,0,0}};
static int cmd_size = sizeof(cmd_array)/(SIZE_CMD_ARRAY*2);  // sizeof array.  2 bytes per int

machine_state setup_ms() { 
  struct machine_state ms;
  ms.state = 0;
  ms.initialize = true;
  ms.n = 0;  // 
  ms.prior_mst = millis();
  ms.prior_pt[0] = LEN_BC/2;
  ms.prior_pt[1] = 0;
  ms.prior_pt[2] = LEN_AB/2;
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
  ratio = s / dist;  // should not recalculate distance
  new_pt[0] = ratio*(p2[0]-p1[0]) + p1[0];
  new_pt[1] = ratio*(p2[1]-p1[1]) + p1[1];
  new_pt[2] = ratio*(p2[2]-p1[2]) + p1[2];
  return new_pt;
}
float * inverse_arm_kinematics(float c[3], float l_ab, float l_bc) {
  // Given a four body system GroundT-TA-AB-BC, where T (turntable) and A are [0,0,0]
  // The location of C is specified (c[])
  // The lengths Length AB (l_ab) and Length BC (l_bc) are specified
  // The joints A,B are on a turntable with rotation T parallel to Z through A
  // With T_angle = 0, then joints A & B are parallel to the Y axis
  // calculate the angles given pt C ***Inverse Kinematics***
  // returns an array with [A_angle,B_angle,T_angle] 
  
  float xy_len, c_len, sub_angle1, sub_angle2;
  
  static float angles[3] = {0.0,0.0,0.0};  // [ A , B , T ]
  static float c_new[3];
  
  xy_len = sqrt(pow(c[0],2.0)+pow(c[1],2.0)); 

  // compute the turntable angle
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
  
  // limit the turntable range to + or - 90
  if (angles[2] > t_limit) {
    angles[2] = t_limit;
  } else if (angles[2] < -t_limit) {
    angles[2] = -t_limit;
  }

  return angles;  // return the angles
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // baud rate, slower is easier to read
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 
  test_machine = setup_ms();
  test_machine.state = 1; 
  //Serial.print("cmd_size,");
  //Serial.println(cmd_size);

  ptC[0] = LEN_BC/2;
  ptC[1] = 10;
  ptC[2] = LEN_AB/2;

  delay(10);
}
void loop_circle() { // NOT DONE
/*  CIRCLE TEST
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
void state_setup() {
  // This is called every loop, but only do at beginning of state change
  if (test_machine.initialize) { // initialize
    Serial.print(", INITIALIZE,");
    test_machine.n = 0; // array pointer
    test_machine.dist = 0.0;  // could use a static variable?
    test_machine.initialize = false;
  }  
}

void go_to_next_cmd() {
  test_machine.n += 1; // go to the next command line
  test_machine.dist = 0.0;
  if (test_machine.n >= cmd_size) { // end of command lines, stop
    test_machine.state = 0;
    test_machine.initialize = true; // tell next state to initialize
  }
}

void commands_loop() {
  static float ds, line_len, to_pt[3];
//  Serial.print(", CMD,");
//  Serial.print(cmd_array[test_machine.n][0]);
  // read type of command
  switch (cmd_array[test_machine.n][0]) {
    case 0: // TBD  DELAY?
       Serial.print(", delay,");
       Serial.print(cmd_array[test_machine.n][1]);
       
      delay(cmd_array[test_machine.n][1]);
      test_machine.prior_mst = millis();
        
      go_to_next_cmd();
      break;
    case 1: // line move
        ds = cmd_array[test_machine.n][1] * (test_machine.dt/1000.0); // s = feed line_len in mm
        if (test_machine.dist == 0.0) { // initialize line
          to_pt[0] = cmd_array[test_machine.n][2];
          to_pt[1] = cmd_array[test_machine.n][3];
          to_pt[2] = cmd_array[test_machine.n][4];
          line_len = ptpt_dist(test_machine.prior_pt,to_pt);
       //Serial.print(", linelen,");
       //Serial.print(line_len,2);
          ptC = pt_on_line(test_machine.dist,line_len, test_machine.prior_pt,to_pt);
           test_machine.dist += ds;
        } else {  // moving
          if (test_machine.dist <= line_len) {  // move along line 
            ptC = pt_on_line(test_machine.dist,line_len, test_machine.prior_pt,to_pt);
            test_machine.dist += ds;
          } else {  // reached end point
            test_machine.prior_pt[0] = to_pt[0];
            test_machine.prior_pt[1] = to_pt[1];
            test_machine.prior_pt[2] = to_pt[2];
            
            go_to_next_cmd();
          }
       }
       Serial.print(", dist,");
       Serial.print(test_machine.dist,2);
       break;
    case 2: // TBD
      break;
  }
}
//
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
  Serial.print(", N,");
  Serial.print(test_machine.n);

  // GET INPUTS depending on state
  switch (test_machine.state) {
    case 0:
      state_setup();
      break;
    case 1:
      state_setup();
      commands_loop();
      break;
    case 2:
      state_setup();
      //loop_circle();
      break;
  }
  // GET OUTPUT ANGLES FROM INPUTS
  angles = inverse_arm_kinematics(ptC,LEN_AB,LEN_BC); 
//

  Serial.print(", Cx,");
  Serial.print(ptC[0]);
  Serial.print(" ,Cy,");
  Serial.print(ptC[1]);
  Serial.print(" ,Cz,");
  Serial.print(ptC[2]);
  Serial.print(" ,A,");
  Serial.print(angles[0]*RADIAN);
  Serial.print(" ,B,");
  Serial.print(angles[1]*RADIAN);
  Serial.print(" ,T,");
  Serial.print(angles[2]*RADIAN);
  Serial.println(", END");
//
}
//
