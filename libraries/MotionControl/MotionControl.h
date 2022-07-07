#define LEN_AB 195.0     // Length of Input AB arm in mm
#define LEN_BC 240.0     // Length of Input BC arm in mm
#define RADIAN 57.2957795  // number of degrees in one radian

static float t_limit = 120.0 / RADIAN;  // Turntable limits

#define R 150.0   // radius of motion circle
#define XC 200.0  // X offset of the motion circle

float ptC[3] = {0.0,240.0,150.0};

float rot_x(float x, float y, float a) {
  return x*cos(a)-y*sin(a);
}

float rot_y(float x, float y, float a) {
  return x*sin(a)+y*cos(a);
}

float * inverse_arm_kinematics(float c[2], float l_ab, float l_bc) {
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
  
  xy_len = sqrt(pow(c[0],2)+pow(c[1],2)); 

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

/*
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // baud rate, slower is easier to read
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 
  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long millisTime;
  static float *angles;
  static float time_ang;

  millisTime = millis();
  time_ang = millisTime*0.0003;

  ptC[0] = XC + R * cos(time_ang);
  ptC[1] = R * sin(time_ang);
//
  Serial.print("Cx,");
  Serial.print(ptC[0]);
  Serial.print(" ,Cy,");
  Serial.print(ptC[1]);
//  
  angles = inverse_arm_kinematics(ptC,LEN_AB,LEN_BC);
//
  Serial.print(" ,A,");
  Serial.print(angles[0]*RADIAN);
  Serial.print(" ,B,");
  Serial.print(angles[1]*RADIAN);
  Serial.print(" ,T,");
  Serial.print(angles[2]*RADIAN);
//
  Serial.println(", END");

}
*/
