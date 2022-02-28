/* ROBOT ARM CONTROL SOFTWARE
 *  By, SrAmo, February 2022
 */
#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#define SERIALOUT false  // COMMENT-OUT TO TURN OFF SERIAL output. 

#define TUNE false  // TRUE WHEN TUNING ONE POT/SERVO,  FALSE WHEN DOING ALL SERVOS

#define lenAB 50.0     // Length of Input AB arm in mm
#define lenBC 60.0     // Length of Input BC arm in mm
#define RADIAN 57.2957795  // number of degrees in one radian
#define SCLR 1  // scaler used to improve accuracy of int variables

// CONSTANTS:
// Use PIN_ constants to identify the analog pin (0-5) for the Potentiometer.
// Use POT_ constants to set potetiometer ranges from 0 (0 Volts) to 1023 (5 Volts)
// Use INPUTARM_ constants to map the Input Angle (deg)for a given potentiometer value
// Compute Output Arm Angles from INPUTS and map() to compute value for servos.
// Use SVODIG_ to identify the digital pin that the servo is attached to.
// Use SVO_ constants to set servo range in microseconds - 
// Values are microseconds (~700 to ~2300--SERVOS MUST BE TESTED), REGARDLESS OF SERVO RANGE
// 270 deg servo, gets 260 deg motion with 500 to 2800 microsecond range
// 180 deg servo, gets 160 deg motion with 500 to 2800 microsecond range
// 180 deg SAVOX, gets 170 deg motion with 500 to 2800 microsecond range
//    ** Start small and increase with tuning **
//    ** to avoid current overloads           **

// SINGLE TEST VALUES (NO JOINT SUFFIX)
#define PIN 5    // TEST PIN IS ZERO
#define POT_MIN 0   // 0 TO 1023
#define POT_MAX 1000   // 0 TO 1023
const int INPUTARM_MIN = -100*SCLR;  // DEG at POT_MIN
#define INPUTARM_MAX 40  // DEG at POT_MAX
// CONSTRAIN INPUT??
#define SVODIG 6
#define OUTPUTARM_MIN -100  // DEG
#define OUTPUTARM_MAX 40  // DEG
#define SVO_MIN 2200   // microseconds @ OUTPUTARM_MIN angle
#define SVO_MAX 500  // microseconds @ OUTPUTARM_MAX angle

// JOINT C, COMPUTED INPUT.
//  TUNER POT IN THE FUTURE, Analog 5 input pin, Digital 6 output pin
#define PIN_C 5    // The C potentiometer is for tuning the Claw position.
#define POT_MIN_C 0   // 0 TO 1023
#define POT_MAX_C 1000   // 0 TO 1023
#define INPUTARM_MIN_C -100  // DEG CW (-110) Smaller than actual angle
#define INPUTARM_MAX_C 40  // DEG CCW (40) Smaller than actual angle
#define TUNEARM_MIN_C -30  // min for how much tuning can be added
#define TUNEARM_MAX_C 30  // max for how much tuning can be added
#define SVODIG_C 6   // Currently 180 deg servo. Could TO UPGRADE TO 270 DEG SERVO
#define OUTPUTARM_MIN_C -100
#define OUTPUTARM_MAX_C 40
#define SVO_MIN_C 2200   // microseconds
#define SVO_MAX_C 500  // microseconds


// JOINT A, Analog 4 input pin, Digital 9 output
#define PIN_A 4
#define POT_MIN_A 134
#define POT_MAX_A 870
#define INPUTARM_MIN_A  0
#define INPUTARM_MAX_A  175
#define SVODIG_A 9
#define OUTPUTARM_MIN_A 0  // DEG
#define OUTPUTARM_MAX_A 160  // DEG SHOULD BE same as input limits
#define SVO_MIN_A 960   // microseconds
#define SVO_MAX_A 2200  // microseconds

// JOINT B, Analog 1 input pin, Digital 5 output pin
#define PIN_B 1
#define POT_MIN_B  0
#define POT_MAX_B  957
#define INPUTARM_MIN_B  -125
#define INPUTARM_MAX_B  120
#define SVODIG_B 5
#define OUTPUTARM_MIN_B -45  // DEG + 90
#define OUTPUTARM_MAX_B 90  // DEG + 90
#define SVO_MIN_B 2110   // microseconds
#define SVO_MAX_B 1130  // microseconds

// ROBOT ARM DATA
// TURNTABLE, Analog 0 input pin, Digital 11 output pin
#define PIN_T 0
#define POT_MIN_T 130
#define POT_MAX_T 930
#define INPUTARM_MIN_T 100  // DEG
#define INPUTARM_MAX_T -90  // DEG
#define SVODIG_T 11
// straight out is at 0 DEG, XXX microseconds
#define OUTPUTARM_MIN_T -25  // DEG, was -45
#define OUTPUTARM_MAX_T 50  // DEG, was 70
#define SVO_MIN_T 500   // microseconds
#define SVO_MAX_T 2300  // microseconds

// JOINT D (CLAW), Analog 2 input pin, Digital 10 output pin
#define PIN_D 2
#define POT_MIN_D  370
#define POT_MAX_D  700
#define INPUTARM_MIN_D 0  // DEG
#define INPUTARM_MAX_D 160  // DEG
#define SVODIG_D 10
#define OUTPUTARM_MIN_D 0  // DEG
#define OUTPUTARM_MAX_D 160  // DEG
#define SVO_MIN_D 500   // microseconds
#define SVO_MAX_D 2300  // microseconds

#ifdef SERIALOUT
  #define SVO_RATE_LIMIT 50.0  // UNITS OF servo microseconds per loop. 50 is good for turntable with serial on.
#else
  #define SVO_RATE_LIMIT 0.2  // UNITS OF servo microseconds per loop.  0.2 is good for turntable with serial off.
#endif  


// ##### GLOBAL VARIABLES #####
Servo servo_tune;  // servo for tuning
Servo servoA,servoB,servoC,servoD,servoT;  // servos for robot arm

struct joint {
  int pot_value;
  int pot_angle;  // input device angle
  int pot_angle_min;
  int pot_angle_max;
  int arm_angle;  // robot arm angle
  int servo_ms;   // servo command in microseconds
  int servo_ms_prior; // used to look at servo rate of change
};
struct joint jA,jB,jC,jD,jT;

int pot; // pot value for tuning
int min_pot,max_pot;  // stores min and max potentiometer reading
float inputJoint;
float outputJoint;
int outputServo;
int potA,potB,potC,potD,potT; // pot values for input arm
int priorT,deltaT;  // stores the computed value for the servo
float inputA,inputB,inputD,inputT;
float outputA,outputB,outputAB,outputC,outputD,outputT;
float tuneCangle;
int output_servoA,output_servoB,output_servoC,output_servoD,output_servoT;
unsigned long millisTime;
float cx,cy;
float new_ang_a,new_ang_b;

/*
float otheracos(float x)
{
  float negate = float(x < 0);
  float ret = -0.0187293;
  x = abs(x);
  ret = x * -0.0187293;
  ret += 0.0742610;
  ret *= x;
  ret -= 0.2121144;
  ret *= x;
  ret += 1.5707288;
  ret *= sqrt(1.0 - x);
  ret = ret - 2.0 * negate * ret;
  return negate * 3.14159265358979 + ret;
} */
//function inverse_arm_kinematics (c=[0,10,0],lenAB=100,lenBC=120) = 
    //  ASSUMES THAT c is on the YZ plane (x is ignored)
    //  ASSUMES that A is at [0,0,0]
    // returns an array with [A_angle,B_angle] where B_angle is ABC (not BC to horizontal)
//    let (vt = norm(c))  // vector length from A to C
//    let (sub_angle1 = atan2(c[2],c[1]))  // atan2 (Y,X)!
//    let (sub_angle2 = acos((vt*vt+lenAB*lenAB-(lenBC*lenBC))/(2*vt*lenAB)) )
    //echo(vt=vt,sub_angle1=sub_angle1,sub_angle2=sub_angle2)
//    [sub_angle1 + sub_angle2,acos((lenBC*lenBC+lenAB*lenAB-vt*vt)/(2*lenBC*lenAB))] ;
void inverse_arm_kinematics(float cx,float cy) {
  // calculate the angles given pt C ***Inverse Kinematics***
  // Assumes that joint A of the robot arm is at 0,0  
  float temp,c_len, sub_angle1, sub_angle2;
  c_len = sqrt(pow(cx,2)+pow(cy,2));
  sub_angle1 = atan2(cy,cx);
  sub_angle2 = acos((pow(c_len,2)+pow(lenAB,2)-pow(lenBC,2))/(2*c_len*lenAB));
  new_ang_a = sub_angle1 + sub_angle2;
  new_ang_b = acos((pow(lenBC,2)+pow(lenAB,2)-pow(c_len,2))/(2*lenBC*lenAB));
  return;  // no return value
}

void setup() {
  #if TUNE
    servo_tune.attach(SVODIG); 
  #else
    servoA.attach(SVODIG_A);
    servoB.attach(SVODIG_B);
    servoC.attach(SVODIG_C);
    servoD.attach(SVODIG_D);
    servoT.attach(SVODIG_T);
  #endif

  // SERIAL OUTPUT AFFECTS SMOOTHNESS OF SERVO PERFORMANCE 
  //  WITH SERIAL ON AND LOW BAUD RATE = JERKY PERFORMANCE
  //   WITH OFF OR HIGH 500000 BAUD RATE = SMOOTH PERFORMANCE
  #if SERIALOUT
    Serial.begin(9600); // baud rate, slower is easier to read
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    } 
  #endif
  
  millisTime = millis();

  priorT = 1800; // for rate limiting a servo, microseconds, middle range
  
  pot = 500;  // VALUE IN MID RANGE SO THE MIN AND MAX WORK  
  min_pot = 10000;  // USE FOR DETERMINING RANGE OF POT
  max_pot = 0;

  // TRYING TO GET A MORE CONSISTENT INITIALIZATION
  #if TUNE
    // Change servo positions using write command (degrees):
    //servo_tune.writeMicroseconds((SVO_MIN+SVO_MAX)/2);
  #else
    //servoA.writeMicroseconds((SVO_MIN_A+SVO_MAX_A)/2);
    //servoB.writeMicroseconds((SVO_MIN_B+SVO_MAX_B)/2);
    //servoC.writeMicroseconds((SVO_MIN_C+SVO_MAX_C)/2);
    //servoD.writeMicroseconds((SVO_MIN_D+SVO_MAX_D)/2);
    //servoT.writeMicroseconds((SVO_MIN_T+SVO_MAX_T)/2); 
  #endif    

}

void loop() {
  millisTime = millis();

  pot = analogRead(PIN);  // for tuning
  
  potA = analogRead(PIN_A);  // read joint A
  potB = analogRead(PIN_B);  // read joint B
  potC = analogRead(PIN_C);  // read joint C
  potD = analogRead(PIN_D);  // read the claw
  potT = analogRead(PIN_T);  // read the turntablee
  
  min_pot = min(min_pot,pot); // check for min value, for tuning
  max_pot = max(max_pot,pot); // check for max value, for tuning

  // Both map the pot value to degrees and constrain the angle, to prevent current overloads.
  // map(value, fromLow, fromHigh, toLow, toHigh)

  inputJoint = map(pot,POT_MIN,POT_MAX,INPUTARM_MIN*10,INPUTARM_MAX*10)/10.0;    // MAP POTENTIOMETER TO INPUT ANGLE
  outputJoint = constrain((inputJoint),OUTPUTARM_MIN,OUTPUTARM_MAX);       // CALCULATE OUTPUT ANGLE
  outputServo = map(outputJoint,OUTPUTARM_MIN,OUTPUTARM_MAX,SVO_MIN,SVO_MAX);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  
  inputA = map(potA,POT_MIN_A,POT_MAX_A,INPUTARM_MIN_A,INPUTARM_MAX_A);    
  outputA = constrain(inputA,OUTPUTARM_MIN_A,OUTPUTARM_MAX_A);
  inputB = map(potB,POT_MIN_B,POT_MAX_B,INPUTARM_MIN_B,INPUTARM_MAX_B);    
  outputB = outputA + inputB;
  outputB = constrain(outputB,OUTPUTARM_MIN_B,OUTPUTARM_MAX_B);
  // The C potentiometer is for tuning the Claw position.
  tuneCangle = constrain(potC,POT_MIN_C,POT_MAX_C);
  // want the claw to be always pointed down, except limit range to > -120 deg to prevent damage
  //outputC = -90 - outputB + tuneCangle+10.0;
  outputC = -90 - outputB;
  if (outputC < -100.0) { outputC = -100.0;  }
  inputD = map(potD,POT_MIN_D,POT_MAX_D,INPUTARM_MIN_D,INPUTARM_MAX_D);    
  inputT = map(potT,POT_MIN_T,POT_MAX_T,INPUTARM_MIN_T,INPUTARM_MAX_T);    
  outputT = constrain(inputT,OUTPUTARM_MIN_T,OUTPUTARM_MAX_T);
  
  output_servoA = map(outputA,OUTPUTARM_MIN_A,OUTPUTARM_MAX_A,SVO_MIN_A,SVO_MAX_A);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  output_servoB = map(outputB,OUTPUTARM_MIN_B,OUTPUTARM_MAX_B,SVO_MIN_B,SVO_MAX_B);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  output_servoC = map(outputC,OUTPUTARM_MIN_C,OUTPUTARM_MAX_C,SVO_MIN_C,SVO_MAX_C);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  output_servoD = map(inputD,OUTPUTARM_MIN_D,OUTPUTARM_MAX_D,SVO_MIN_D,SVO_MAX_D);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  output_servoT = map(outputT,OUTPUTARM_MIN_T,OUTPUTARM_MAX_T,SVO_MIN_T,SVO_MAX_T);  // MAP SERVO VALUE FROM OUTPUT ANGLE

  /* rate limit the Turntable servo
  deltaT = priorT-output_servoT;
  if (deltaT > SVO_RATE_LIMIT) { // greater positive than limit
    priorT = priorT - SVO_RATE_LIMIT;
  } else if (deltaT < -SVO_RATE_LIMIT) { // more negative than limit
    priorT = priorT + SVO_RATE_LIMIT;
  } else { // within the limit
    priorT = output_servoT;
  }
  */
  
  // calculate c arm positions from angles
  //cx = lenAB*cos(outputA*1000 / 57296) + lenBC*cos(outputB*1000 / 57296);
  cy = lenAB*sin(outputA*1000 / 57296) + lenBC*sin(outputB*1000 / 57296);
   
  /* IF C IS TOO LOW, MODIFY KINEMATICS TO PREVENT ARM FROM RUNNING INTO THE GROUND
  if (cy < 0) {
    inverse_arm_kinematics(cx,0.0);
   Serial.print(", new_ang_a,");
    Serial.print(new_ang_a*RADIAN);
    Serial.print(", new_ang_b,");
    Serial.print(new_ang_b*RADIAN);
  }
    //angles = (c[2] > A_Z_shift) ? [Arob,Brob] : inverse_arm_kinematics([0,c[1],0],lenAB=lenAB,lenBC=lenBC); 
        // calculate NEW b and c positions from angles
    //br2=[0,lenAB*cos(angles[0]),lenAB*sin(angles[0])];  // B relative location
    //b2=a+br2; // B absolute
    //ba2 = (c[2]>A_Z_shift) ? Brob : -(180-angles[0]-angles[1]);  // Angle of BC arm relative to horizontal
   //cr2 = [0,cos(ba2)*lenBC,sin(ba2)*lenBC];
    //c2=b2+cr2;  // C absolute
*/
  
  #if TUNE
    // Change servo positions using write command (degrees):
    servo_tune.writeMicroseconds(outputServo);
  #else
    servoA.writeMicroseconds(output_servoA);
    servoB.writeMicroseconds(output_servoB);
    servoC.writeMicroseconds(output_servoC);
    servoD.writeMicroseconds(output_servoD);
    servoT.writeMicroseconds(output_servoT);  // rate limited
  #endif    

  #if SERIALOUT
    // output for debugging
    // Serial.print(val,digits)
    Serial.print("millis,");
    Serial.print(millisTime);
    #if TUNE
      Serial.print(", PIN,");
      Serial.print(PIN);
      Serial.print(", pot,");
      Serial.print(pot);  
      Serial.print(", min_pot,");
      Serial.print(min_pot);
      Serial.print(", max_pot,");
      Serial.print(max_pot);
      Serial.print(", inputJoint,");
      Serial.print(inputJoint,1);
      Serial.print(", outputJoint,");
      Serial.print(outputJoint,1);
      Serial.print(", outputServo,");
      Serial.print(outputServo);
    #else
      Serial.print(", inputA,");
      Serial.print(inputA,1);
      Serial.print(", inputB,");
      Serial.print(inputB,1);
      Serial.print(", outputB,");
      Serial.print(outputB,1);
      Serial.print(", outputC,");
      Serial.print(outputC,1);
      Serial.print(", tuneC,");
      Serial.print(tuneCangle,1);
      Serial.print(", outputD,");
      Serial.print(inputD,1);
      Serial.print(", outputT,");
      Serial.print(outputT,1);
      Serial.print(", cy,");
      Serial.print(cy,1);
      if (outputC < -100.0) {
        Serial.print(", outputC<-100,");
        }
      if (cy < 0.0) {
        Serial.print(", cy<0,");
        }
    #endif  
    Serial.println(", END");
  #endif
}
