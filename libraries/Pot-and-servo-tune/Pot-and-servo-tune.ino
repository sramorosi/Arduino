/* ROBOT ARM CONTROL SOFTWARE
 *  By, SrAmo, February 2022
 */
#include <Servo.h>  // servo library
// Servo Function library at: http://arduino.cc/en/Reference/Servo

#define SERIALOUT false  // COMMENT-OUT TO TURN OFF SERIAL AND MAKE SMOOTH!

#define TUNE false  // TRUE WHEN TUNING ONE POT/SERVO,  FALSE WHEN DOING ALL SERVOS

#define lenAB 50.0     // Length of Input AB arm in mm
#define lenBC 60.0     // Length of Input BC arm in mm

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
#define PIN 1    // TEST PIN IS ZERO
#define POT_MIN 0   // 0 TO 1023
#define POT_MAX 957   // 0 TO 1023
#define INPUTARM_MIN -125  // DEG
#define INPUTARM_MAX 120  // DEG
#define SVODIG 5
#define OUTPUTARM_MIN -125  // DEG
#define OUTPUTARM_MAX 120  // DEG
#define SVO_MIN 2210   // microseconds @ OUTPUTARM_MIN angle
#define SVO_MAX 830  // microseconds @ OUTPUTARM_MAX angle

// JOINT B, Analog 1 input pin, Digital 5 output pin
#define PIN_B 1
#define POT_MIN_B  0
#define POT_MAX_B  957
#define INPUTARM_MIN_B  -125
#define INPUTARM_MAX_B  120
#define SVODIG_B 5
#define OUTPUTARM_MIN_B -125  // DEG
#define OUTPUTARM_MAX_B 120  // DEG
#define SVO_MIN_B 2210   // microseconds
#define SVO_MAX_B 830  // microseconds

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

// JOINT A, Analog 4 input pin, Digital 9 output
#define PIN_A 4
#define POT_MIN_A 134
#define POT_MAX_A 870
#define INPUTARM_MIN_A  0
#define INPUTARM_MAX_A  175
#define SVODIG_A 9
#define OUTPUTARM_MIN_A 0  // DEG
#define OUTPUTARM_MAX_A 140  // DEG
#define SVO_MIN_A 960   // microseconds
#define SVO_MAX_A 1940  // microseconds

// JOINT C, COMPUTED INPUT.
//  TUNER POT IN THE FUTURE, Analog 5 input pin, Digital 6 output pin
#define PIN_C 5    // 
#define POT_MIN_C 130   // 0 TO 1023
#define POT_MAX_C 910   // 0 TO 1023
#define INPUTARM_MIN_C -110  // DEG CW
#define INPUTARM_MAX_C 40  // DEG CCW
#define SVODIG_C 6
#define OUTPUTARM_MIN_C -110  // NEED TO UPGRADE TO 270 DEG SERVO
#define OUTPUTARM_MAX_C 0
#define SVO_MIN_C 2330   // microseconds
#define SVO_MAX_C 350  // microseconds

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

Servo servo_tune;  // servo for tuning

Servo servoA,servoB,servoC,servoD,servoT;  // servos for robot arm

// Use these constants to control the range of the servo outputs

int pot; // pot value for tuning
int min_pot,max_pot;  // stores min and max potentiometer reading
double inputJoint;
double outputJoint;
int outputServo;

int potA,potB,potD,potT; // pot values for input arm

double priorT,deltaT;  // stores the computed value for the servo

double inputA,inputB,inputD,inputT;
double outputA,outputB,outputAB,outputC,outputD,outputT;
int output_servoA,output_servoB,output_servoC,output_servoD,output_servoT;

unsigned long millisTime;

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
    servo_tune.writeMicroseconds((SVO_MIN+SVO_MAX)/2);
  #else
    servoA.writeMicroseconds((SVO_MIN_A+SVO_MAX_A)/2);
    servoB.writeMicroseconds((SVO_MIN_B+SVO_MAX_B)/2);
    //servoC.writeMicroseconds((SVO_MIN_C+SVO_MAX_C)/2);
    servoD.writeMicroseconds((SVO_MIN_D+SVO_MAX_D)/2);
    servoT.writeMicroseconds((SVO_MIN_T+SVO_MAX_T)/2); 
  #endif    

}

void loop() {
  millisTime = millis();

  pot = analogRead(PIN);  // for tuning
  
  potA = analogRead(PIN_A);  // read joint A
  potB = analogRead(PIN_B);  // read joint B
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
  inputD = map(potD,POT_MIN_D,POT_MAX_D,INPUTARM_MIN_D,INPUTARM_MAX_D);    
  inputT = map(potT,POT_MIN_T,POT_MAX_T,INPUTARM_MIN_T,INPUTARM_MAX_T);    
  outputT = constrain(inputT,OUTPUTARM_MIN_T,OUTPUTARM_MAX_T);

  outputB = outputA + inputB;
  outputB = constrain(outputB,OUTPUTARM_MIN_B,OUTPUTARM_MAX_B);
  //outputC = constrain((-90 - outputB),OUTPUTARM_MIN_C,OUTPUTARM_MAX_C);  // want the hand to be always pointed down, within range of -120 to 120
  //     Crob = (Brob > 45) ? Brob - 135 : -90; // limit the claw from contacting the arm
  outputC = -90 -outputB;
  if (outputC < -120.0) {
    outputC = -120.0;
  }
  //outputC = -outputC;
  
  output_servoA = map(outputA,OUTPUTARM_MIN_A,OUTPUTARM_MAX_A,SVO_MIN_A,SVO_MAX_A);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  output_servoB = map(outputB,OUTPUTARM_MIN_B,OUTPUTARM_MAX_B,SVO_MIN_B,SVO_MAX_B);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  output_servoC = map(outputC,OUTPUTARM_MIN_C,OUTPUTARM_MAX_C,SVO_MIN_C,SVO_MAX_C);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  output_servoD = map(inputD,OUTPUTARM_MIN_D,OUTPUTARM_MAX_D,SVO_MIN_D,SVO_MAX_D);  // MAP SERVO VALUE FROM OUTPUT ANGLE
  output_servoT = map(outputT,OUTPUTARM_MIN_T,OUTPUTARM_MAX_T,SVO_MIN_T,SVO_MAX_T);  // MAP SERVO VALUE FROM OUTPUT ANGLE

  // rate limit the Turntable servo
  deltaT = priorT-output_servoT;
  if (deltaT > SVO_RATE_LIMIT) { // greater positive than limit
    priorT = priorT - SVO_RATE_LIMIT;
  } else if (deltaT < -SVO_RATE_LIMIT) { // more negative than limit
    priorT = priorT + SVO_RATE_LIMIT;
  } else { // within the limit
    priorT = output_servoT;
  }
  
    // calculate b and c positions from angles
    //br=[0,lenAB*cos(Arob),lenAB*sin(Arob)];  // B relative location
    //b=a+br; // B absolute
    //cr = [0,cos(Brob)*lenBC,sin(Brob)*lenBC];
    //c=b+cr;  // C absolute
    
    // IF C IS TOO LOW, MODIFY KINEMATICS TO PREVENT ARM FROM RUNNING INTO THE GROUND
    //angles = (c[2] > A_Z_shift) ? [Arob,Brob] : inverse_arm_kinematics([0,c[1],0],lenAB=lenAB,lenBC=lenBC); 
        // calculate NEW b and c positions from angles
    //br2=[0,lenAB*cos(angles[0]),lenAB*sin(angles[0])];  // B relative location
    //b2=a+br2; // B absolute
    //ba2 = (c[2]>A_Z_shift) ? Brob : -(180-angles[0]-angles[1]);  // Angle of BC arm relative to horizontal
   //cr2 = [0,cos(ba2)*lenBC,sin(ba2)*lenBC];
    //c2=b2+cr2;  // C absolute

  
  #if TUNE
    // Change servo positions using write command (degrees):
    servo_tune.writeMicroseconds(outputServo);
  #else
    servoA.writeMicroseconds(output_servoA);
    servoB.writeMicroseconds(output_servoB);
    //servoC.writeMicroseconds(output_servoC);
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
      Serial.print(inputA);
      Serial.print(", inputB,");
      Serial.print(inputB);
      Serial.print(", outputB,");
      Serial.print(outputB);
      Serial.print(", outputC,");
      Serial.print(outputC);
      Serial.print(", outputD,");
      Serial.print(inputD);
      Serial.print(", outputT,");
      Serial.print(outputT);
    #endif  
    Serial.println("     END");
  #endif
}
//function inverse_arm_kinematics (c=[0,10,0],lenAB=100,lenBC=120) = 
    // calculate the angles given pt C ***Inverse Kinematics***
    //  ASSUMES THAT c is on the YZ plane (x is ignored)
    //  ASSUMES that A is at [0,0,0]
    // returns an array with [A_angle,B_angle] where B_angle is ABC (not BC to horizontal)
//    let (vt = norm(c))  // vector length from A to C
//    let (sub_angle1 = atan2(c[2],c[1]))  // atan2 (Y,X)!
//    let (sub_angle2 = acos((vt*vt+lenAB*lenAB-(lenBC*lenBC))/(2*vt*lenAB)) )
    //echo(vt=vt,sub_angle1=sub_angle1,sub_angle2=sub_angle2)
//    [sub_angle1 + sub_angle2,acos((lenBC*lenBC+lenAB*lenAB-vt*vt)/(2*lenBC*lenAB))] ;
