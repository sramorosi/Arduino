#define MAXMV 680  // 680 FOR 3.3 V,  1023 FOR 5 V
#define OFFSET 10 // to smooth blips between pots
const float LOWMV = 0.22*MAXMV;
const float HIGHMV = 0.78*MAXMV;
const float MIDMV = 0.5*MAXMV;

float valmv;
float deg;

unsigned long millisTime;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
}

void loop() {
  int val1 = analogRead(A0); // pot 1
  int val2 = analogRead(A1); // pot 2. two should follow 1 in the curves
  millisTime = millis();

  if (val1>val2) {
    if ((val2>LOWMV) && (val2 < HIGHMV)) {
       valmv = 2*MAXMV - val2 + OFFSET;
    } else {
      valmv = 2*MAXMV - val1 + MIDMV + OFFSET;
    }
  } else {
    if ((val2>LOWMV) && (val2 < HIGHMV)) {
       valmv = val2 - OFFSET;
    } else {
      valmv = val1 + MIDMV + OFFSET;
    }
  }
  valmv = valmv - MIDMV/2; // important offset

  deg = map(valmv,0,2*MAXMV,0,3600)/10.0;
  
  // output for debugging
  // Serial.print(val,digits)
  Serial.print("millis,");
  Serial.print(millisTime);
  Serial.print(", pot_value1,");
  Serial.print(val1);
  Serial.print(", pot_value2,");
  Serial.print(val2);
  Serial.print(", valmv,");
  Serial.print(valmv,1);
  Serial.print(", deg,");
  Serial.print(deg,1);

  Serial.println(", END");
  
}
