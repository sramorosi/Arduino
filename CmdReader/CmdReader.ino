#define SIZE_CMD_ARRAY 6

struct command {
  int arg [SIZE_CMD_ARRAY];
};

struct commandS {  
  int num_cmds;
  command cmds[];
};

void setup() {
  // setup code here, to run once:
 Serial.begin(115200);
}

command getCmdSerial() { // read command from serial port
  // If cmd.arg[0] = 0 then command has NOT been read
  command cmd;
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
  return(cmd); 
}

void loop() {
  static command cmd;
  static commandS cmdS;
  int i,j;
  // put your main code here, to run repeatedly:
  cmd = getCmdSerial();
   
  cmdS.cmds[0] = cmd;
  cmdS.cmds[1] = cmd;
  
  if (cmdS.cmds[0].arg[0] != 0) for (i=0;i<2;i++) {
    for (j=0;j<SIZE_CMD_ARRAY;j++) {
      Serial.print(cmdS.cmds[i].arg[j]);
      Serial.print(",");
    }
    Serial.println("END");
  }
}
