

//counting program
// 6 ticks = 1 revolution
// count how many revs to reach top 
# define maxRev 5 //change value to correct one once tested
int motorDirection = 0;
int revolutionCount = 0;
int tickCount = 0;
int lastSwitchState =0;

//pins for motor driver
int enA = 5;
int in1 = 5;
int in2 = 23;
//random numbers 
int sensorPin = 25; //pin for limit switch

void setup() {
  // set the pin modes and initial rotation direction of motor

  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  //set the inital rotation direction - turn on motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  //for PMW maximum possible vlaues are 0 to 255
  ledcAttachPin(enA, 255);  //alter speed value to approperiate value
  Serial.begin(9600);

}

void loop() {
  // function counts each time limit switch is pressed (HIGH) and increments varial tickCount, for each 6 tickCounts variable revalutionCount is incremented
  //first if statement: each time limit switch is depressed increment counter by 1
  if(digitalRead(sensorPin)== LOW && lastSwitchState ==0 ){
      lastSwitchState = 1;      
      tickCount++;  
      } 
    if(digitalRead(sensorPin)==HIGH && lastSwitchState ==1) {
      lastSwitchState = 0;
    }

Serial.println(tickCount);
      
  //second if statement if counter = 6, increment revCounter by 1 and reset tickCounter
  if(tickCount == 6){
    revolutionCount ++;
    tickCount = 0;
  }
  //third if statement if revCounter = maxRev stop motor, and then change direction of rotation to degin the descent of robot
  if(revolutionCount == maxRev){

    //stop motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);

   //go down
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  }
}


  
