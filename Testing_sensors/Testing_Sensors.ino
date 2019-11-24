/* Sumo_2_Code
 * Group 8
 * Cornerstone of Engineering 1
 */
#include <Servo.h>  // servo library  Note including this library diables PWM on pins 9, 10 Analogue writes will no longer
                     // work there but digital will
Servo servo_R;  //declare servo objects first -servo_R for right side and servo_L for left
Servo servo_L;

//creates a struct to be used with the three distance sensors
struct DistanceSensor {
  const int trigPin;  //connects to the echo pin on the distance sensor
  const int echoPin;  //connects to the trigger pin on the distance sensor
  float distance;     //stores the distance measure by the distance sensor
};
//sets structs to hold pins for the Front, Left, and Right
DistanceSensor FrontDist {5,6,0};
DistanceSensor LeftDist {2,3,0};
DistanceSensor RightDist {4,7,0};

//constants for motor pins
const int rightMotor = 12;
const int leftMotor = 13;

//creates a struct for the LED sensor pin and value
struct LEDSensor {
  const int Sensor;   //Assign sensor pins for down sensor
  int sensorState;    //Sensor variable
};
//sets structs to hold pins and variables for Front right and left and back right and left
LEDSensor fLeftSensor {10,0};
LEDSensor fRightSensor {9,0};
LEDSensor bLeftSensor {8,0};
LEDSensor bRightSensor {11,0};

//bool used with functions to break loops
bool dontRepeat = false;  //for edge sensing causing a breaking in code

//bool to be used to only run the starting strategy on the first loop
bool startStrat;   //runs the starting strategy

//class to create objects that allow checking if a certain amount of time passed
class TimeTest {
  private:
  long int preTime;
  long int interval;
  public:
  TimeTest();
  void SetIntvl(long int);
  bool TimePassTest();
  void ResetTime();
};

//create instances of TimeTest for testing if time has pas
TimeTest TTEvadeSide;

void setup() {
  Serial.begin (9600);        //set up a serial connection with the computer

  //set up distance sensors
  pinMode(FrontDist.trigPin, OUTPUT);   //the trigger pin will output pulses of electricity 
  pinMode(FrontDist.echoPin, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor
  pinMode(LeftDist.trigPin, OUTPUT);
  pinMode(LeftDist.echoPin, INPUT);
  pinMode(RightDist.trigPin, OUTPUT);
  pinMode(RightDist.echoPin, INPUT);

  //set up motors
  servo_R.attach(rightMotor);  //connect Right motor single wire to Digital Port rightMotor
  servo_L.attach(leftMotor);  //connect Left motor single wire to Digital Port leftMotor

  //set up LED sensors
  pinMode(fLeftSensor.Sensor, INPUT);   // Declare assigned sensor pins as input
  pinMode(fRightSensor.Sensor, INPUT);
  pinMode(bLeftSensor.Sensor, INPUT);
  pinMode(bRightSensor.Sensor, INPUT);

  //starts from a stop
  congruentMove(0,true);

  //do starting strategy only after set up
  startStrat = true;

  //set TimeTest intervals
  TTEvadeSide.SetIntvl(10000);
}

void loop() {
  startStrategy(startStrat);
  spinAndScan(24);
  //congruentMove(90,true);
}

//------------------FUNCTIONS-------------------------------

//----Functions to be used in other functions---------------

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
void getDistance(DistanceSensor& sensor)
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time
  
  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(sensor.trigPin, LOW);

  echoTime = pulseIn(sensor.echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor

  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  calculatedDistance = 0.9831*calculatedDistance + 0.14; //change to actual distance based on Excel Data Analysis
  
  sensor.distance = calculatedDistance;              //send back the distance that was calculated

  Serial.print(sensor.distance);     //print the distance that was measured
  Serial.println(" in");      //print units after the distance
}

//function to use when writing to the motors to move at the same speed
void congruentMove(int moveSpeed, bool forward) {
  if(forward) { //move forward
    servo_R.write(90-moveSpeed);
    servo_L.write(90+moveSpeed);
  }
  else {  //move backward
    servo_R.write(90+moveSpeed);
    servo_L.write(90-moveSpeed);
  }
} //end congruentMove()

//----Functions to be used for guiding robot actions----------------------------------

//Input desired distance (in) and speed 1-90 and direction
void straightMove(float range, float moveSpeed, bool forward){ //input range (inch), speed (1-90), and f or b for direction

  //calculate time needed for a chosen distance at a chosen speed
  float distanceFromRange;
  if(moveSpeed <= 10) //relation between movespeed and how far it travels when less than 10
    distanceFromRange = range * (5000/(3.4179 * moveSpeed + 2.4821));
  else  //speed doesn't matter when more than 10ish
    distanceFromRange = (range * (5000/32.5));

  //set motors to move at the speed
  congruentMove(moveSpeed, forward);

  //move for certain time
  safeDelay(distanceFromRange);
}

//a function to turn the robot and with the option of rotating for a specific angle of rotation
void turnHold(char lORr, bool doAngle, float angle) { //turn left or right, if doAngle how much of an angle (degrees)
  switch(lORr){ //determine how to set servos
    case 'l': //turn left
      servo_L.write(0);
      servo_R.write(0);
      break;
    case 'r': //turn right
      servo_L.write(180);
      servo_R.write(180);
      break;
  }
  if(doAngle){  //turn for certain amount of degrees
    safeDelay((angle/360)*3350);
    congruentMove(0,true); //stop after rotation finishes
  }
}

//funtion to move with a curve
void moveAndTurn(char lORr, float turnRate){ //curved movement, turnRate 1-180 for rate of turn
  switch(lORr){
    case 'l': //turn left
      servo_L.write(100-turnRate);
      servo_R.write(0);
      break;
    case 'r': //turn right
      servo_L.write(180);
      servo_R.write(80+turnRate);
      break;
  }
}

void spinAndScan(int range){
    do{     //spin and scan loop
    getDistance(FrontDist);  //check for robot in front sensor
    if (FrontDist.distance <= range){
      break;
    }
    getDistance(LeftDist);
    getDistance(RightDist);
    if (LeftDist.distance < range) {
      turnHold('l',false,0);
    } else if (RightDist.distance < range) {
      turnHold('r',false,0);
    }
    
    if (matSense() || evadeSide()){  //make sure not to run off the edge of the mat
      return;
    }
  }while(FrontDist.distance > range);   //keep scanning until something is seen within 24 inches
  congruentMove(90,true);
  holdCommand();
}

//Strategy plays out only on the first run
void startStrategy(bool &firstTime){
  if (!firstTime) {  //check if it is first time to break or not
    return;
  }
  turnHold('l',true, 55); //turn left
  straightMove(16,90,'f');  //move to edge of the ring
  turnHold('r',false,0);  //turn right to start scanning
  firstTime = false;  //never repeat starting stragtegy
}

//function to keep the most recent command running unless an evasive measure is necessary
void holdCommand() {
  for(;;){
    if (matSense() || evadeSide()){  //make sure not to run off the edge of the mat
      break;
    }
  }
}

//----Safety functions to be used to perform evasive measures----------------------------

//function to check edge sensors and perform evasion movement and tells code if it was sensed
bool edgeSense(){
  getDistance(FrontDist);
  if (FrontDist.distance < 4 || FrontDist.distance > 900) {
    return false;
  }
  
  //read sensors
  fRightSensor.sensorState = digitalRead(fRightSensor.Sensor);  //read right sensor
  fLeftSensor.sensorState = digitalRead(fLeftSensor.Sensor);   //read left sensor
  
  //execute evasion pattern if something is sensed
  if(fRightSensor.sensorState == LOW){  //if the right sensor is triggered
    //back up
    congruentMove(90,false);
    delay(500);  //  .5 second delay
    //rotate away from the edge
    servo_L.write(0);  //left motor Clockwise for reverse
    servo_R.write(0);   //right motor clockwise forward
    //delay(700);  //  .7 second delay to turn far enough
    return true;  //return that the edge was hit
  }
  else if(fLeftSensor.sensorState == LOW){
    //back up
    congruentMove(90,false);
    delay(500);  //  .5 second delay
    //rotate away from the edge
    servo_R.write(180);    // spin right motor backwards
    servo_L.write(180);    // spin Left motor forwards
    //delay(700);  //  .25 second delay
    return true;  //return that the edge was hit
  }
  else  //do nothing if nothing is sensed
    return false; //return that the edge was not hit
}

//function to check back sensors and perform evasion movement and tells code if it was sensed
bool backSense(){
  //read sensors
  bRightSensor.sensorState = digitalRead(bRightSensor.Sensor);  //read right sensor
  bLeftSensor.sensorState = digitalRead(bLeftSensor.Sensor);   //read left sensor
  
  //execute evasion pattern if something is sensed
  if(bRightSensor.sensorState == LOW){  //if the right sensor is triggered
    //back up
    congruentMove(90,true);
    delay(500);  //  .5 second delay
    //rotate away from the edge
    servo_L.write(0);  //left motor Clockwise for reverse
    servo_R.write(0);   //right motor clockwise forward
    //delay(700);  //  .7 second delay to turn far enough
    return true;  //return that the edge was hit
  }
  else if(bLeftSensor.sensorState == LOW){
    //back up
    congruentMove(90,true);
    delay(500);  //  .5 second delay
    //rotate away from the edge
    servo_R.write(180);    // spin right motor backwards
    servo_L.write(180);    // spin Left motor forwards
    //delay(700);  //  .25 second delay
    return true;  //return that the edge was hit
  }
  else  //do nothing if nothing is sensed
    return false; //return that the edge was not hit
}

bool matSense() {
  return edgeSense() || backSense();
}

bool evadeSide() {
  getDistance(LeftDist);
  getDistance(RightDist);
  if (TTEvadeSide.TimePassTest()){
    if (LeftDist.distance < 3  && LeftDist.distance > 0.2){ //if something near (< 3 inch) on left do evasive manuver
      servo_R.write(180);
      servo_L.write(90);
      delay(1600);
      straightMove(6, 90, false);
      turnHold('l',false,0);
      TTEvadeSide.ResetTime();
      return true;
    }
    if (RightDist.distance < 3 && RightDist.distance > 0.2){ //if something near (< 3 inch) on right do evasive manuver
      servo_R.write(90);
      servo_L.write(0);
      delay(1600);
      straightMove(6, 90, false);
      TTEvadeSide.ResetTime();
      turnHold('r',false,0);
      return true;
    }
  }
  return false;
}

void safeDelay(long int intvl) {
  long int timeTest, currentTime;
  timeTest = intvl + millis();
  do{
    currentTime = millis();
    if (matSense() || evadeSide()){
      break;
    }
  }while(currentTime < timeTest);
}

//----Time Test class methods---------------------------------

TimeTest::TimeTest() {
  preTime = 0;
}
void TimeTest::SetIntvl(long int i) {
  interval = i;
}
bool TimeTest::TimePassTest(){
  if (millis() - interval < preTime) {
    return false;
  }
  return true;
}
void TimeTest::ResetTime() {
  preTime = millis();
}
