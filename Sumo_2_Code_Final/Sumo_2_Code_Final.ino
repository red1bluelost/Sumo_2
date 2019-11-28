/* Sumo_2_Code
 * Group 8
 * Cornerstone of Engineering 1
 */
#include <Servo.h>  // servo library  Note including this library diables PWM on pins 9, 10 Analogue writes will no longer
                     // work there but digital will
Servo servo_R;  //declare servo objects first -servo_R for right side and servo_L for left
Servo servo_L;

//struct for holding distance sensors pins and return values
struct DistanceSensor {
  const int trigPin;  //connects to the echo pin on the distance sensor
  const int echoPin;  //connects to the trigger pin on the distance sensor
  float distance;     //stores the distance measure by the distance sensor
};
//sets structs with specific pins for the Front, Left, and Right
DistanceSensor FrontDist {5,6,0};
DistanceSensor LeftDist {2,3,0};
DistanceSensor RightDist {4,7,0};

//constants for motor pins
const int rightMotor = 12;
const int leftMotor = 13;

//struct for LED sensors pin and value
struct LEDSensor {
  const int Sensor;   //Assign sensor pins for down sensor
  int sensorState;    //Sensor variable
};
//sets structs with specific pins for front right and left and back right and left
LEDSensor fLeftSensor {10,0};
LEDSensor fRightSensor {9,0};
LEDSensor bLeftSensor {8,0};
LEDSensor bRightSensor {11,0};

//bool used with functions to break loops
bool dontRepeat = false;  //for edge sensing causing a breaking in code

//bool to be used to only run the starting strategy on the first loop
bool startStrat;   //runs the starting strategy

//class that allow checking if a certain amount of time passed since calling the reset
class TimeTest {
  private:
  long int preTime;         //holds time when last event occured
  long int interval;        //holds value of the time interval that is being checked
  public:
  TimeTest();
  void SetIntvl(long int);  //sets the interval of time that needs to be checked
  bool TimePassTest();      //checks if enough time has pasted and returns a bool
  void ResetTime();         //resets the time since the chosen event has occured
};

//create instances of TimeTest for the evadeSide function
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
  servo_R.attach(rightMotor);           //connect Right motor single wire to Digital Port rightMotor
  servo_L.attach(leftMotor);            //connect Left motor single wire to Digital Port leftMotor

  //set up LED sensors
  pinMode(fLeftSensor.Sensor, INPUT);   // Declare assigned sensor pins as input
  pinMode(fRightSensor.Sensor, INPUT);
  pinMode(bLeftSensor.Sensor, INPUT);
  pinMode(bRightSensor.Sensor, INPUT);

  //starts from a stop
  congruentMove(0,true);

  //sets starting strategy to true so that it will happen in the first loop
  startStrat = true;

  //set TimeTest interval for evadeSide
  TTEvadeSide.SetIntvl(10000);
}

void loop() {
  startStrategy(startStrat);  //function holding our starting strategy
  spinAndScan(30);            //function holding the main spin, scan, and charge strategy
}

//------------------FUNCTIONS-------------------------------

//----Functions to be used in other functions---------------

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
void getDistance(DistanceSensor& sensor)    //passes the distance sensor struct by reference; function can be called on any distance sensor
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
  
  sensor.distance = calculatedDistance;               //send back the distance that was calculated

  Serial.print(sensor.distance);                      //print the distance that was measured
  Serial.println(" in");                              //print units after the distance
} //end getDistance(...)

//Input speed and direction to move wheels at same rate
void congruentMove(int moveSpeed, bool forward) {
  if(forward) { //move forward
    servo_R.write(90-moveSpeed);
    servo_L.write(90+moveSpeed);
  } else {  //move backward
    servo_R.write(90+moveSpeed);
    servo_L.write(90-moveSpeed);
  }
} //end congruentMove()

//keeps the most recent command running unless an evasive measure is invoked
void holdCommand() {
  //start infinite loop
  for(;;){
    //check if evasive measures have been invoked
    if (matSense() || evadeSide()){
      //breaks the infinite loop if an evasive measure was called
      break;
    }
  } //end infinite loop
} //end holdCommand()

//----Functions to be used for guiding robot actions--------

//Input desired distance (in) and speed (1-90) and direction to move straight for a chosen distance
void straightMove(float range, float moveSpeed, bool forward){ 
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
} //end straightMove(...)

//Input turn direction; if turning a specific direction add true and the angle (degrees)
void turnHold(char lORr, bool doAngle, float angle) {
  //determine how to set servos
  switch(lORr){
    case 'l': //turn left
      servo_L.write(0);
      servo_R.write(0);
      break;
    case 'r': //turn right
      servo_L.write(180);
      servo_R.write(180);
      break;
  }

  //perform a delay if an angle was called
  if(doAngle){  
    safeDelay((angle/360)*3350);  //delay for certain amount of degrees
    congruentMove(0,true); //stop after rotation finishes
  }
} //end turnHold(...)

//input curved movement direction and the rate to turn
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
} //end moveAndTurn(...)

//input the range to check; performs the main startegy to spin, scan, and charge
void spinAndScan(int range){
  //must be turning before this function starts
  do{   //scan loop
    //check for robot in front sensor
    getDistance(FrontDist);
    //break from scan loop if something is seen within range
    if (FrontDist.distance <= range){ //
      break;
    }

    //check side sensors
    getDistance(LeftDist);
    getDistance(RightDist);

    //changes turn directions if something is seen on that side
    if (LeftDist.distance < (range/.8)) {
      turnHold('l',false,0);
    } else if (RightDist.distance < (range/.8)) {
      turnHold('r',false,0);
    }

    //checks if the evasive manuvers were invoked
    if (matSense() || evadeSide()){  
      return;
    }
  }while(FrontDist.distance > range);   //keep repeating scan until something is seen within range

  //sets the robot to charge forward
  congruentMove(90,true);

  //hold charge until an evasive manuver is invoked
  holdCommand();
} //end spinAndScan(...)

//input bool holding the value if the starting strategy already occured; executes starting strategy if its the first time
void startStrategy(bool &firstTime){
  //check if it is first time to break or not
  if (!firstTime) {
    return;
  }

  //turn left to point at side edge
  turnHold('l',true, 55);

  //move to edge of the ring
  straightMove(16,90,'f');

  //turn right to start scanning
  turnHold('r',false,0);
  
  //sets bool to never repeat starting stragtegy
  firstTime = false;
} //end startStrategy(...)

//--Safety functions to be used to perform evasive measures-

//checks front edge sensors and perform evasive movement and returns a bool of whether edge was detected
bool edgeSense(){
  //disables the edgeSense() if something is seen within a close range
  /*getDistance(FrontDist);
  if (FrontDist.distance < 4) {
    return false;
  }*/
  
  //read front edge sensors
  fRightSensor.sensorState = digitalRead(fRightSensor.Sensor);  //read right sensor
  fLeftSensor.sensorState = digitalRead(fLeftSensor.Sensor);    //read left sensor
  
  //execute evasion pattern if something is sensed
  if(fRightSensor.sensorState == LOW){  //if the right sensor is triggered
    //back up
    congruentMove(90,false);
    delay(800);  //  .8 second delay
    //rotate away from the edge
    servo_L.write(0);  //left motor Clockwise for reverse
    servo_R.write(0);   //right motor clockwise forward
    return true;  //return that the edge was hit
  } else if(fLeftSensor.sensorState == LOW){
    //back up
    congruentMove(90,false);
    delay(800);  //  .8 second delay
    //rotate away from the edge
    servo_R.write(180);    // spin right motor backwards
    servo_L.write(180);    // spin Left motor forwards
    return true;  //return that the edge was hit
  } else { //do nothing if nothing is sensed
    return false; //return that the edge was not hit
  } //end if checks for front edge sensors
} //end edgeSense()

//checks back edge sensors and perform evasive movement and returns a bool of whether edge was detected
bool backSense(){
  //read back edge sensors
  bRightSensor.sensorState = digitalRead(bRightSensor.Sensor);  //read right sensor
  bLeftSensor.sensorState = digitalRead(bLeftSensor.Sensor);   //read left sensor
  
  //execute evasion pattern if something is sensed
  if(bRightSensor.sensorState == LOW){  //if the right sensor is triggered
    //move forward
    congruentMove(90,true);
    delay(500);  //  .5 second delay
    //rotate away from the edge
    servo_L.write(0);  //left motor Clockwise for reverse
    servo_R.write(0);   //right motor clockwise forward
    return true;  //return that the edge was hit
  } else if(bLeftSensor.sensorState == LOW) {
    //move foward
    congruentMove(90,true);
    delay(500);  //  .5 second delay
    //rotate away from the edge
    servo_R.write(180);    // spin right motor backwards
    servo_L.write(180);    // spin Left motor forwards
    return true;  //return that the edge was hit
  } else {  //do nothing if nothing is sensed
    return false; //return that the edge was not hit
  } //end if checks for back edge sensors
} //end backSense()

//calls both front and back edge sensor functions and returns if an edge was hit
bool matSense() {
  return edgeSense() || backSense();
}

//checks if something is close on the side and then performs evasive manuvers if so
bool evadeSide() {
  //check side distance sensors
  getDistance(LeftDist);
  getDistance(RightDist);

  //checks if enough time has passed since the previous time side evasion was performed
  if (TTEvadeSide.TimePassTest()){
    //if something near (< 3 inch) on left; if so performs evasive manuver
    if (LeftDist.distance < 3  && LeftDist.distance > 0.2){
      //pivots to the right on the left wheel
      servo_R.write(180);
      servo_L.write(90);
      delay(1600);

      //move backwards a short distance
      straightMove(6, 90, false);

      //start turning towards what has been evaded from
      turnHold('l',false,0);

      //reset the time test object because the event occured
      TTEvadeSide.ResetTime();
      return true;
    } //end of if for checking the left side
    //if something near (< 3 inch) on right; if so performs evasive manuver
    if (RightDist.distance < 3 && RightDist.distance > 0.2){
      //pivots to the left on the right wheel
      servo_R.write(90);
      servo_L.write(0);
      delay(1600);

      //move backwards a short distance
      straightMove(6, 90, false);

      //start turning towards what has been evaded from
      turnHold('r',false,0);

      //reset the time test object because the event occured
      TTEvadeSide.ResetTime();
      return true;
    } //end of if for checking the right side
  } //end the main side evasion if test for time past
  return false;
} //end evadeSide()

//delays the code like delay(int) but while constantly checking side and edge sensors
void safeDelay(long int intvl) {
  //sets value that delay will end
  long int timeTest;
  timeTest = intvl + millis();

  //loops until time has passed
  do{
    //checks all safety function and breaks if one is hit
    if (matSense() || evadeSide()){
      break;
    }
  }while(millis() < timeTest);  //checks if current time has reached the time when the delay will end
} //end safeDelay()

//----Time Test class methods-------------------------------

//constructor presets first time to zero
TimeTest::TimeTest() {
  preTime = 0;
}
//called in the setup main function to set the interval
void TimeTest::SetIntvl(long int i) {
  interval = i;
}
//checks if enough time has passed since the previous time event was called
bool TimeTest::TimePassTest(){
  if (millis() - interval < preTime) {
    return false;
  }
  return true;
}
//resets time value by setting preTime to the current time
void TimeTest::ResetTime() {
  preTime = millis();
}
