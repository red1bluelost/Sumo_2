/* Sumo_1_Code
 * Micah Weston and Kelli Vallentini
 * Cornerstone of Engineering 1
 * Professor Richard Whalen 
 */
#include <Servo.h>  // servo library  Note including this library diables PWM on pins 9, 10 Analogue writes will no longer
                     // work there but digital will
Servo servo_R;  //declare servo objects first -servo_R for right side and servo_L for left
Servo servo_L;

//constants for the FRONT facing distance sensor
const int trigPin = 5;           //connects to the echo pin on the distance sensor       
const int echoPin = 6;           //connects to the trigger pin on the distance sensor     
//constants for the LEFT facing distance sensor
const int LEFTtrigPin = 2;           //connects to the echo pin on the distance sensor       
const int LEFTechoPin = 3;           //connects to the trigger pin on the distance sensor
//constants for the RIGHT facing distance sensor
const int RIGHTtrigPin = 4;           //connects to the echo pin on the distance sensor       
const int RIGHTechoPin = 7;           //connects to the trigger pin on the distance sensor

//variable to hold the distance from the FRONT sensor
float distanceFront = 0;               //stores the distance measured by the distance sensor
//variable to hold the distance from the LEFT sensor
float distanceLEFT = 0;               //stores the distance measured by the distance sensor
//variable to hold the distance from the RIGHT sensor
float distanceRIGHT = 0;               //stores the distance measured by the distance sensor

//constants for motor pins
const int rightMotor = 12;
const int leftMotor = 13;

//constants for the FRONT side sensors
const int leftSensor = 10;     // Assign sensor pins for two down sensors
const int rightSensor = 9;
//variable for the FRONT sensor states
int lsensorState = 0;          // Initialize sensor variable 
int rsensorState = 0;
//constants for the BACK side sensors
const int BACKleftSensor = 8;     // Assign sensor pins for two down sensors
const int BACKrightSensor = 11;
//variable for the BACK sensor states
int BACKlsensorState = 0;          // Initialize sensor variable 
int BACKrsensorState = 0;

//bool used with functions to break loops
bool dontRepeat = false;  //for edge sensing causing a breaking in code

//bool to be used to only run the starting strategy on the first loop
bool startStrat;   //runs the starting strategy

void setup() {
  Serial.begin (9600);        //set up a serial connection with the computer

  //set up front sensor
  pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity 
  pinMode(echoPin, INPUT);    //the echo pin will measure the duration of pulses coming back from the distance sensor

  //set up motors
  servo_R.attach(rightMotor);  //connect Right motor single wire to Digital Port rightMotor
  servo_L.attach(leftMotor);  //connect Left motor single wire to Digital Port leftMotor

  //set up sensors
  pinMode(leftSensor, INPUT);   // Declare assigned sensor pins as input
  pinMode(rightSensor, INPUT);

  //starts from a stop
  servo_R.write(90);
  servo_L.write(90);

  //do starting strategy only after set up
  startStrat = true;
}

void loop() {
  //Strategy plays out only on the first run
  if(startStrat){

    //0.1 Turn left
    turnHold('l',true,55);

    //0.2 move to edge of the ring
    straightMove(16,90,'f');
  
    //0.3 Turn right and start scanning
    turnHold('r',false,0);
    delay(750); //don't scan off map on first run
    startStrat = false; //never repeat starting strategy
  }
  //Starting point for all loops after first

  //1.0 spin and scan for another robot
  do{     //spin and scan loop
    distanceFront = getDistance();  //check for robot in front sensor
    Serial.print(distanceFront);     //print the distance that was measured
    Serial.println(" in");      //print units after the distance
    edgeSense();  //make sure not to run off the edge of the mat
  }while(distanceFront > 20);   //keep scanning until something is seen within 24 inches
  
  //2.0 If something is seen within 24 inches, charge
  charge(90,'f');

  //3.0 loop so the charge does not stop
  do{
    dontRepeat = edgeSense(); //make sure he does not run off the map
  }while(!dontRepeat);  //keep charging if edgeSense doesn't see the edge
  //go back to the beginning of the loop if the edge was hit while charging at a robot
}

//------------------FUNCTIONS-------------------------------

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time
  
  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor

  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  calculatedDistance = 0.9831*calculatedDistance + 0.14; //change to actual distance based on Excel Data Analysis
  
  return calculatedDistance;              //send back the distance that was calculated
}


//Input desired distance (in) and speed 1-90 and direction
void straightMove(float range, float moveSpeed, char goOrBack){ //input range (inch), speed (1-90), and f or b for direction

  //calculate time needed for a chosen distance at a chosen speed
  float distanceFromRange;
  if(moveSpeed <= 10) //relation between movespeed and how far it travels when less than 10
    distanceFromRange = range * (5000/(3.4179 * moveSpeed + 2.4821));
  else  //speed doesn't matter when more than 10ish
    distanceFromRange = (range * (5000/32.5));

  //set motors to move at the speed
  switch(goOrBack){
    case 'f': //move forward
    servo_R.write(90 - moveSpeed);
    servo_L.write(90 + moveSpeed);
    break;
    case 'b': //move back
    servo_R.write(90 + moveSpeed);
    servo_L.write(90 - moveSpeed);
    break;
  } //end switch to move forward or back at a certain speed

  //move for certain time
  delay(distanceFromRange);
}

void charge(float moveSpeed, char goOrBack){ //set to move without any delays
  //set motors to move at the speed
  switch(goOrBack){
    case 'f': //move forward
    servo_R.write(90 - moveSpeed);
    servo_L.write(90 + moveSpeed);
    break;
    case 'b': //move back
    servo_R.write(90 + moveSpeed);
    servo_L.write(90 - moveSpeed);
    break;
  } //end switch to move forward or back at a certain speed
}

//a function to just stop movement
void stopMove(){
  //stop
  servo_R.write(90);
  servo_L.write(90); 
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
    delay((angle/360)*3350);
    stopMove(); //stop after rotation finishes
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

//function to check edge sensors and perform evasion movement and tells code if it was sensed
bool edgeSense(){
  //read sensors
  rsensorState = digitalRead(rightSensor);  //read right sensor
  lsensorState = digitalRead(leftSensor);   //read left sensor
  
  //execute evasion pattern if something is sensed
  if(rsensorState == LOW){  //if the right sensor is triggered
    //back up
    servo_R.write(180); //right motor counterclockwise reverse
    servo_L.write(0);  //left motor Clockwise for reverse
    delay(500);  //  .5 second delay
    //rotate away from the edge
    servo_L.write(0);  //left motor Clockwise for reverse
    servo_R.write(0);   //right motor clockwise forward
    //delay(700);  //  .7 second delay to turn far enough
    Serial.println("Right Sensor");
    return true;  //return that the edge was hit
  }
  else if(lsensorState == LOW){
    //back up
    servo_R.write(180); //right motor counterclockwise reverse
    servo_L.write(0);  //left motor Clockwise for reverse
    delay(500);  //  .5 second delay
    //rotate away from the edge
    servo_R.write(180);    // spin right motor backwards
    servo_L.write(180);    // spin Left motor forwards
    //delay(700);  //  .25 second delay
    Serial.println("Left Sensor") ;
    return true;  //return that the edge was hit
  }
  else  //do nothing if nothing is sensed
    return false; //return that the edge was not hit
}
