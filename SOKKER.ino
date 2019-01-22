unsigned long pingTimer;
int pingSpeed;
int state = 0;
//Distance sensors (4)
#include<NewPing.h>
const int trigPin = 6;
const int echoPin1 = 2;
const int echoPin2 = 3;
const int echoPin3 = 4;
const int echoPin4 = 5;

int bufferBack = 1;

int MAX_DISTANCE = 400;
NewPing sonar[4] = {NewPing(trigPin, echoPin1, MAX_DISTANCE),
  NewPing(trigPin, echoPin2, MAX_DISTANCE),
  NewPing(trigPin, echoPin3, MAX_DISTANCE),
  NewPing(trigPin, echoPin4, MAX_DISTANCE)
  };
int groundDist = 6;
int threatDist = 100;

//Servos (2)
#include <Servo.h> 
int servoPin1 = 11;
int servoPin2 = 10; 
Servo servo1;
Servo servo2; 
//91 to 96 = no motion for servo
//60 to 90 and 97 to 120
int noMotion = 93;
int minC = 97;
int maxC = 120;
int cRange = maxC - minC;
int minCC = 90;
int maxCC = 60;
int cCRange = maxCC - minCC; 

//Accelerometer (1)
int scale = 3; //measure Â±3g
int rawX, rawY, rawZ;
float scaledX, scaledY, scaledZ; // Scaled values for each axis

void setup() { 
  Serial.begin(9600);
  servo1.attach(servoPin1); 
  servo2.attach(servoPin2);
  delay(1500);
}//end setup

void loop() {
  //sorted list of sonar reading medians
  int decisions[3] = {0, 0, 0};
  int decision;
  int distances[4];
  int distancesSorted[4];
  int distanceData[5]; //used for finding median
  for(int i = 0; i < 5; i++){
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 5; j++){
        distanceData[j] = sonar[i].ping_cm();
        delay(10);
      }
      qsort(distanceData, sizeof(distanceData)/sizeof(distanceData[0]), sizeof(distanceData[0]), sort_desc); //sorted in dec order
      distances[i] = distanceData[2];
      distancesSorted[i] = distanceData[2];
    }
    qsort(distancesSorted, sizeof(distancesSorted)/sizeof(distancesSorted[0]), sizeof(distancesSorted[0]), sort_desc); //distancesSorted now sorted in dec order
  
    //Resort if one of the medians is 0 (extraneous readings)
    if(distancesSorted[3] == 0 && findDownAcc() != distancesSorted[3]) {
      distancesSorted[3] = 400;
      qsort(distancesSorted, sizeof(distancesSorted)/sizeof(distancesSorted[0]), sizeof(distancesSorted[0]), sort_desc); 
    }
    if(distancesSorted[2] <= threatDist && distancesSorted[1] <= threatDist)//robot is surrounded on 2 sides
      decisions[2]++;  
    else if(distancesSorted[2] < threatDist) //sees human or wall; just blocked in 1 direction
      decisions[1]++;
    else
      decisions[0]++;  
  }//end for
  
  if(decisions[1] > decisions[0] && decisions[1] > decisions[2])
    decision = 1; 
  else if(decisions[2] > decisions[0] && decisions[2] > decisions[1])
    decision = 2;  
  else
    decision = 0;
  for(int k = 0; k < 3; k++)
    decisions[k] = 0;     
  Serial.println(decision);     
  //decide what to do
  //IMPRTANT: CHECK 0
 // if(distancesSorted[3] <= groundDist) { //robot is on ground
    int down = findDownAcc(); //getFace(distances, distancesSorted, 3);
    int back = getFace(distances, distancesSorted, 2);
    if(distancesSorted[2] <= threatDist)
      bufferBack = back;
    int lowSpeed, range;
    if(distancesSorted[2] <= groundDist)
      back = stuckOnWall(down, back); 
    
    if(down - back == 1 || down - back == -3) {
      lowSpeed = minC;
      range = cRange;
    }
    
    else {
      lowSpeed = minCC;
      range = cCRange;
    }

    //rotateRight(down, back);
    
    if(decision == 2) {//robot is surrounded on 2 sides
      rotateRight(down, back);       
      Serial.println("Surrounded on 2 sides and turning");
    }
    
    else if(decision == 1){ //sees human or wall; just blocked in 1 direction
      moveStraight(lowSpeed, range);  
      Serial.println("Surrounded on 1 sides and moving away");
    }

    else { //nothing nearby
      if(down - bufferBack == 1 || down - bufferBack == -3) {
        lowSpeed = minC;
        range = cRange;
      }
       
      else {
        lowSpeed = minCC;
        range = cCRange;
      }
      
      defaultMotion(lowSpeed, range);
      Serial.println("No threat detected. Default motion");
    }
  //}
}//end loop

//required method of quick sort
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  return a > b ? -1 : (a < b ? 1 : 0);
}

//EDITED: now requires down-back to be evaluated beforehand. Pass in minC/cRange or minCC/ccRange based on that value
void moveStraight(int lowSpeed, int range) {
    servo1.write(lowSpeed + 0.8 * range);
    servo2.write(lowSpeed + 0.8 * range);
}

void rotateRight(int down, int back) {
  if(down - back == 1 || down - back == -3) {
    servo2.write(minC + .6 * cRange);
    servo1.write(minCC + .6 * cCRange);
  }
  else {
    servo1.write(minC + .6 * cRange);
    servo2.write(minCC + .6 * cCRange);
  }
  pingTimer += 600;
}

void rotateLeft(int down, int back) {
  if(down - back == 1 || down - back == -3) {
    servo1.write(minC + .5 * cRange);
    servo2.write(minCC + .5 * cCRange);
  }
  else {
    servo2.write(minC + .5 * cRange);
    servo1.write(minCC + .5 * cCRange);
  }

  pingTimer += 200;
}


void turnRight(int lowSpeed, int range) {
  servo1.write(lowSpeed + .2 * range);
  servo2.write(lowSpeed + .8 * range);
  pingTimer += 600;
}


void turnLeft(int lowSpeed, int range) {
  servo1.write(lowSpeed + .8 * range);
  servo2.write(lowSpeed + .2 * range);
  pingTimer += 600;
}

void defaultMotion(int lowSpeed, int range) {
  if(millis() >= pingTimer) { //check if it's time to update
    switch (state) {
      //Are we turning "left"?    
      case 2:
        turnLeft(lowSpeed, range);
        state = 1;
        break;        
      //Are we turning "right"?
      case 3:
        turnRight(lowSpeed, range);
        state = 0;
        break;
      //Are we going straight?
      case 1: 
        pingTimer = 400;
        moveStraight(lowSpeed, range);
        state = 3;
        break;
      case 0:
        pingTimer = 400;
        moveStraight(lowSpeed, range);
        state = 2;
        break;
      default:
        state = 0;
        Serial.print("How did we even get here?");
        break;
    }
  }
}

//Returns "back" given two ground readings
int stuckOnWall(int d1, int d2) {
  int downD = findDownAcc();
  if(downD == d1) 
    return d2;
  else
    return d1;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int findDownAcc() {
  float rawX = analogRead(A0);
  float rawZ = analogRead(A2);
  
  //acceleration in terms of g: 
  //-3g = 0, -2g = 112.53, -1g = 225.06, 0g = 337.59, 1g = 450.12, 2g = 562.65, 3g = 675.18
  // 3.3/5 * 1023 = 675.18
  float x = mapf(rawX, 0, 675, -scale, scale);
  float z = mapf(rawZ, 0, 675, -scale, scale);
//Find the axis which is experiencing the greatest acceleration
  float maximum = max(x, z);
  float minimum = min(x, z);
  
  if(abs(minimum) > abs(maximum))
    maximum = minimum;
    
//Check the polarity of the acceleration and map it to an ultrasonic sensor 
  if(maximum == x && maximum < 0)
    return 3; //2
  else if(maximum == x && maximum > 0)
    return 1; //4
  else if(maximum == z && maximum < 0)
    return 4; //1
  else if(maximum == z && maximum > 0)
    return 2; //3
}

int getFace(int d[], int dSorted[], int index){
  for(int i = 0; i < 4; i++){
    if(d[i] == dSorted[index])
      return i+1;
  }
}


