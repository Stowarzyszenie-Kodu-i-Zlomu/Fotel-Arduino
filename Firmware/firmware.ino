#include "Arduino.h"
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <stdint.h>
#include <PID_v1.h>
#include <Servo.h>

#define MOSFET_1_PIN_RETRACT	7
#define MOSFET_1_PIN_EXTEND	8
#define MOSFET_2_PIN_RETRACT	9
#define MOSFET_2_PIN_EXTEND	10
#define MOSFET_3_PIN_RETRACT	11
#define MOSFET_3_PIN_EXTEND	12

#define DEV_I2C Wire1
#define PCAADDR 0x70

// define constant values
const int actuator_start_index = 0; //min 0
const int actuator_end_index = 3; //max 3
const int offset = 40;                // distance from sensor to base position of satellite                                                         
const int stroke = 500;               // pneumatic cylinder stroke
const int pidSwitchDistance = 70;
const int deadzone = 3;              // deadzone
const int sensorTiming = 0;
const int targetTiming = 300;
const int infoTiming = 5;

const double c1Kp = 0.5;          // actuator 1 PID tuning //0.1 //servo: 0.6
const double c1Ki = 0.1;          //0 //servo: 0.1
const double c1Kd = 0.001; //0.001 //servo: 0.005
// K krytyczne = 0.175, Tu = 0.0025s, kP = 0.105
const double a1Kp = 10.0;
const double a1Ki = 0.0;
const double a1Kd = 0.0;


// define variables
double distance[] = {0,0,0};
double target[] = {300,300,300};
double error[] = {0,0,0};
double actuatorTarget[] = {0,0,0};
bool extending[] = {false,false,false};
bool retracting[] = {false,false,false};
unsigned long sensorStart = 0;
unsigned long targetStart = 0;
unsigned long infoStart = 0;



// define an array for the 6ch relay module pins
int MOSFETPins[] = { 
  MOSFET_1_PIN_RETRACT, MOSFET_1_PIN_EXTEND,
  MOSFET_2_PIN_RETRACT, MOSFET_2_PIN_EXTEND,
  MOSFET_3_PIN_RETRACT, MOSFET_3_PIN_EXTEND};


// object initialization
Servo servo[3];
VL53L4CD sensor[3] = {
    VL53L4CD(&DEV_I2C, A1),
    VL53L4CD(&DEV_I2C, A1),
    VL53L4CD(&DEV_I2C, A1)
};
PID actuatorPID[3] = {
  PID(&distance[0], &actuatorTarget[0], &target[0], c1Kp, c1Ki, c1Kd, P_ON_E, DIRECT),
  PID(&distance[1], &actuatorTarget[1], &target[1], c1Kp, c1Ki, c1Kd, P_ON_E, DIRECT),
  PID(&distance[2], &actuatorTarget[2], &target[2], c1Kp, c1Ki, c1Kd, P_ON_E, DIRECT)};


//multiplexer selector
void pcaselect(uint8_t i) {
  if (i > 3) return;
 
  Wire1.beginTransmission(PCAADDR);
  Wire1.write(1 << i);
  Wire1.endTransmission();  
}


// -----===== Main functions =====-----



void setup() 
{
  Serial.begin(9600);
  delay(2000);
  mosfetSetup();
  Serial.print(1);
  servoSetup();
  Serial.print(2);
  pidSetup();
  Serial.print(3);
  sensorSetup();
  Serial.print(4);
  initTimings();
}


void loop()
{
  unsigned long currentTime = millis();

  if(currentTime - sensorStart >= sensorTiming){
    getDistance();
    sensorStart = millis();
  }
  
  move();

  if(currentTime - targetStart > targetTiming) {
    getTarget();
    targetStart = millis();
  }

  if(currentTime - infoStart > infoTiming) {
    plotInfo();
    infoStart = millis();
  }
}



// -----===== Other functions =====-----



void mosfetSetup() {
  for (int i = 0; i < 2*actuator_end_index; i++) { 
    pinMode(MOSFETPins[i], OUTPUT);
    digitalWrite(MOSFETPins[i], LOW);
  }
}


void servoSetup() {
  servo[0].attach(3);
  servo[0].write(0);
  servo[1].attach(5);
  servo[1].write(0);
  servo[2].attach(6);
  servo[2].write(0);
}


void pidSetup() {
  for(int i = actuator_start_index; i < actuator_end_index; i++){
   actuatorPID[i].SetOutputLimits(-90, 90);
   actuatorPID[i].SetMode(AUTOMATIC);
  }
}


void sensorSetup() {
  // Adafruit VL53L4CD init
  DEV_I2C.begin();
  for(int i = actuator_start_index; i < actuator_end_index; i++){
    
    pcaselect(i);
    sensor[i].begin();
    sensor[i].VL53L4CD_Off();
    sensor[i].InitSensor();
    sensor[i].VL53L4CD_SetRangeTiming(200, 0);
    sensor[i].VL53L4CD_StartRanging();
  }
  delay(2000);
  getDistance();
}


void initTimings() {
  sensorStart = millis();
  targetStart = millis();
  infoStart = millis();
}


void getDistance() {
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;

  for(int i = actuator_start_index; i < actuator_end_index; i++){
    pcaselect(i);
    do {
      status = sensor[i].VL53L4CD_CheckForDataReady(&NewDataReady);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0)) {
      sensor[i].VL53L4CD_ClearInterrupt();
      sensor[i].VL53L4CD_GetResult(&results);
      distance[i] = results.distance_mm - offset;
    }
  }
}


void move() {

  for(int i = actuator_start_index; i < actuator_end_index; i++){
    calculatePid(i);
    setServo(i);
    if(actuatorTarget[i] > deadzone) {
      stopRetracting(i);
      extend(i);
    } else if(actuatorTarget[i] < -deadzone) {
      stopExtending(i);
      retract(i);
    } else {
      stopExtending(i);
      stopRetracting(i);
    }
  }
}


void extend(int i) {

    if(!extending[i]) {
      digitalWrite(MOSFETPins[i*2+1], HIGH);
      extending[i] = true;
    }
}


void retract(int i) {
  if(!retracting[i]) {
    digitalWrite(MOSFETPins[i*2], HIGH);
    retracting[i] = true;
  }
}


void stopExtending(int i) {
    if(extending[i]) {
      digitalWrite(MOSFETPins[i*2 + 1], LOW);
      extending[i] = false;
    }

}


void stopRetracting(int i) {
    if(retracting[i]) {
      digitalWrite(MOSFETPins[i*2], LOW);
      retracting[i] = false;
    }
}


void getTarget() {
  
  if(Serial.available() > 8){
    String packet = Serial.readStringUntil(';');
    packet = Serial.readStringUntil(';');
    int first_comma  = packet.indexOf(',');
    int second_comma = packet.indexOf(',', first_comma+1);
    target[0] = packet.substring(0, first_comma).toInt();
    target[1] = packet.substring(first_comma + 1, second_comma).toInt();
    target[2] = packet.substring(second_comma + 1).toInt();
  }
}


void calculatePid(int i) {

    error[i] = target[i]-distance[i];
    //float diff = abs(target-distance);
    // if(diff < pidSwitchDistance) {
    //   actuatorPID.SetTunings(c1Kp, c1Ki, c1Kd);
    // } else {
    //   actuatorPID.SetTunings(a1Kp, a1Ki, a1Kd);
    // }
    actuatorPID[i].Compute();
  
}


void setServo(int i) {
  
    int deg = abs(actuatorTarget[i]);
    if(deg > 90){
      deg = 90;
    }
    servo[i].write(90 - deg);

}


void plotInfo() {
  for(int i = 0 ; i < 3; i++){
  Serial.print("Dist_: "+ i+1);
  Serial.print(distance[i]);
  Serial.print(",");
  Serial.print("PID_: " + i+1);
  Serial.print(actuatorTarget[i]);
  Serial.print(",");
  Serial.print("Trgt_ "+ i+1);
  Serial.println(target[i]);
  if(i != 2){
  Serial.print(",");
  }
  }
}