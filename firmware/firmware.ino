#include "Arduino.h"
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <stdint.h>
#include <PID_v1.h>
#include <Servo.h>

#define MOSFET_PIN_SIG1	10
#define MOSFET_PIN_SIG2	11

#define DEV_I2C Wire1


// define constant values
const int offset = 40;                // distance from sensor to base position of satellite
const int stroke = 500;               // pneumatic cylinder stroke
const int deadzone = 1;              // deadzone
const int sensorTiming = 0;
const int targetTiming = 300;
const int infoTiming = 5;

const double a1Kp = 0.11;          // actuator 1 PID tuning //0.1
const double a1Ki = 0.000;
const double a1Kd = 0.001; //0.001
// K krytyczne = 0.175, Tu = 0.0025s, kP = 0.105


// define variables
double distance = 0;
double target = 300;
double error = 0;
double actuatorTarget = 0;
bool extending = false;
bool retracting = false;
unsigned long sensorStart = 0;
unsigned long targetStart = 0;
unsigned long infoStart = 0;


// define an array for the 6ch relay module pins
int MOSFETPins[] = { MOSFET_PIN_SIG1, MOSFET_PIN_SIG2 };


// object initialization
Servo servo;
VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);
PID actuatorPID(&distance, &actuatorTarget, &target, a1Kp, a1Ki, a1Kd, DIRECT);



// -----===== Main functions =====-----



void setup()
{
  Serial.begin(9600);
  delay(2000);
  mosfetSetup();
  servoSetup();
  pidSetup();
  sensorSetup();
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
  for (int i = 0; i < 2; i++) {
    pinMode(MOSFETPins[i], OUTPUT);
    digitalWrite(MOSFETPins[i], LOW);
  }
}


void servoSetup() {
  servo.attach(3);
  servo.write(0);
}


void pidSetup() {
  actuatorPID.SetOutputLimits(-90, 90);
  actuatorPID.SetMode(AUTOMATIC);
}


void sensorSetup() {
  // Adafruit VL53L4CD init
  DEV_I2C.begin();
  sensor_vl53l4cd_sat.begin();
  sensor_vl53l4cd_sat.VL53L4CD_Off();
  sensor_vl53l4cd_sat.InitSensor();
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();
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

  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);

    distance = results.distance_mm - offset;
  }
}


void move() {
  error = target-distance;
  actuatorPID.Compute();
  setServo();
  if(actuatorTarget > deadzone) {
    stopRetracting();
    extend();
  } else if(actuatorTarget < -deadzone) {
    stopExtending();
    retract();
  } else {
    stopExtending();
    stopRetracting();
  }
}


void extend() {
  if(!extending) {
    digitalWrite(MOSFETPins[1], HIGH);
    extending = true;
  }
}


void retract() {
  if(!retracting) {
    digitalWrite(MOSFETPins[0], HIGH);
    retracting = true;
  }
}


void stopExtending() {
  if(extending) {
    digitalWrite(MOSFETPins[1], LOW);
    extending = false;
  }
}


void stopRetracting() {
  if(retracting) {
    digitalWrite(MOSFETPins[0], LOW);
    retracting = false;
  }
}


void getTarget() {
  if(Serial.available()) {
    target = Serial.parseInt();
    Serial.read();
  }
}


void setServo() {
  int deg = actuatorTarget;
  if(deg < 0) {
    deg = -deg;
  }
  if(deg > 90){
    deg = 90;
  }
  servo.write(90 - deg);
}


void plotInfo() {
  Serial.print("Dist:");
  Serial.print(distance);
  Serial.print(",");
  Serial.print("PID:");
  Serial.print(actuatorTarget);
  Serial.print(",");
  Serial.print("Trgt:");
  Serial.println(target);
}
