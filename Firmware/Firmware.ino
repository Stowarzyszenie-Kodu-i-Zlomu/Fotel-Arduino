// Include Libraries
#include "Arduino.h"
#include "LiquidCrystal.h"
#include "Potentiometer.h"
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <stdint.h>

// Pin Definitions
#define LCD_PIN_RS	9
#define LCD_PIN_E	8
#define LCD_PIN_DB4	4
#define LCD_PIN_DB5	5
#define LCD_PIN_DB6	6
#define LCD_PIN_DB7	7
#define POTENTIOMETER_5V_PIN_SIG	A3
#define RELAYMODULE4CH_PIN_IN1	10
#define RELAYMODULE4CH_PIN_IN2	11
#define DEV_I2C Wire1

// define constant values
const int offset = 35;                // dolna granica mierzenia sensora                                                              
const int stroke = 500;               // skok siłownika
const int deadzone = 25;              // deadzone

// define variables
int distance = 0;
int target = 0;
bool extending = false;
bool retracting = false;

// define an array for the 6ch relay module pins
int RelayModule6chPins[] = { RELAYMODULE4CH_PIN_IN1, RELAYMODULE4CH_PIN_IN2 };

// object initialization
LiquidCrystal lcd(LCD_PIN_RS,LCD_PIN_E,LCD_PIN_DB4,LCD_PIN_DB5,LCD_PIN_DB6,LCD_PIN_DB7);
Potentiometer potentiometer_5v(POTENTIOMETER_5V_PIN_SIG);
VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);



// ---=== Main functions ===---



// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
    Serial.begin(9600);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");
    
    // set up the LCD's number of columns and rows
    lcd.begin(16, 2);

    // set up the Relay
    pinMode(RELAYMODULE4CH_PIN_IN1, OUTPUT);
    pinMode(RELAYMODULE4CH_PIN_IN2, OUTPUT);

    // Turn off all of the relay channels
    for (int i = 0; i < 2; i++) { 
      digitalWrite(RelayModule6chPins[i],HIGH);
    }
    
    // Adafruit VL53L4CD init
    DEV_I2C.begin();
    sensorSetup();
}

// Main logic
void loop() 
{
  int target = getTarget();
  int distance = getDistance();
  lcdStats();
  delay(150);

  move();
  delay(200);
}



// ---=== Other functions ===---



void sensorSetup(){
  sensor_vl53l4cd_sat.begin();
  sensor_vl53l4cd_sat.VL53L4CD_Off();
  sensor_vl53l4cd_sat.InitSensor();
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();
}

int getDistance(){
  uint8_t sensorDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;
  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&sensorDataReady);
  } while (!sensorDataReady);

  if ((!status) && (sensorDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
  }
  return results.distance_mm;
}

// Reads potentiometer value and maps it from 0 to stroke
int getTarget(){
  int target = potentiometer_5v.read()/(1022/stroke);
  if(target>stroke){
    target = stroke;
  }
  return stroke-target;
}

// Print target distance, piston distance, show if retracting/extending on lcd
void lcdStats(){
  // First row
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Trgt:");
  lcd.setCursor(6, 0);
  lcd.print(target);
  if(retracting){
    lcd.setCursor(12, 0);
    lcd.print("+");
  }

  // Second row
  lcd.setCursor(0, 1);
  lcd.print("Dist:");
  lcd.setCursor(6, 1);
  lcd.print(distance);
  if(extending){
    lcd.setCursor(12, 1);
    lcd.print("-");
  }
}

void move(){
  if(distance<target-deadzone){
    stopRetracting();
    extend();
  } else if(distance>target+deadzone){
    stopExtending();
    retract();
  } else {
    stopExtending();
    stopRetracting();
  }
}

void extend(){
  if(!extending){
    digitalWrite(RelayModule6chPins[1],LOW);
    extending = true;
  }
}

void retract(){
  if(!retracting){
    digitalWrite(RelayModule6chPins[0],LOW);
    retracting = true;
  }
}

void stopExtending(){
  if(extending){
    digitalWrite(RelayModule6chPins[1],HIGH);
    extending = false;
  }
}

void stopRetracting(){
  if(retracting){
    digitalWrite(RelayModule6chPins[0],HIGH);
    retracting = false;
  }
}