// Include Libraries
#include "Arduino.h"
#include "LiquidCrystal.h"
#include "Potentiometer.h"

// Pin Definitions
#define HCSR04_PIN_TRIG	3
#define HCSR04_PIN_ECHO	2
#define LCD_PIN_RS	9
#define LCD_PIN_E	8
#define LCD_PIN_DB4	4
#define LCD_PIN_DB5	5
#define LCD_PIN_DB6	6
#define LCD_PIN_DB7	7
#define POTENTIOMETER_5V_PIN_SIG	A3
#define RELAYMODULE4CH_PIN_IN1	10
#define RELAYMODULE4CH_PIN_IN2	11

// Global variables and defines
//define an array for the 4ch relay module pins
int RelayModule6chPins[] = { RELAYMODULE4CH_PIN_IN1, RELAYMODULE4CH_PIN_IN2 };

// object initialization
LiquidCrystal lcd(LCD_PIN_RS,LCD_PIN_E,LCD_PIN_DB4,LCD_PIN_DB5,LCD_PIN_DB6,LCD_PIN_DB7);
Potentiometer potentiometer_5v(POTENTIOMETER_5V_PIN_SIG);

// define vars for testing menu
const int timeout = 3000;       //define timeout of 10 sec
char menuOption = 0;
long time0;
int offset = 19;                //dolna granica mierzenia sensora                                                                 
int skok = 80;                  //skok siłownika

// Function definitions
void setup();
void loop();
void ultrasonicSensor();
int lcDisplay(String line1, String line2);
void potentiometer();
void relayModule();

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
    // Setup Serial which is useful for debugging
    // Use the Serial Monitor to view printed messages
    Serial.begin(9600);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");
    
    // set up the LCD's number of columns and rows
    lcd.begin(16, 2);

    // set up the Relay
    pinMode(RELAYMODULE4CH_PIN_IN1, OUTPUT);
    pinMode(RELAYMODULE4CH_PIN_IN2, OUTPUT);

    // set up the ultrasonic sensor
    pinMode(HCSR04_PIN_TRIG, OUTPUT);
    pinMode(HCSR04_PIN_ECHO, INPUT);

    // Turn off all of the relay channels
    for (int i = 0; i < 2; i++) { 
      digitalWrite(RelayModule6chPins[i],HIGH);
    }
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
  bool retracting = false;
  bool extending = false;
  int target = getTarget();
  int distance = ultrasonicDistance();
  int diff = target-distance;

  // Kod do rozciągania i chowania siłownika

  lcdStats(target, distance, retracting, extending);
  delay(100);
}

// returns distance in mm as int
int ultrasonicDistance(){
    // trig pin controls sending pulses
    // echo pin turns on when waves bounce back and hit it

    // make sure trig pin is low 
    digitalWrite(HCSR04_PIN_TRIG, LOW);  
    delayMicroseconds(2);  

    // send pulses
    digitalWrite(HCSR04_PIN_TRIG, HIGH);  
    delayMicroseconds(10);

    // stop sending pulses
    digitalWrite(HCSR04_PIN_TRIG, LOW);

    // wait for echo pin to send HIGH signal
    float duration = pulseIn(HCSR04_PIN_ECHO, HIGH);  

    // speed = distance / time -> distance = time * speed
    // divided by 2 because it needs to travel back

    // in cm / microsecond
    float speedOfSound = 0.0343;

    float distancef = (duration * speedOfSound) / 2;  

    // convert cm to mm
    int distance = floor(distancef*10);

    //set limits (sensor works from certain distance - offset), max distance is skok siłownika
    if(distance>offset){
      if(distance>skok+offset){
        return skok;
      } else {
        return distance-offset;
      }
    } else {
      return 0;
    }
}

void ultrasonicSensor(){
    // Ultrasonic Sensor - HC-SR04 - Test Code
    // Read distance measurment from UltraSonic sensor           
    int dist = ultrasonicDistance();
    delay(10);
    Serial.print(F("Distance: ")); Serial.print(dist); Serial.println(F("[mm]"));
}


// Reads potentiometer value and maps it from 0 to skok siłownika
int getTarget(){
  int target = potentiometer_5v.read()/(1022/skok);
  if(target>skok){
    target = skok;
  }
  return skok-target;
}

// Print target distance, piston distance, show if retracting/extending on lcd
void lcdStats(int target, int distance, bool retracting, bool extending){
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

void relayModule(){
    // Relay Module 6-Ch - Test Code
    // This loop will turn on and off each relay in the array for 0.1 sec and then wait 0.7 sec
    for (int i = 0; i < 2; i++) { 
      digitalWrite(RelayModule6chPins[i],LOW);
      delay(100);
      digitalWrite(RelayModule6chPins[i],HIGH);
      delay(700);
    }
 }
