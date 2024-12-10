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

    for (int i = 0; i < 2; i++) { 
      digitalWrite(RelayModule6chPins[i],HIGH);
    }

    // show the menu (check in Serial Monitor)
    menuOption = menu();
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
    switch(menuOption){
      case '1': 
        ultrasonicSensor();
        break;

      case '2':
        lcDisplay("Stowarzyszenie", "Kodu i Zlomu");
        break;

      case '3':
        potentiometer();
        break;

      case '4':
        relayModule();
        break;
    }
  
    if (millis() - time0 > timeout)
    {
        menuOption = menu();
            
    }
}



// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions
char menu()
{
  Serial.println(F("\nWhich component would you like to test?"));
  Serial.println(F("(1) Ultrasonic Sensor - HC-SR04"));
  Serial.println(F("(2) LCD 16x2"));
  Serial.println(F("(3) Rotary Potentiometer - 10k Ohm, Linear"));
  Serial.println(F("(4) Relay Module 4-Ch"));
  Serial.println(F("(menu) send anything else or press on board reset button\n"));
  while (!Serial.available());

  // Read data from serial monitor if received
  while (Serial.available()) {
    char c = Serial.read();
    if (isAlphaNumeric(c)) {     
      switch(c){
        case '1':
  			  Serial.println(F("Now Testing Ultrasonic Sensor - HC-SR04"));
          break;

        case '2':
    		  Serial.println(F("Now Testing LCD 16x2"));
          break;

        case '3':
          Serial.println(F("Now Testing Rotary Potentiometer - 10k Ohm, Linear"));
          break;

        case '4':
          Serial.println(F("Now Testing Relay Module 4-Ch"));
          break;

        default:
          Serial.println(F("illegal input!"));
          return 0;
      }

      time0 = millis();
      return c;
    }
  }
}

// returns distance in cm as float
float ultrasonicDistance(){
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

    float distance = (duration * speedOfSound) / 2;  
    return distance;
}

void ultrasonicSensor(){
    // Ultrasonic Sensor - HC-SR04 - Test Code
    // Read distance measurment from UltraSonic sensor           
    float dist = ultrasonicDistance();
    delay(10);
    Serial.print(F("Distance: ")); Serial.print(dist); Serial.println(F("[cm]"));
}

// returns 1 on failure and 0 on success
int lcDisplay(String line1, String line2){
    if(line1.length() > 16 && line2.length() > 16){
      Serial.print(F("Length of each line must be at most 16 characters"));
      return 1;
    }
    // LCD 16x2 - Test Code
    // Print a message to the LCD.
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);

    return 0;
}

void potentiometer(){
    // Rotary Potentiometer - 10k Ohm, Linear - Test Code
    int potentiometer_5vVal = potentiometer_5v.read();
    Serial.print(F("Val: ")); Serial.println(potentiometer_5vVal);
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
