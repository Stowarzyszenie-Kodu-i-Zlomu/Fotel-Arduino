// Include Libraries
#include "Arduino.h"
#include "NewPing.h"
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
NewPing hcsr04(HCSR04_PIN_TRIG,HCSR04_PIN_ECHO);
LiquidCrystal lcd(LCD_PIN_RS,LCD_PIN_E,LCD_PIN_DB4,LCD_PIN_DB5,LCD_PIN_DB6,LCD_PIN_DB7);
Potentiometer potentiometer_5v(POTENTIOMETER_5V_PIN_SIG);

// define vars for testing menu
const int timeout = 3000;       //define timeout of 10 sec
char menuOption = 0;
long time0;



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
    for (int i = 0; i < 2; i++) { 
      digitalWrite(RelayModule6chPins[i],HIGH);
    }

    // show the menu (check in Serial Monitor)
    menuOption = menu();
    
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
    if(menuOption == '1') {
      // Ultrasonic Sensor - HC-SR04 - Test Code
      // Read distance measurment from UltraSonic sensor           
      int hcsr04Dist = hcsr04.ping_cm();
      delay(10);
      Serial.print(F("Distance: ")); Serial.print(hcsr04Dist); Serial.println(F("[cm]"));
    }

    else if(menuOption == '2') {
      // LCD 16x2 - Test Code
      // Print a message to the LCD.
      lcd.setCursor(0, 0);
      lcd.print("Stowarzyszenie");
      lcd.setCursor(0, 1);
      lcd.print("Kodu i Zlomu");

      // Turn off the display:
      lcd.noDisplay();
      delay(50);

      // Turn on the display:
      lcd.display();
      delay(50);
    }

    else if(menuOption == '3') {
      // Rotary Potentiometer - 10k Ohm, Linear - Test Code
      int potentiometer_5vVal = potentiometer_5v.read();
      Serial.print(F("Val: ")); Serial.println(potentiometer_5vVal);
    }

    else if(menuOption == '4') {
      // Relay Module 6-Ch - Test Code
      //This loop will turn on and off each relay in the array for 0.1 sec and then wait 0.7 sec
      for (int i = 0; i < 2; i++) { 
        digitalWrite(RelayModule6chPins[i],LOW);
        delay(100);
        digitalWrite(RelayModule6chPins[i],HIGH);
        delay(700);
      }
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
      if(c == '1') 
  			Serial.println(F("Now Testing Ultrasonic Sensor - HC-SR04"));
  		else if(c == '2') 
    		Serial.println(F("Now Testing LCD 16x2"));
    	else if(c == '3') 
  			Serial.println(F("Now Testing Rotary Potentiometer - 10k Ohm, Linear"));
  		else if(c == '4') 
    		Serial.println(F("Now Testing Relay Module 4-Ch"));
      else {
        Serial.println(F("illegal input!"));
        return 0;
      }
      time0 = millis();
      return c;
    }
  }
}

/*******************************************************

*    Stowarzyszenie Kodu i Złomu

*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.

*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.

*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*    In addition, and without limitation, to the disclaimers of warranties 
*    stated above and in the GNU General Public License version 3 (or any 
*    later version), Roboplan Technologies Ltd. ("Roboplan") offers this 
*    program subject to the following warranty disclaimers and by using 
*    this program you acknowledge and agree to the following:
*    THIS PROGRAM IS PROVIDED ON AN "AS IS" AND "AS AVAILABLE" BASIS, AND 
*    WITHOUT WARRANTIES OF ANY KIND EITHER EXPRESS OR IMPLIED.  ROBOPLAN 
*    HEREBY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT 
*    NOT LIMITED TO IMPLIED WARRANTIES OF MERCHANTABILITY, TITLE, FITNESS 
*    FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT, AND THOSE ARISING BY 
*    STATUTE OR FROM A COURSE OF DEALING OR USAGE OF TRADE. 
*    YOUR RELIANCE ON, OR USE OF THIS PROGRAM IS AT YOUR SOLE RISK.
*    ROBOPLAN DOES NOT GUARANTEE THAT THE PROGRAM WILL BE FREE OF, OR NOT 
*    SUSCEPTIBLE TO, BUGS, SECURITY BREACHES, OR VIRUSES. ROBOPLAN DOES 
*    NOT WARRANT THAT YOUR USE OF THE PROGRAM, INCLUDING PURSUANT TO 
*    SCHEMATICS, INSTRUCTIONS OR RECOMMENDATIONS OF ROBOPLAN, WILL BE SAFE 
*    FOR PERSONAL USE OR FOR PRODUCTION OR COMMERCIAL USE, WILL NOT 
*    VIOLATE ANY THIRD PARTY RIGHTS, WILL PROVIDE THE INTENDED OR DESIRED
*    RESULTS, OR OPERATE AS YOU INTENDED OR AS MAY BE INDICATED BY ROBOPLAN. 
*    YOU HEREBY WAIVE, AGREE NOT TO ASSERT AGAINST, AND RELEASE ROBOPLAN, 
*    ITS LICENSORS AND AFFILIATES FROM, ANY CLAIMS IN CONNECTION WITH ANY OF 
*    THE ABOVE. 
********************************************************/