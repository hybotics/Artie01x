/*
  Program:      4WD Rover (DFRobot Baron Rover) Motor and Servo Test sketch
  Date:         29-Jun-2014
  Version:      0.1.1 ALPHA

  Platform:     DFRobot Romeo v1.1 Microcntroller (Arduino Uno compatible)

  Purpose:      To have FUN with a little 4WD rover - can control the Rover over a serial link.

                                                  Change Log
                -------------------------------------------------------------------------------
                v0.1.0 ALPHA 27-Jun-2014:
                Initial build from example code on the DFRobot Wiki
                -------------------------------------------------------------------------------
                v0.1.1 ALPHA 29-Jun-2014:
                Motors work when controlled over the serial port.
                Adding servo test code.
                -------------------------------------------------------------------------------

  Dependencies: Adafruit libraries:
                  Adafruit_Sensor, Adafruit_TMP006, and Adafruit_TCS34725, Adafruit_LEDBackpack,
                  Adafruit_GFX libraries

                Hybotics libraries:
                  None (Yet)

                Other libraries:
                  RTClib for the DS1307 (Adafruit's version)

  Comments:     Credit is given, where applicable, for code I did not originate.

  Copyright (c) 2014 Dale A. Weber <hybotics.pdx@gmail.com, @hybotics on App.Net and Twitter>
*/
#include <Wire.h>
#include <Servo.h>
#include <BMSerial.h>

#include "Romeo_v1_1_Motor_Servo_Test.h"

//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control
 
void stop (byte durationMS = 0) {
  digitalWrite(E1, LOW);  
  digitalWrite(E2, LOW);     

  if (durationMS > 0) {
    delay(durationMS);
  }
}

void forward (char a, char b, byte durationMS = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, HIGH);   
  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);

  if (durationMS > 0) {
    delay(durationMS);
    stop();
  }
} 

void reverse (char a, char b, byte durationMS = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);  
  analogWrite (E2, b);   
  digitalWrite(M2, LOW);

  if (durationMS > 0) {
    delay(durationMS);
    stop();
  }
}

void turnLeft (char a, char b, byte durationMS = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);   
  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);

  if (durationMS > 0) {
    delay(durationMS);
    stop();
  }
}

void turnRight (char a, char b, byte durationMS = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, HIGH);   
  analogWrite (E2, b);   
  digitalWrite(M2, LOW);

  if (durationMS > 0) {
    delay(durationMS);
    stop();
  }
}

void setup (void) {
  int i;

  //  Set all the motor control pins to OUTPUT
  for(i = M1; i <= M2; i++) {
    pinMode(i, OUTPUT); 
  }

  Serial.begin(9600);
  Serial.println(F("Run keyboard control.."));
}

void loop(void) {
  char val;

  if (Serial.available()) {
    Serial.println(F("Processing serial command.."));

    val = Serial.read();

    if (val != -1) {
      switch(val) {
        case 'w':
        case 'W':
          forward(100, 100);
          break;

        case 's':
        case 'S':
          reverse(100, 100);
          break;

        case 'a':
        case 'A':
          turnLeft(100, 100);
          break;      

        case 'd':
        case 'D':
          turnRight(100, 100);
          break;

        case 'z':
        case 'Z':
          Serial.println("Hello");
          break;

        case 'x':
        case 'X':
          stop();
          break;

        default:
          Serial.println(F("Invalid command received!"));
          break;
      }
    }
      else stop(); 
  }/* else {
    //  Perform a sequence of maneuvers
//    Serial.println(F("Performing maneuvers.."));
//    Serial.println();

//    Serial.println(F("Moving forward.."));
    forward(100, 100, 250);

//    Serial.println(F("Turning LEFT.."));
    turnLeft(100, 100, 250);

//    Serial.println(F("Moving forward.."));
    forward(100, 100, 250);

//    Serial.println(F("Turning LEFT.."));
    turnLeft(100, 100, 250);

//    Serial.println(F("Moving forward.."));
    forward(100, 100, 250);

//    Serial.println(F("Turning LEFT.."));
    turnLeft(100, 100, 250);

//    Serial.println(F("Moving forward.."));
    forward(100, 100, 250);

//    Serial.println(F("Stopping motors.."));
    stop(2000);
  }
*/

//  Serial.println(F("Turning RIGHT.."));
//  Serial.println(F("Reversing.."));

}
