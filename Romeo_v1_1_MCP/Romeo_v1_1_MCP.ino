/*
  Program:      4WD Rover (DFRobot Baron Rover) Master Control Program (MCP)
  Date:         27-Jun-2014
  Version:      0.1.0 ALPHA

  Platform:     DFRobot Romeo v1.1 Microcntroller (Arduino Uno compatible)

  Purpose:      To have FUN with a little 4WD rover - controls the Rover over the serial link.

                                                  Change Log
                -------------------------------------------------------------------------------
                v0.1.0 ALPHA 27-Jun-2014:
                Initial build from example code on the DFRobot Wiki

                Added ms delay parameter to all motor functions to provide a delayed exit before
                  the next function.
                -------------------------------------------------------------------------------

  Dependencies: None (Yet)

  Comments:     Credit is given, where applicable, for code I did not originate.

  Copyright (c) 2014 Dale A. Weber <hybotics.pdx@gmail.com, @hybotics on App.Net and Twitter>
*/

#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>

/*
  Additional sensors
*/
#include <Adafruit_TMP006.h>
#include <Adafruit_TCS34725.h>

#include "Romeo_V1_1_MCP.h"

//  Standard PWM DC control
#define E1 5;                         //  M1 Speed Control
#define M1 4;                         //  M1 Direction Control

#define E2 6;                         //  M2 Speed Control
#define M2 7;                         //  M2 Direction Control

//  Define the servo object for the pan servo
Servo pan;

/*
  Display the TCS34725 RGB color sensor readings
*/
void displayColorSensorData (ColorSensor *colorData) {
  lastRoutine = String(F("displayColorSensorData"));

  console.println(F("Color Sensor Data:"));
  console.println();

  console.print(F("Color Temperature: "));
  console.print(colorData->colorTempC, DEC);
  console.print(F(" K - Lux: "));
  console.print(colorData->lux, DEC);
  console.print(F(" - Red: "));
  console.print(colorData->red, DEC);
  console.print(F(" Green: "));
  console.print(colorData->green, DEC);
  console.print(F(" Blue: "));
  console.print(colorData->blue, DEC);
  console.print(F(" C: "));
  console.println(colorData->c, DEC);
  console.println(F("."));
}

/*
  Display the TMP006 heat sensor readings
*/
void displayHeatSensorData (HeatSensor *heatData) {
  lastRoutine = String(F("displayHeatSensorData"));

  float objCelsius = heatData->objectTempC;
  float objFahrenheit = toFahrenheit(objCelsius);
  float dieCelsius = heatData->dieTempC;
  float dieFahrenheit = toFahrenheit(dieCelsius);

  console.println(F("Heat Sensor Data:"));
  console.println();

  console.print(F("Object Temperature: "));
  console.print(objFahrenheit);
  console.print(F(" F, "));
  console.print(objCelsius);
  console.println(F(" C."));
  console.print(F("Die Temperature: "));
  console.print(dieFahrenheit);
  console.print(F(" F, "));
  console.print(dieCelsius);
  console.println(F(" C."));
}

/*
    Display the GP2Y0A21YK0F IR sensor readings (cm)
*/
void displayIR (void) {
  uint8_t sensorNr = 0;

  lastRoutine = String("displayIR");
  
  console.println(F("IR Sensor readings:"));

  while (sensorNr < MAX_NUMBER_IR) {
    console.print(F("IR #"));
    console.print(sensorNr + 1);
    console.print(F(" range = "));
    console.print(ir[sensorNr]);
    console.println(F(" cm"));

    sensorNr += 1;
  }

  console.println();
}

/*
  Display the readings from the PING Ultrasonic sensors
*/
void displayPING (void) {
  uint8_t sensorNr = 0;

  lastRoutine = String(F("displayPING"));
  
  console.println("PING Ultrasonic Sensor readings:");
  
  //  Display PING sensor readings (cm)
  while (sensorNr < MAX_NUMBER_PING) {
    console.print("Ping #");
    console.print(sensorNr + 1);
    console.print(" range = ");
    console.print(ping[sensorNr]);
    console.println(" cm");

    sensorNr += 1;
  }
 
  console.println();
}

ColorSensor readColorSensor (void) {
  ColorSensor colorData;

  rgb.getRawData(&colorData.red, &colorData.green, &colorData.blue, &colorData.c);
  colorData.colorTempC = rgb.calculateColorTemperature(colorData.red, colorData.green, colorData.blue);
  colorData.lux = rgb.calculateLux(colorData.red, colorData.green, colorData.blue);

  return colorData;
}

HeatSensor readHeatSensor (void) {
  HeatSensor heatData;

  heatData.dieTempC = heat.readDieTempC();
  heatData.objectTempC = heat.readObjTempC();

  return heatData;
}

/*
  Read distance in cm from a Sharp GP2Y0A21YK0F IR sensor
*/
float readSharpGP2Y0A21YK0F (byte sensorNr) {
  byte pin = sensorNr + IR_PIN_BASE;
  int reading = analogRead(pin);
  float distance = (6762.0 / (reading - 9)) - 4;

  lastRoutine = String(F("readSharpGP2Y0A21YK0F"));

  return distance;
}

/*
  Parallax Ping))) Sensor 

  This routine reads a PING))) ultrasonic rangefinder and returns the
    distance to the closest object in range. To do this, it sends a pulse
    to the sensor to initiate a reading, then listens for a pulse
    to return.  The length of the returning pulse is proportional to
    the distance of the object from the sensor.

  The circuit:
    * +V connection of the PING))) attached to +5V
    * GND connection of the PING))) attached to ground
    * SIG connection of the PING))) attached to digital pin 7

  http://www.arduino.cc/en/Tutorial/Ping

  Created 3 Nov 2008
    by David A. Mellis

  Modified 30-Aug-2011
    by Tom Igoe

  Modified 09-Aug-2013
    by Dale Weber

    Set units = true for cm, and false for inches
*/
int readParallaxPING (byte sensorNr, boolean units = true) {
  byte pin = sensorNr + PING_PIN_BASE;
  long duration;
  int result;

  lastRoutine = String(F("readParallaxPING"));

  /*
    The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  */
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  /*
    The same pin is used to read the signal from the PING))): a HIGH
    pulse whose duration is the time (in microseconds) from the sending
    of the ping to the reception of its echo off of an object.
  */
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  //  Convert the duration into a distance
  if (units) {
    //  Return result in cm
    result = microsecondsToCentimeters(duration);
  } else {
    //  Return result in inches.
    result = microsecondsToInches(duration);
  }
 
  delay(100);
  
  return result;
}

/********************************************************************/
/*  Lynxmotion SSC-32 Servo Controller routines                     */
/********************************************************************/
/*  These need to be modified to use the Servo library              */
/********************************************************************/

/*
    Move a servo by pulse width in ms (500ms - 2500ms) - Modified to use HardwareSerial2()
*/
uint16_t moveServoPw (Servo *servo, int servoPosition, boolean term = true, uint16_t moveSpeed = 0, uint16_t moveTime = 0) {
  uint16_t errorStatus = 0;
  char asciiCR = 13;

  lastRoutine = String(F("moveServoPw"));

  servo->error = 0;
  
  if ((servoPosition >= servo->minPulse) && (servoPosition <= servo->maxPulse)) {
    ssc32Command = ssc32Command + "#" + String(servo->pin) + "P" + String(servoPosition + servo->offset) + " ";

    servo->msPulse = servoPosition;
    servo->angle = ((servoPosition - SERVO_CENTER_MS) / 10);
    
    if (servo->maxDegrees == 180) {
      servo->angle += 90;
    }
  }

  if ((servoPosition < servo->minPulse) || (servoPosition > servo->maxPulse)) {
    errorStatus = 201;
    processError(errorStatus, F("Servo pulse is out of range"));
  } else {
    //  Add servo move speed
    if (moveSpeed != 0) {
      ssc32Command = ssc32Command + " S" + String(moveSpeed) + " ";
    }
    
    //  Terminate the command
    if (term == true) {
      if (moveTime != 0) {
        ssc32Command = ssc32Command + " T" + String(moveTime) + String(" ");
      }

      ssc32Command = ssc32Command + asciiCR;

      ssc32.print(ssc32Command);
      ssc32.println();

      ssc32Command = "";
    }
    }

  if (errorStatus != 0) {
    servo->error = errorStatus;
  }

    return errorStatus;
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180) - Modified to use BMSerial
*/
uint16_t moveServoDegrees (Servo *servo, int servoDegrees, boolean term, uint16_t moveSpeed = 0, uint16_t moveTime = 0) {
  uint16_t servoPulse;

  uint16_t errorStatus = 0;
  String errorMsg;

  lastRoutine = String(F("moveServoDegrees"));

  servo->error = 0;
  
  //  Convert degrees to ms for the servo move
  if (servo->maxDegrees == 90) {
    servoPulse = SERVO_CENTER_MS + (servoDegrees * 11);
  } else if (servo->maxDegrees == 180) {
    servoPulse = SERVO_CENTER_MS + ((servoDegrees -90) * 11);
  } else {
    errorStatus = 202;
  }

  if (errorStatus != 0) {
    processError(errorStatus, F("Servo position (degrees) is invalid"));
  } else {
    if ((servoPulse >= servo->minPulse) && (servoPulse <= servo->maxPulse)) {
      errorStatus = moveServoPw(servo, servoPulse, true, moveSpeed, moveTime);

      if (errorStatus != 0) {
        processError(errorStatus, "Could not move the " + servo->descr + " servo");
      }
    }
  }

  return errorStatus;
}

DistanceObject findDistanceObjects () {
  uint8_t readingNr;

  DistanceObject distObj = {0, 0, 0, 0, 0, 0, 0, 0};

  console.println(F("Finding the closest and farthest objects.."));

  //  Find the closest and farthest objects
  for (readingNr = 0; readingNr <= nrAreaReadings; readingNr++) {
    //  Check for the closest object
    if (areaScan[readingNr].ping < areaScan[distObj.closestPING].ping) {
      distObj.closestPING = readingNr;
      distObj.closestPosPING = areaScan[readingNr].positionDeg;
    }

    if (areaScan[readingNr].ir <=  areaScan[distObj.closestIR].ir) {
      distObj.closestIR = readingNr;
      distObj.closestPosIR = areaScan[readingNr].positionDeg;
    }

    //  Check for the farthest object
    if (areaScan[readingNr].ping > areaScan[distObj.farthestPING].ping) {
      distObj.farthestPING = readingNr;
      distObj.farthestPosPING = areaScan[readingNr].positionDeg;
    }

    if (areaScan[readingNr].ir > areaScan[distObj.farthestIR].ir) {
      distObj.farthestIR = readingNr;
      distObj.farthestPosIR = areaScan[readingNr].positionDeg;
    }
  }

  return distObj;
}

/*
  Scan an arc of up to 180 degrees, and take sensor readings at each angle increment
*/
uint16_t scanArea (Servo *pan, int startDeg, int stopDeg, int incrDeg) {
  uint16_t errorStatus = 0;
  uint16_t readingNr = 0, nrReadings = 0;
  int positionDeg = 0;
  int totalRangeDeg = 0;

  lastRoutine = String(F("scanArea"));

  //  Check the parameters
  if (startDeg > stopDeg) {
    //  Start can't be greater than stop
    errorStatus = 401;
  } else if (((pan->maxDegrees == 90) && ((startDeg < -90) || (stopDeg > 90))) || ((pan->maxDegrees == 180) && ((startDeg < 0) || (stopDeg > 180)))) {
    //  One or more parameters is outside of the valid range
    errorStatus = 402;
  } else if ((startDeg < pan->minPulse) || (stopDeg > pan->maxPulse)) {
    //  Out of range for the pan servo
    errorStatus = 403;
  } else {
    //  Calculate the total range, in degrees
    totalRangeDeg = abs(stopDeg - startDeg);

    //  Calculate the number of readings we need room for
    nrReadings = totalRangeDeg / incrDeg;

    //  More error checking
    if (totalRangeDeg > 180) {
      //  Servos can only move up to 180 degrees
      errorStatus = 404;
    } else if (nrReadings > MAX_NUMBER_AREA_READINGS) {
      //  We can't take this many readings
      errorStatus = 405;
    } else if (incrDeg > totalRangeDeg) {
      //  Increment is too large for our range
      errorStatus = 406;
    } else {
      /*
        Continue normal processing
      */

      //  Stop, so we can do this scan
      errorStatus = stopGearMotors();

      if (errorStatus != 0) {
        runAwayRobot(errorStatus);
      } else {
        readingNr = 0;

        console.println(F("Scanning the area.."));

        for (positionDeg = startDeg; positionDeg < stopDeg; positionDeg += incrDeg) {
          errorStatus = moveServoDegrees(pan, positionDeg, true);

          if (errorStatus != 0) {
            processError(errorStatus, "Could not move the " + pan->descr + " servo");
            break;
          } else {
            //  Delay to let the pan/tilt stabilize after moving it
            delay(1500);

            //  Take a reading from each pan/tilt sensor in cm
            areaScan[readingNr].ping = readParallaxPING(PING_FRONT_CENTER, true);
            areaScan[readingNr].ir = readSharpGP2Y0A21YK0F(IR_FRONT_CENTER);
            areaScan[readingNr].positionDeg = positionDeg;

            if (HAVE_COLOR_SENSOR) {
              areaScan[readingNr].color = readColorSensor();
            }

            if (HAVE_HEAT_SENSOR) {
              areaScan[readingNr].heat = readHeatSensor();
            }

            readingNr += 1;
          }
        }

        //  Send the pan servo back to home position
        errorStatus = moveServoPw(pan, pan->homePos, true);

        if (errorStatus != 0) {
          processError(errorStatus, "Could not move the " + pan->descr + " servo");
        }
      }
    }
  }

  if (errorStatus != 0) {
    processError(errorStatus, F("Could not complete the area scan"));
    nrAreaReadings = -1;
    areaScanValid = false;
  } else {
    //  Robot has not moved
    hasNotMoved = true;

    //  Set the number of readings taken
    nrAreaReadings = readingNr;

    //  This area scan is valid
    areaScanValid = true;
  }

  return errorStatus;
}

/*
  Turn towards the farthest detected object
*/
uint16_t turnToFarthestObject (DistanceObject *distObj, Servo *pan) {
  uint16_t errorStatus = 0;

  if (distObj->farthestPosPING < 0) {
    //  Turn to the right
    errorStatus = setGearMotorSpeed(&rightGearMotorM1, -50, false);

    if (errorStatus == 0) {
      errorStatus = setGearMotorSpeed(leftGearMotorM2, 50, true);
      delay(1000);
    }

    if (errorStatus == 0) {
      //  Start moving forward again
      errorStatus = setGearMotorSpeed(&rightGearMotorM1, 50, true);
    }

    if (errorStatus != 0) {
      processError(errorStatus, F("There was a problem turning RIGHT"));
    }
  } else if (distObj->farthestPosPING > 0) {
    //  Turn to the left
    errorStatus = setGearMotorSpeed(&rightGearMotorM1, 50, false);

    if (errorStatus == 0) {
      errorStatus = setGearMotorSpeed(&leftGearMotorM2, -50, true);
      delay(1000);
    }

    if (errorStatus == 0) {
      //  Start moving forward again
      errorStatus = setGearMotorSpeed(&leftGearMotorM2, 50, true);
    }

    if (errorStatus != 0) {
      processError(errorStatus, F("There was a problem turning LEFT"));
    }
  } else {
    //  Backup and scan again
    stopGearMotors();

    errorStatus = setGearMotorSpeed(&rightGearMotorM1, -50, false);

    if (errorStatus == 0) {
      errorStatus = setGearMotorSpeed(&leftGearMotorM2, -50, true);
      delay(1000);

      stopGearMotors();
    } else {
      processError(errorStatus, F("There was a problem backing up"));
    }

    if (errorStatus == 0) {
      errorStatus = stopGearMotors();

      if (errorStatus != 0) {
        runAwayRobot(errorStatus);
      } else {
        errorStatus = scanArea(pan, -90, 90, 10);

        if (errorStatus != 0) {
          processError(errorStatus, F("There was a problem with the area scan"));
        }
      }
    }
  }

  return errorStatus;
}
 
//  Stop 
void stop (void) {
  digitalWrite(E1, LOW);  
  digitalWrite(E2, LOW);     
}

//  Move forward
void forward (char a, char b, short ms = 0) {
  analogWrite (E1, a);                //  PWM Speed Control
  digitalWrite(M1, HIGH);   

  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);

  if (ms > 0) {
    delay(ms);
  }
}

//  Move backward 
void reverse (char a, char b, short ms = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);  

  analogWrite (E2, b);   
  digitalWrite(M2, LOW);

  if (ms > 0) {
    delay(ms);
  }
}

//  Turn Left
void turnLeft (char a, char b, short ms = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);   

  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);

  if (ms > 0) {
    delay(ms);
  }
}

//  Turn Right
void turnRight (char a, char b, short ms = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, HIGH);   

  analogWrite (E2, b);   
  digitalWrite(M2, LOW);

  if (ms > 0) {
    delay(ms);
  }
}

void setup(void) {
  int i;

  for(i = 4; i <= 7; i++) {
    pinMode(i, OUTPUT); 
  }

  //  Set the pan servo to home (front facing) position
  pan.attach(5);
  pan.writeMicroseconds(1500);

  //  Set Baud Rate
  Serial.begin(9600);

  delay(250);

  Serial.println("Run keyboard control");
}

void loop (void) {
  char val;

  if (Serial.available()) {
    val = Serial.read();

    if (val != -1) {
      switch(val) {
        case 'w':                     //  Move Forward
        case 'W':
          forward(100, 100);          //  Move forward at half speed
          break;

        case 's':                     //  Move Backward
        case 'S':
          reverse(100, 100);          //  Move back at half speed
          break;

        case 'a':                     //  Turn Left
        case 'A':
          turnLeft(50, 50);
          break;      

        case 'd':                     //  Turn Right
        case 'D':
          turnRight(50, 50);
          break;

        case 'z':
        case 'Z':
          Serial.println("Hello");
          break;

        case 'x':
        case 'X':
          stop();
          break;
      }
    } else {
      stop();
    }
  }
}
