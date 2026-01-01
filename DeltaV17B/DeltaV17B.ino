

//Libraries Needed
#include <HTTPClient.h>
#include <WiFi.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <TinyGPSPlus.h>
#include "base64.h"
#include <WiFiClientSecure.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "secrets.h" // Passwords 

#include <Preferences.h>
#include <ArduinoJson.h>
//Set Delay between fresh samples

#define BNO055_SAMPLERATE_DELAY_MS (150)
#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!"))

//Blade
#define RXD2 16
#define TXD2 17
int tempPower = 21;
int tempDCMotorLeft = 39;
int tempBlade = 34;



/*

   Verison 17 Delta Main
   Partner with IotaV12 Support
   Esp32 Code for GPS+Main Esp32
   Gardener GX

   GX Lawns 6/19/25
   Dinith Wijeratne


   Major Updates
    - IMU Intergration
    - Heading
    - Directions
    - Precission Waypoint Finder (PWF)
    - LED system
    - Point Perfect RTK Solution 
    - Adding Motor Connection 
    - Added Stuck Protection
        + Gradualy Increase
        + Inital Boost Projection
        + Dead Stall Protection 
    - GPS Error Correction
        + Fix Decection 
    - Blade Connection 
    - Lidar Bumpers 
    - Slope Correction





*/

//Checking I2C device address and correct line
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// GPS Object
SFE_UBLOX_GNSS myGNSS;
// Stepper Motor
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);
// Lidar


//Wifi Data
const char *firebase_base = "https://waypoint-7fafc-default-rtdb.firebaseio.com";
int tempLimit = 85;
bool transmitLocation = true;
unsigned int Actual_Millis, Previous_Millis;
long lastReceived_ms = 0;
long lastReceivedRTCM_ms = 0;  //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000;
int refresh_time = 500;
int turnangle;
int CourseNeeded;
int dc = 1;
int sa = 0;
int csa = 0;
int bb = 0;
int ze = 1;
int imuCalibrationSamples = 0;
int ft = 0;

int waypointIndex = 0;
int waypointIndexDatabase = 21;
int waypointArrivalCounter = 0;  //Uncomment for prescission WayPoint Finder
struct Waypoint {
  double lat;
  double lon;
  int cmd;
};
Waypoint waypoints[20];
Waypoint wp;
int waypointCount = 0;

bool overheatingProtection = false;
Preferences prefs;


//Info Var
int rotationCmd = 90;  // 0-90-180 Full 180 Degrees
int Speed = 0;         // 0-100
int lastSpeed = 0;
int rotationSpeed = 0;
int bladeHeight = 0;  // 0-255 PWM
int bladeSpeed = 0;   // Sets Blade speed 0-255
int preBladeSpeed = 0;
int currentBladeHeight = 0;
int objLidar = 0;
// Stuck Ration
int lastRotation = 10;
double lastDistance = 0;
int rotationBraking = 10;
int lastDis = 0;
int rotationIncreasingAmount = 0;
int counterForNotMoving = 0;
int bc = 0;
int rt = 0;
int carrSol = 2;
int horAcc = 80;
bool checkProtection = false;
int protectionCounter = 0;
int zeroZ = 0;
int zeroY = 0;

int dock = 0;
bool charging = false;
int convert = 0;

bool initialWaypointTurnAlignment = false;

String incomeAuxPacket;
int in;
int recoveryCounter = 0;
int lidarEvent = 0;
int releaseCounter = 0;

// GPS EC Data
int ec = 0;

bool stuckBoostActivate = false;
//Location Var
int WPlimit = 663;
double latitude;
double longtiude;
double localat = 0;
long fellaLat = 0;
double localong = 0;
double courseToWP;
int tempCycle = 0;
bool readingMotorLeft = false;
bool readingMotorRight = false;
bool readingBlade = false;
//double brng;
double distanceFTtoWP = 0;
double protectionDistanceLower = 0;
double protectionDistanceHigher = 0;
String latdata = "";
String longdata = "";
String tadata = "";
String longv = "";
String valid;
String dataQuery = "";
String va = "";
long lastTime = 0;
long lastSerial = 0;
int powerSwitch = 0;
uint8_t carrSoln;
bool navigationPaused = false;
int brakingCounter = 0;
bool brakingActivation = false;
int rotationBreaking = 0;

uint32_t hAcc;
//Motor Functions Pins

//Lidar Acting Middle (LAM)
int activateRotationStuck = 0;

//Motor Pins
const int motor1Pin1 = 32;  //26
const int motor1Pin2 = 26;
const int motor2Pin1 = 33;
const int motor2Pin2 = 27;

// Lidar Pins
//Relay Pins
int motorBraking_Flipping = 0;
double closerDistance = 0.3;

//Setting PWM freq
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 200;
const int pwmCh = 0;
const int pwmCh2 = 1;
int rangel = 0;
int rangel2 = 0;

bool bladeShutoff = false;

WiFiClient ntripClient;

//Lidar
int lidarAvoidanceLevel = 0;


//Print UBX_RXM_COR data
void printRXMCOR(UBX_RXM_COR_data_t *ubxDataStruct);

//Print UBX_NAV_PVT
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct);

void pushGPGGA(NMEA_GGA_data_t *nmeaData);

sensors_event_t event;

void setup() {
  delay(10);
  Serial.begin(115200);
  Serial.println("Verison 17B");
  Wire.begin();
  //Initialise IMU
  Serial.print("IMU Adafruit BNO055 Initialising...");
  Serial.println();
  while (!bno.begin()) {
    Serial.print("Error #501: Failed to connect to 0x28 I2C IMU, Checking Wiring or I2C ADDR!");
    delay(250);
  }
  delay(500);
  displayIMUdetail();
  displayIMUstatus();
  bno.setExtCrystalUse(true);

  // Connect to GPS

  while (myGNSS.begin() == false)  //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring."));
    delay(2000);
  }
  Serial.println(F("u-blox GNSS module connected"));
  //Firmwrae checker  
  if (myGNSS.getModuleInfo()) {
    Serial.print(F("FWVER: "));
    Serial.print(myGNSS.getFirmwareVersionHigh());  // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getFirmwareVersionLow());  // Returns uint8_t

    Serial.print(F("Firmware: "));
    Serial.println(myGNSS.getFirmwareType());  // Returns HPG, SPG etc. as (const char *)

    if (strcmp(myGNSS.getFirmwareType(), "HPG") == 0)
      if ((myGNSS.getFirmwareVersionHigh() == 1) && (myGNSS.getFirmwareVersionLow() < 30))
        Serial.println("Your module is running old firmware which may not support SPARTN. Please upgrade.");

    if (strcmp(myGNSS.getFirmwareType(), "HPS") == 0)
      if ((myGNSS.getFirmwareVersionHigh() == 1) && (myGNSS.getFirmwareVersionLow() < 21))
        Serial.println("Your module is running old firmware which may not support SPARTN. Please upgrade.");
  } else
    Serial.println(F("Error: could not read module info!"));

  //Now configure the module
  uint8_t ok = myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);                   //Turn off NMEA noise
  if (ok) ok = myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN);  // Be sure SPARTN input is enabled.

  if (ok) ok = myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);  // Set the differential mode - ambiguities are fixed whenever possible
  if (ok) ok = myGNSS.setNavigationFrequency(2);                          //Set output in Hz.
  if (ok) ok = myGNSS.setVal8(UBLOX_CFG_SPARTN_USE_SOURCE, 0);            // Use IP source (default). Change this to 1 for L-Band (PMP)

  if (ok) ok = myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);

  if (ok) ok = myGNSS.setNMEAGPGGAcallbackPtr(&pushGPGGA);  // Set up the callback for GPGGA

  if (ok) ok = myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 10);  // Tell the module to output GGA every 10 seconds

  if (ok) ok = myGNSS.setAutoPVTcallbackPtr(&printPVTdata);  // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed
  //if (ok) ok = myGNSS.setAutoPVT(true);

  if (ok) ok = myGNSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_COR_I2C, 1);  // Enable UBX-RXM-COR messages on I2C
  if (ok) ok = myGNSS.setRXMCORcallbackPtr(&printRXMCOR);            // Print the contents of UBX-RXM-COR messages so we can check if the SPARTN data is being decrypted successfully

  //if (ok) ok = myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save the ioPort and message settings to NVM

  Serial.print(F("GNSS: configuration "));
  Serial.println(OK(ok));

  // Setup Motors
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  //Setup Lidar
  pinMode(tempPower, OUTPUT);

  ledcSetup(pwmCh, freq, resolution);
  ledcSetup(pwmCh2, freq, resolution);

  ledcAttachPin(motor1Pin1, pwmCh);
  ledcAttachPin(motor2Pin1, pwmCh2);

  //  stepper.begin(RPM, MIRCOSTEPS);
  //  stepper.enable();


  //Start Duel Motors
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin2, LOW);


  Serial.println("Started the Sender");
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(50);


  //Connect to Wifi
  WiFi.begin(ssid, password);
  Serial.print("Connecting... ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected, my IP: ");
  Serial.println(WiFi.localIP());
  Actual_Millis = millis();
  Previous_Millis = Actual_Millis;

  // Get Waypoints
  Serial.println("Waypoint");
  delay(1500);
  getWP(waypointIndexDatabase, 20);  // 21

  delay(1500);
  wp = waypoints[waypointIndex];  // 0
  latitude = wp.lat;
  longtiude = wp.lon;
  initialWaypointTurnAlignment = true;
  Serial.print("Success");
  Serial.print(latitude, 7);
  Serial.print(", ");
  Serial.println(longtiude, 7);

  int cw = 0;
  while (bb == 1) {
    Serial.print("Retrying 1");
    bb = 0;
    getWP(waypointIndexDatabase, 20);  // 1
    wp = waypoints[waypointIndex];
    latitude = wp.lat;
    longtiude = wp.lon;
    delay(375);
    cw++;
    if (cw > 4) {
      break;
    }
  }
  Serial.print("Started RTK MQTT Client");
  beginClient();
}


void loop() {
  bno.getEvent(&event);
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();
}




void logic() {
  if (millis() - lastTime > 2) {  //50
    bno.getEvent(&event);
    if (imuCalibrationSamples < 1000) {  // Calibates IMU Senors Zeros
      zeroZ = event.orientation.z;
      zeroY = event.orientation.y;
      imuCalibrationSamples++;
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Not Connected");
      WiFi.begin(ssid, password);
      Serial.print("Connecting... ");
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
    }
    if (carrSoln == 0 && hAcc >= 800) {
      ec++;
    } else {
      ec = 0;
    }

    lastTime = millis();
    //Serial.print("Los Poluco");
    //Serial.println(localat, 8);
    //Serial.print(" ");
    //Serial.println(localong, 8);
    if (waypointIndexDatabase == WPlimit) {
      navigationPaused = true;
    }
    if (carrSoln >= carrSol && hAcc <= horAcc) {
      if (rt > 7200) {  // 300
        if (powerSwitch > 0) {
          Serial2.print(999);  //Turn 24v Battery ON
          Serial2.print("\n");
          delay(50);
          Serial2.print(889);  // Turn Fan ON
          Serial2.print("\n");
          powerSwitch = 0;
        }

        navigationPaused = false;
        carrSol = 1;
        horAcc = 300;
        distanceFTtoWP = distanceBetween(
          localat * 10000000,
          localong * 10000000,
          latitude * 10000000,
          longtiude * 10000000);
        if (distanceFTtoWP <= closerDistance) {  // Lowered for headrows
          Serial.print("You are there: ");
          Serial.print(waypointArrivalCounter);
          closerDistance = 0.8;
          waypointArrivalCounter++;
          navigationPaused = true;
          if (waypointArrivalCounter > 30) {  // Waits until location inside WP more than 4 times
            closerDistance = 0.3;
            waypointArrivalCounter = 0;
            Serial.print("Next Waypoint");
            waypointIndex++;  // Next Waypoint
            waypointIndexDatabase++;
            int batchSize = 20;
            if ((waypointIndexDatabase + batchSize) >= WPlimit) {
              batchSize = WPlimit - waypointIndexDatabase;
            }
            if (waypointIndex == 20) {
              getWP(waypointIndexDatabase, batchSize);
              delay(375);
              waypointIndex = 0;
            }
            wp = waypoints[waypointIndex];
            latitude = wp.lat;
            longtiude = wp.lon;
            initialWaypointTurnAlignment = true;
          }
        } else {
          navigationPaused = false;
          //Serial.print("Now False");
        }
        courseToWP = TinyGPSPlus::courseTo(localat, localong, latitude, longtiude);
        ///printFloat(courseToWP, true, 7, 2);
        //Serial.println();
        //Serial.println("Uploading Dataing ");
        // uploadData();
      } else {
        rt++;
        //Serial.print("RT: " + String(rt));
      }
    } else {
      if (powerSwitch == 0) {
        Serial2.print(998);  //Turn 24v Battery OFF
        Serial2.print("\n");
        delay(50);
        Serial2.print(888);  // Turn Fan OFF
        Serial2.print("\n");
        powerSwitch++;
      }
      navigationPaused = true;
      carrSol = 2;   // 2
      horAcc = 100;  // 80
    }
    if (tempCycle < 5) {
      digitalWrite(tempPower, HIGH);
      tempCycle++;
    }

    if (((tempCycle % 50) == 0) && tempCycle < 250) {
      readingMotorLeft = checkTemp(analogRead(tempDCMotorLeft));
      readingBlade = checkTemp(analogRead(tempBlade));
      if (readingMotorLeft || readingBlade) {
        navigationPaused = true;
        if (overheatingProtection == false) {
          tempLimit = 25;
          Serial2.print(998);  //Turn 24v Battery OFF
          Serial2.print("\n");
          delay(50);
          Serial2.print(888);  // Turn Fan OFF
          Serial2.print("\n");
          overheatingProtection = true;
        }
      } else if (overheatingProtection == true) {
        overheatingProtection = false;
        tempLimit = 85;
        Serial2.print(999);  //Turn 24v Battery ON
        Serial2.print("\n");
        delay(50);
        Serial2.print(889);  // Turn Fan ON
        Serial2.print("\n");
        navigationPaused = false;
      }
    }
    if (tempCycle == 250) {
      digitalWrite(tempPower, LOW);
    }
    if (tempCycle >= 1000) {
      tempCycle = 0;
    }
    if (overheatingProtection == true) {
      navigationPaused = true;
    }
    tempCycle++;
    //Serial.print("Heading: ");
    //Serial.print(event.orientation.x, 5);
    //Serial.println();
    CourseNeeded = (int)(360 + courseToWP - event.orientation.x) % 360;
    //Serial.println("Course Change Needed:");
    //Serial.print(CourseNeeded);
    //Serial.println();
    //processLidar();
    Speed = ConvertSpeed();
    CalCourse(CourseNeeded);  // Caluate Course Using BRNG
    //Debug only
    //Serial.print("Speed: ");
    //Serial.print(Speed);
    //Serial.print(" Rotation: ");
    //Serial.println(Rot);
    //Serial.print("RC: ");

    while (Serial2.available() >= 1) {
      incomeAuxPacket = Serial2.readStringUntil('\n');
      Serial.print("LidarDec: ");
      Serial.println(incomeAuxPacket);
      Serial.print(" incomeAuxPacket Bytes Size: ");
      Serial.println(sizeof(incomeAuxPacket));
      if (sizeof(incomeAuxPacket) >= 16) {
        break;
      }
    }

    //Lidar Pressed
    if (lidarEvent == 1) {
      rotationCmd = 999;
      Speed = 100;
      Serial.println("Backward Set by Left Lidar");
    }
    if (lidarEvent == 2) {
      rotationCmd = 999;
      Speed = 100;
      Serial.println("Backward Set by Right Lidar");
    }

    //Released
    if (lidarEvent == 3) {
      if (releaseCounter >= 6) {
        rotationCmd = 180;
        Speed = 100;
      }
      recoveryCounter++;
      releaseCounter++;
    }
    if (lidarEvent == 4) {
      if (releaseCounter >= 6) {
        rotationCmd = 0;
        Speed = 100;
      }
      recoveryCounter++;
      releaseCounter++;
    }

    if (recoveryCounter > 10) {
      Speed = 130;
      rotationCmd = 90;
      releaseCounter = 0;
      lidarEvent = 0;
      recoveryCounter++;
    }
    if (recoveryCounter > 14) {
      recoveryCounter = 0;
    }
    if (convert == 1) {
      sa = 1;
    }

    //String digonasticData = "Lidar Dignoastic Data: L = " + String(l) + " N: " + String(n) + " OD: " + String(od) + " Rot: " + String(Rot) + " Speed: " + String(Speed);
    //Serial.print(digonasticData);
    ///Serial.println("--End of digonasticData--");
    //Slope Protection
    int rf = RotationF();
    int SlopeSpeedChange = 0;
    if (event.orientation.y >= zeroY + 10 && event.orientation.y < 30) {
      switch (rf) {
        case 1:  //Full Left
          SlopeSpeedChange = 10;
          break;
        case 2:  // Full Right
          SlopeSpeedChange = -45;
          break;
        case 3:  // Slightly Left
          SlopeSpeedChange = 20;
          break;
        case 4:  //Slightly Right
          SlopeSpeedChange = -20;
          break;
      }
    }
    if (event.orientation.y <= zeroY - 10 && event.orientation.y > -30) {
      switch (rf) {
        case 1:  //Full Left
          SlopeSpeedChange = -35;
          break;
        case 2:  // Full Right
          SlopeSpeedChange = 10;
          break;
        case 3:  // Slightly Left
          SlopeSpeedChange = -20;
          break;
        case 4:  //Slightly Right
          SlopeSpeedChange = 20;
          break;
      }
    }
    if (navigationPaused == false && ft < 1) {
      Speed += SlopeSpeedChange;
      ft++;
    }


    // End of Slope Protection
    // Stuck Protection
    //Incase Stuck maneuvers fails
    if (checkProtection == true && Speed > 0) {
      if (distanceFTtoWP > protectionDistanceLower && distanceFTtoWP < protectionDistanceHigher) {
        protectionCounter++;
      } else {
        checkProtection = false;
        protectionCounter = 0;
      }
      if (protectionCounter >= 500) {
        Speed = 0;
        navigationPaused = true;
      }
    }

    if (round(distanceFTtoWP) == round(lastDis) && Speed >= 50 && navigationPaused == false) {
      counterForNotMoving++;
      //Serial.print("Ds");
      //Serial.println(ds);
    } else {
      counterForNotMoving = 0;
      stuckBoostActivate = false;
    }

    int rotationCurrent = 0;
    rotationCurrent = RotationF();


    if ((counterForNotMoving > 500 && ((counterForNotMoving % 375) == 0)) || counterForNotMoving == 3000) {
      MotorCustomMove(0, 0, false, true);
      delay(100);
    }
    if (counterForNotMoving >= 1000 && counterForNotMoving < 2000) {
      if (Speed == 150) {
        Speed += 15;
      }
      //Serial.print("Increase 15");
    }
    if (counterForNotMoving >= 2000 && counterForNotMoving < 3000) {
      if (Speed == 150) {
        Speed += 30;
      }
      //Serial.print("Increase 30");
    }
    if (counterForNotMoving > 3000) {
      Speed = 200;
      stuckBoostActivate = true;
      counterForNotMoving = 0;
      checkProtection = true;
      protectionDistanceLower = distanceFTtoWP - 0.2;
      protectionDistanceHigher = distanceFTtoWP + 0.2;
      activateRotationStuck++;
    }
    //Gives A Boost for turning left or right then resets back to 105. Then slowly increases speed the longer turn left or right is commanded
    if (lastRotation == rotationCurrent && navigationPaused == false && ((rotationIncreasingAmount - 985) < 170) && sa == 0) {
      int rotationCmd = rotationCurrent;
      if (rotationCmd == 1 || rotationCmd == 2) {
        rotationIncreasingAmount += 5;
        if ((lastRotation == 1 || lastRotation == 2) && (rotationIncreasingAmount > 500 && (rotationIncreasingAmount - bc) != 500)) {  //If right or left and rotationIncreasingAmount > 5 to set right left back from inital jump
          Speed = 105;
          ft = 0;
        } else if ((rotationIncreasingAmount - bc) == 5000) {
          bc = rotationIncreasingAmount;
        }
        if (rotationIncreasingAmount >= 1000) {
          Speed += rotationIncreasingAmount - 985;
        }
      }
      if (activateRotationStuck >= 5 && activateRotationStuck <= 20) {
        rotationCmd = 999;
        Speed = 120;
        activateRotationStuck++;
      }
    } else {
      rotationIncreasingAmount = 0;
      stuckBoostActivate = false;
      activateRotationStuck = 0;
    }

    if (lidarAvoidanceLevel > 0 && (rotationCurrent == 1 || rotationCurrent == 2) && navigationPaused == false) {
      switch (lidarAvoidanceLevel) {
        case 1:
          Speed -= 35;
          break;
        case 2:
          Speed -= 35;
          break;
        case 3:
          Speed -= 50;
          break;
        case 4:
          Speed -= 50;
          break;
        case 5:
          rotationCmd = 90;
          Speed = 110;
          break;
        case 6:
          rotationCmd = 90;
          Speed = 110;
          break;
      }
    }
    //EMF Backing Prevention.
    if (rotationCurrent == 1 || rotationCurrent == 2) {
      switch (rotationCurrent) {
        case 1:
          if (lastRotation != 1) {
            Speed = 0;
            motorBraking_Flipping = 1;
          }

          break;
        case 2:
          if (lastRotation != 2) {
            Speed = 0;
            motorBraking_Flipping = 1;
          }
          break;
      }
      if (motorBraking_Flipping <= 30 && motorBraking_Flipping >= 1) {
        Speed = 0;
        motorBraking_Flipping++;
      } else {
        motorBraking_Flipping = 0;
      }
    } else {
      motorBraking_Flipping = 0;
    }
    Serial.println(rotationIncreasingAmount);

    //Serial.print("Motor Turning to: ");
    if (Speed >= 240) {
      Speed = 220;
    }
    if (stuckBoostActivate == true && navigationPaused == false) {
      Speed = 230;
    }
    bladeShutoff = false;
    objDetection(incomeAuxPacket, rotationCurrent, lastRotation);
    // incomeAuxPacket = "0";
    if (initialWaypointTurnAlignment == true) {
      turningRot(CourseNeeded);
    }
    //Calucates if mower is moving past Speed=0 when Speed was > 0 indicating mower moving due to inertia
    if ((lastSpeed > 0 && Speed == 0) || brakingActivation == true) {
      if (brakingCounter == 0) {
        rotationBraking = lastRotation;
        brakingActivation = true;
        rotationSpeed = lastSpeed;
        lastDistance = distanceFTtoWP;
      }

      if (brakingCounter > 5) {
        if (distanceFTtoWP < lastDistance + 0.3 || distanceFTtoWP > lastDistance - 0.3) {
          brakingActivation = false;
          brakingCounter = 0;
        }
      }
      if (event.orientation.z >= zeroZ + 8) {
        if (rotationBraking == 0) {
          rotationBraking = 5;
          brakingCounter = 15;
        }
      }
      //Stops exptected inertia
      if (brakingCounter <= 35) {
        switch (rotationBraking) {  //  0 if Straight | 1 = Left | 2 = Right | 3 = Mixed Right | 4 = Mixed Left | 5 = Backward | 6 = HP Left Back | 7 = HP Right Back | 8 = Slightly Left B | 9 = Slightly Right B
          case 2:
            MotorCustomMove(rotationSpeed, rotationSpeed, true, false);
            //Serial.print("Breaking Right by moving left");
            break;
          case 1:
            MotorCustomMove(rotationSpeed, rotationSpeed, false, true);
            // Serial.print("Breaking Left by moving Right");
            break;
          case 0:
            MotorCustomMove(rotationSpeed, rotationSpeed, true, true);
            //Serial.print("Breaking backwards by moving fwd");
            break;
          case 5:
            MotorCustomMove(rotationSpeed, rotationSpeed, false, false);
            // Serial.print("Moving Stairght");
            break;
        }
        brakingCounter++;
      } else if (brakingCounter > 35) {
        brakingActivation = false;
        brakingCounter = 0;
      }
    }
    lastSpeed = Speed;
    lastRotation = rotationCurrent;
    lastDis = distanceFTtoWP;
    if (brakingActivation == false) {
      switch (rotationCurrent) {  //  0 if Straight | 1 = Left | 2 = Right | 3 = Mixed Right | 4 = Mixed Left | 5 = Backward | 6 = HP Left Back | 7 = HP Right Back | 8 = Slightly Left B | 9 = Slightly Right B
        case 1:
          MotorCustomMove(Speed, Speed, true, false);
          Serial.print("Turning Left");
          break;
        case 2:
          MotorCustomMove(Speed, Speed, false, true);
          Serial.print("Turning Right");
          break;
        case 3:
          MotorCustomMove(Speed + 25, Speed - 5, false, false);
          Serial.print("Slight to Right");
          break;
        case 4:
          MotorCustomMove(Speed - 5, Speed + 25, false, false);
          Serial.print("Slight to Left");
          break;
        case 5:
          MotorCustomMove(Speed - 10, Speed - 10, true, true);
          Serial.print("Backwards");
          break;
        case 6:
          MotorCustomMove(Speed, Speed - 10, true, true);
          Serial.print("HP Left Back");
          break;
        case 7:
          MotorCustomMove(Speed - 10, Speed, true, true);
          Serial.print("HP Right Back");
          break;
        case 8:
          MotorCustomMove(Speed + 10, Speed - 10, true, true);
          Serial.print("Slightly Left Back");
          break;
        case 9:
          MotorCustomMove(Speed - 10, Speed + 10, true, true);
          Serial.print("Slight Right Back");
          break;
        case 0:
          MotorCustomMove(Speed, Speed, false, false);
          Serial.print("Moving Stairght");
          break;
      }
    }
    Serial.println();
    //Serial.println("Uploading Data ");

    // uploadData();
  }
}


void beginClient() {
  long rtcmCount = 0;

  Serial.println(F("Subscribing to Caster. Press key to stop"));
  delay(10);                                 //Wait for any serial to arrive
  while (Serial.available()) Serial.read();  //Flush

  while (Serial.available() == 0) {
    //Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false) {
      Serial.print(F("Opening socket to "));
      Serial.println(casterHost);

      if (ntripClient.connect(casterHost, casterPort) == false)  //Attempt connection
      {
        Serial.println(F("Connection to caster failed"));
        return;
      } else {
        Serial.print(F("Connected to "));
        Serial.print(casterHost);
        Serial.print(F(": "));
        Serial.println(casterPort);

        Serial.print(F("Requesting NTRIP Data from mount point "));
        Serial.println(mountPoint);

        const int SERVER_BUFFER_SIZE = 512;
        char serverRequest[SERVER_BUFFER_SIZE + 1];

        snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
                 mountPoint);

        char credentials[512];
        if (strlen(casterUser) == 0) {
          strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
        } else {
          //Pass base64 encoded user:pw
          char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1];  //The ':' takes up a spot
          snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

          Serial.print(F("Sending credentials: "));
          Serial.println(userCredentials);

#if defined(ARDUINO_ARCH_ESP32)
          //Encode with ESP32 built-in library
          base64 b;
          String strEncodedCredentials = b.encode(userCredentials);
          char encodedCredentials[strEncodedCredentials.length() + 1];
          strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials));  //Convert String to char array
          snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
          //Encode with nfriendly library
          int encodedLen = base64_enc_len(strlen(userCredentials));
          char encodedCredentials[encodedLen];                                          //Create array large enough to house encoded data
          base64_encode(encodedCredentials, userCredentials, strlen(userCredentials));  //Note: Input array is consumed
#endif
        }
        strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
        strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

        Serial.print(F("serverRequest size: "));
        Serial.print(strlen(serverRequest));
        Serial.print(F(" of "));
        Serial.print(sizeof(serverRequest));
        Serial.println(F(" bytes available"));

        Serial.println(F("Sending server request:"));
        Serial.println(serverRequest);
        ntripClient.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripClient.available() == 0) {
          if (millis() - timeout > 5000) {
            Serial.println(F("Caster timed out!"));
            ntripClient.stop();
            return;
          }
          delay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripClient.available()) {
          if (responseSpot == sizeof(response) - 1) break;

          response[responseSpot++] = ntripClient.read();
          if (strstr(response, "200") != nullptr)  //Look for 'ICY 200 OK'
            connectionSuccess = true;
          if (strstr(response, "401") != nullptr)  //Look for '401 Unauthorized'
          {
            Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
            connectionSuccess = false;
          }
        }
        response[responseSpot] = '\0';

        Serial.print(F("Caster responded with: "));
        Serial.println(response);

        if (connectionSuccess == false) {
          Serial.print(F("Failed to connect to "));
          Serial.print(casterHost);
          Serial.print(F(": "));
          Serial.println(response);
          return;
        } else {
          Serial.print(F("Connected to "));
          Serial.println(casterHost);
          lastReceivedRTCM_ms = millis();  //Reset timeout
        }
      }  //End attempt to connect
    }    //End connected == false

    if (ntripClient.connected() == true) {
      uint8_t rtcmData[512 * 4];  //Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      //Print any available RTCM data
      while (ntripClient.available()) {
        //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
        rtcmData[rtcmCount++] = ntripClient.read();
        if (rtcmCount == sizeof(rtcmData)) break;
      }

      if (rtcmCount > 0) {
        lastReceivedRTCM_ms = millis();

        //Push RTCM to GNSS module over I2C
        myGNSS.pushRawData(rtcmData, rtcmCount, false);
        Serial.print(F("RTCM pushed to ZED: "));
        Serial.println(rtcmCount);
      }
    }

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      if (ntripClient.connected() == true)
        ntripClient.stop();
      return;
    }
    myGNSS.checkUblox();  // Check for the arrival of new GNSS data and process it.
    myGNSS.checkCallbacks();
    logic();

    delay(10);
  }

  Serial.println(F("User pressed a key"));
  Serial.println(F("Disconnecting..."));
  ntripClient.stop();
}


int ConvertSpeed() {
  float spd = Speed * 2.55;
  return (int)spd;
}
//////////////////////////
//Uploads Data to Server//
//////////////////////////
void uploadData(int h, int f, int r, String lat, String lon) {
  Actual_Millis = millis();
  if (Actual_Millis - Previous_Millis > refresh_time) {
    Previous_Millis = Actual_Millis;
    //Wifi Connnected
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient data;
      String url = String(firebase_base) + "/LOC_wp/1/cmd.json";
      data.begin(url);
      data.addHeader("Content-Type", "application/json");
      String res = String(h) + "_" + String(f) + "_" + String(r) + "_" + lat + "_" + lon;
      int code = data.PUT(res);
      if (code > 0) {
        Serial.println("Good: " + String(code));
      } else {
        Serial.println("Error: " + String(code));
      }
      data.end();
    }
  }
}
void getWP(int startIndex, int batchSize) {
  /*
    getWP() Method
  */
  Actual_Millis = millis();
  int karley = 0;
  if (Actual_Millis - Previous_Millis > refresh_time) {
    Previous_Millis = Actual_Millis;
    //WIFI Connected
    if (WiFi.status() == WL_CONNECTED) {
      char url[256];
      snprintf(url, sizeof(url),
               "https://waypoint-7fafc-default-rtdb.firebaseio.com/LOC_wp.json?orderBy=\"$key\"&startAt=\"%d\"&limitToFirst=%d",
               startIndex, batchSize);


      HTTPClient http;
      http.begin(url);
      int httpCode = http.GET();

      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();

        StaticJsonDocument<8192> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
          Serial.print("JSON Error: ");
          Serial.println(error.c_str());
          return;
        }

        waypointCount = 0;
        for (JsonPair kv : doc.as<JsonObject>()) {
          if (waypointCount >= batchSize) break;
          JsonObject wp = kv.value();
          waypoints[waypointCount].lat = wp["lat"];
          waypoints[waypointCount].lon = wp["lon"];
          waypoints[waypointCount].cmd = wp["cmd"];
          Serial.printf("Loaded WP[%d]: lat=%.7f lon=%.7f cmd=%d\n",
                        waypointCount,
                        waypoints[waypointCount].lat,
                        waypoints[waypointCount].lon,
                        waypoints[waypointCount].cmd);
          waypointCount++;
        }

        Serial.print("Pulled waypoints from index ");
        Serial.println(startIndex);
        //saveWaypointsToNVS();  //
      } else {
        Serial.print("HTTP error: ");
        Serial.println(httpCode);
      }

      http.end();
    } else {
      Serial.println("WIFI connection error #021");  // Could not connect to WIFI
    }
  }
}
/* For offline use 
void saveWaypointsToNVS() {
  prefs.begin("wps", false);
  prefs.putBytes("waypoints", waypoints, sizeof(waypoints));
  prefs.putUInt("wpCount", waypointCount);
  prefs.end();
}
void loadWaypointsFromNVS() {
  prefs.begin("wps", true);
  prefs.getBytes("waypoints", waypoints, sizeof(waypoints));
  waypointCount = prefs.getUInt("wpCount", 0);
  prefs.end();
}
*/

/////////////////
//Get Location///
/////////////////
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
  localat = ubxDataStruct->lat;  // Print the latitude
  Serial.print(F("Lat: "));
  Serial.print(localat / 10000000.0, 7);
  localat = localat / 10000000.0;

  localong = ubxDataStruct->lon;  // Print the longitude
  Serial.print(F("  Long: "));
  Serial.print(localong / 10000000.0, 7);
  localong = localong / 10000000.0;
  /*
    double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
    Serial.print(F("  Height: "));
    Serial.print(altitude / 1000.0, 3);
  */
  uint8_t fixtype = ubxDataStruct->fixType;  // Print the fix type
  Serial.print(F("  Fix: "));
  Serial.print(fixtype);
  if (fixtype == 0)
    Serial.print(F(" (None)"));
  else if (fixtype == 1)
    Serial.print(F(" (Dead Reckoning)"));
  else if (fixtype == 2)
    Serial.print(F(" (2D)"));
  else if (fixtype == 3)
    Serial.print(F(" (3D)"));
  else if (fixtype == 3)
    Serial.print(F(" (GNSS + Dead Reckoning)"));
  else if (fixtype == 5)
    Serial.print(F(" (Time Only)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  carrSoln = ubxDataStruct->flags.bits.carrSoln;  // Print the carrier solution
  Serial.print(F("  Carrier Solution: "));
  Serial.print(carrSoln);
  if (carrSoln == 0)
    Serial.print(F(" (None)"));
  else if (carrSoln == 1)
    Serial.print(F(" (Floating)"));
  else if (carrSoln == 2)
    Serial.print(F(" (Fixed)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  hAcc = ubxDataStruct->hAcc;  // Print the horizontal accuracy estimate
  Serial.print(F("  Horizontal Accuracy Estimate: "));
  Serial.print(hAcc);
  Serial.print(F(" (mm)"));

  Serial.println();
}

void pushGPGGA(NMEA_GGA_data_t *nmeaData) {
  //Provide the caster with current position as needed
  if ((ntripClient.connected() == true) && (transmitLocation == true)) {
    Serial.print(F("Pushing GGA to server: "));
    Serial.print((const char *)nmeaData->nmea);  // .nmea is printable (NULL-terminated) and already has \r\n on the end

    //Push our current GGA sentence to caster
    ntripClient.print((const char *)nmeaData->nmea);
  }
}

void printRXMCOR(UBX_RXM_COR_data_t *ubxDataStruct) {
  if (ubxDataStruct->statusInfo.bits.protocol == 1)
    Serial.print(F("RTCM3"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 2)
    Serial.print(F("SPARTN"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 29)
    Serial.print(F("PMP (SPARTN)"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 30)
    Serial.print(F("QZSSL6"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  errStatus: "));
  if (ubxDataStruct->statusInfo.bits.errStatus == 1)
    Serial.print(F("Error-free"));
  else if (ubxDataStruct->statusInfo.bits.errStatus == 2)
    Serial.print(F("Erroneous"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgUsed: "));
  if (ubxDataStruct->statusInfo.bits.msgUsed == 1)
    Serial.print(F("Not used"));
  else if (ubxDataStruct->statusInfo.bits.msgUsed == 2)
    Serial.print(F("Used"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgEncrypted: "));
  if (ubxDataStruct->statusInfo.bits.msgEncrypted == 1)
    Serial.print(F("Not encrypted"));
  else if (ubxDataStruct->statusInfo.bits.msgEncrypted == 2)
    Serial.print(F("Encrypted"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgDecrypted: "));
  if (ubxDataStruct->statusInfo.bits.msgDecrypted == 1)
    Serial.print(F("Not decrypted"));
  else if (ubxDataStruct->statusInfo.bits.msgDecrypted == 2)
    Serial.print(F("Successfully decrypted"));
  else
    Serial.print(F("Unknown"));

  Serial.println();
}

/////////////////////
//IMU Sensor Method//
/////////////////////

void displayIMUdetail(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayIMUstatus(void) {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displayCalStatus(void) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
void CalcRot(int h, int dr) {  // Backward Rotations when given heading and desisred heading .
  Serial.print("H:");
  Serial.println(h);
  if (h > dr + 3) {     // Heading is to the Right by 3 Need to movet to the left
    rotationCmd = 981;  // To the Right
    Speed = 125;
  } else if (h < dr - 3) {  // Heading to the Left
    rotationCmd = 982;      // HP Left
    Speed = 125;
  } else if (h > dr + 8) {  // Heading to Right by 8
    rotationCmd = 971;      // Slight right
    Speed = 125;
  } else if (h < dr - 8) {  // Heading to the Left by 8
    rotationCmd = 972;      // Slight L eft
    Speed = 125;
  } else if (h > dr + 15) {
    rotationCmd = 180;  // Turn Right
    Speed = 125;
  } else if (h < dr - 15) {
    rotationCmd = 0;  // Turn Left
    Speed = 125;
  } else {
    rotationCmd = 999;
    Speed = 125;
  }
}

int RotationF() {  // Return 0 if Straight | 1 = Left | 2 = Right | 3 = Mixed Right | 4 = Mixed Left | 5 = Backward | 6 = HP Left Back | 7 = HP Right Back | 8 = Slightly Left B | 9 = Slightly Right B
  if (rotationCmd > 110 && rotationCmd <= 180) {
    return 2;  // Turn Right
  } else if (rotationCmd > 90 && rotationCmd <= 110) {
    return 3;  // Mixed Right
  } else if (rotationCmd < 90 && rotationCmd >= 70) {
    return 4;  // Mixed Left
  } else if (rotationCmd < 70) {
    return 1;  // Turn Left
  } else if (rotationCmd == 999) {
    return 5;  // Backward
  } else if (rotationCmd == 982) {
    return 6;  // HP left B
  } else if (rotationCmd == 981) {
    return 7;  // HP Right B
  } else if (rotationCmd == 972) {
    return 8;  // SLight Left B
  } else if (rotationCmd == 971) {
    return 9;  // Slight Right B
  } else {
    return 0;  // Straight
  }
}

void objDetection(String in, int r, int lr) {
  int a = in.toInt();
  lidarAvoidanceLevel = 0;
  Serial.print("Object Detected: ");  // Add Comments and notes and clean the code PLEASE
  Serial.println(a);
  if (navigationPaused == false) {
    if (r == 2 && lr == 2) {
      switch (a) {
        case 1:  // Too Close
          Speed = 0;
          bladeShutoff = true;
          break;
        case 2:  // Catch
          Speed = 0;
          bladeShutoff = true;
          break;
        case 3:  // Offset
          Speed = 0;
          bladeShutoff = true;
          break;
        case 4:  // Birth
          //Speed = 80;
          break;
        case 5:  // Probliamatic
          //Speed = 100;
          break;
      }
    }
    if (r == 1 && lr == 1) {
      switch (a) {
        case 11:  // Too Close
          Speed = 0;
          bladeShutoff = true;
          break;
        case 12:  // Catch
          Speed = 0;
          bladeShutoff = true;
          break;
        case 13:  // Offset
          Speed = 0;
          bladeShutoff = true;
          break;
        case 14:  // Birth
          //Speed = 80;
          break;
        case 15:  // Probliamatic
          //Speed = 100;
          break;
      }
    }
    switch (a) {
      case 7:  // Too Close
        Speed = 0;
        bladeShutoff = true;
        break;
      case 8:  // Catch
        bladeShutoff = false;
        break;
      case 9:  // Offset
        Speed = 0;
        bladeShutoff = true;
        break;
      case 10:  // Birth
        bladeShutoff = false;
        break;
      default:
        break;
    }
  }
}


void turningRot(int CourseNeeded) {
  int bSpeed = 0;
  //  ft = 0;
  if (CourseNeeded >= 359 || CourseNeeded <= 1) {
    Serial.println("In Line");  //Middle LED
    //digitalWrite(mled, HIGH);

    initialWaypointTurnAlignment = false;
    Speed = 0;

  } else if (CourseNeeded >= 180 && CourseNeeded < 359) {  // 345
    Serial.println("Turn Left");       //  ||1|| 2 3 4 5 (First Led)
    //digitalWrite(leftled, HIGH);

    //Speed
    Speed = 150;
    rotationCmd = 0;
    bSpeed = 0;
  } else if (CourseNeeded > 1 && CourseNeeded < 180) {  // 105
    Serial.println("Turn Right");   // 1 2 3 4 || 5 || (Fifth LED)
    //digitalWrite(rightled, HIGH);

    //Speed
    Speed = 150;
    rotationCmd = 180;
    bSpeed = 0;
  }
  if (navigationPaused == true) {
    Speed = 0;
    rotationCmd = 90;
    bSpeed = 0;
    Serial.print("This is beening navigationPaused");
    // digitalWrite(leftled, HIGH);
    //digitalWrite(hpleft, HIGH);
    //digitalWrite(hpright, HIGH);
    // digitalWrite(rightled, HIGH);
    //digitalWrite(mled, HIGH);
  }
  if (millis() - lastSerial > 500) {
    if (bladeShutoff == true) {
      bSpeed = 0;
    }
    Serial.print("Blade Speed");
    Serial.println(bSpeed);
    Serial2.print((String)bSpeed);
    Serial2.print("\n");
    lastSerial = millis();
  }
}

void CalCourse(int CourseNeeded) {
  int bSpeed = 0;
  ft = 0;
  //int CourseNeeded = (int)(360 + ct - h) % 360; // Finds Course Change Needed

  //leftled, hpleft, mled, hpright, rightled

  if (CourseNeeded >= 358 || CourseNeeded < 2) {
    Serial.println("In Line");  //Middle LED
    //digitalWrite(mled, HIGH);

    //Speed
    Speed = 150;  // 70
    rotationCmd = 90;
    bSpeed = 50;

  } else if (CourseNeeded >= 355 && CourseNeeded < 358) {
    Serial.println("HP Left");  // 1 ||2 3|| 4 5 (2 and 3 led)
    //digitalWrite(mled, HIGH);
    //digitalWrite(hpleft, HIGH);

    //Speed
    Speed = 150;
    rotationCmd = 80;
    bSpeed = 50;

  } else if (CourseNeeded >= 2 && CourseNeeded < 5) {
    Serial.println("HP Right");  // 1 2 ||3 4|| 5 (3 and 4 led)
    //digitalWrite(mled, HIGH);
    //digitalWrite(hpright, HIGH);

    //Speed
    Speed = 150;
    rotationCmd = 100;
    bSpeed = 50;

  } else if (CourseNeeded >= 340 && CourseNeeded < 355) {
    Serial.println("Slight Left");  // 1 2 3 ||4|| 5 (4th led)
    //digitalWrite(hpleft, HIGH);

    //Speed
    Speed = 150;
    rotationCmd = 80;
    bSpeed = 50;

  } else if (CourseNeeded >= 5 && CourseNeeded < 20) {
    Serial.println("Slight Right");  // 1 ||2|| 3 4 5 (2nd led)
    //digitalWrite(hpright, HIGH);

    //Speed
    Speed = 150;
    rotationCmd = 120;
    bSpeed = 50;
  } else if (CourseNeeded >= 180 && CourseNeeded < 340) {  // 345
    Serial.println("Turn Left");       //  ||1|| 2 3 4 5 (First Led)
    //digitalWrite(leftled, HIGH);

    //Speed
    Speed = 190;
    rotationCmd = 0;
    bSpeed = 0;
  } else if (CourseNeeded >= 20 && CourseNeeded < 180) {  // 105
    Serial.println("Turn Right");     // 1 2 3 4 || 5 || (Fifth LED)
    //digitalWrite(rightled, HIGH);

    //Speed
    Speed = 190;
    rotationCmd = 180;
    bSpeed = 0;

  } else {
    Serial.println("Turn Around");  // All LEDs Active but Middle LED
    Speed = 190;
    rotationCmd = 180;
    bSpeed = 0;
    //digitalWrite(leftled, HIGH);
    //digitalWrite(hpleft, HIGH);
    //digitalWrite(hpright, HIGH);
    //digitalWrite(rightled, HIGH);
  }
  if (navigationPaused == true) {
    Speed = 0;
    rotationCmd = 90;
    bSpeed = 0;
    Serial.print("navigationPaused");
    // digitalWrite(leftled, HIGH);
    //digitalWrite(hpleft, HIGH);
    //digitalWrite(hpright, HIGH);
    // digitalWrite(rightled, HIGH);
    //digitalWrite(mled, HIGH);
  }
  if (millis() - lastSerial > 500) {
    if (bladeShutoff == true) {
      bSpeed = 0;
    }
    Serial.print("Blade Speed");
    Serial.println(bSpeed);
    Serial2.print((String)bSpeed);
    Serial2.print("\n");
    lastSerial = millis();
  }
}

bool checkTemp(float reading) {
  int seriesResistor = 10000;
  reading = (4095 / reading) - 1;      // (1023/ADC - 1)
  reading = seriesResistor / reading;  // 10K / (1023/ADC - 1)
  Serial.print("Thermistor resistance ");
  Serial.println(reading);
  float steinhart;
  steinhart = reading / 10000;       // (R/Ro)
  steinhart = log(steinhart);        // ln(R/Ro)
  steinhart /= 3950;                 // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;       // Invert
  steinhart -= 273.15;
  Serial.print("Tempature");
  Serial.println(steinhart);
  if (steinhart > tempLimit) {
    return true;
  } else {
    return false;
  }
}

void MotorCustomMove(int mp1, int mp2, bool mp1d, bool mp2d) {
  int m1pd = mp1d ? 1 : 0;  //If true m1pd == True else m1pd == false
  int m2pd = mp2d ? 1 : 0;

  Serial.print("Motor1 Set to: ");
  Serial.println(mp1);
  Serial.print("Motor2 Set to: ");
  Serial.println(mp2);

  digitalWrite(motor1Pin2, m1pd);
  digitalWrite(motor2Pin2, m2pd);

  ledcWrite(pwmCh, mp1);
  ledcWrite(pwmCh2, mp2);
}

//////////////////
//Convert Values//
//////////////////
double todouble(String &str) {
  return atof(str.c_str());
}

double Todouble(String str) {
  return atof(str.c_str());
}
/*
  double bearingTo(double lat1, double long1, double lat2, double long2){
  double y = sin(long2 - long1) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(long2 - long1);
  float r = atan2(y, x);
  r = (r+180) % 360;
  int brng = (r*180/PI + 360);
  Serial.print(r);
  Serial.println();
  return brng;

  }
*/
////////////////
//Print Vaules//
////////////////
static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  //  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid) {
    sprintf(sz, "%ld", val);
  }
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i) {
    sz[i] = ' ';
  }
  if (len > 0) {
    sz[len - 1] = ' ';
  }
  Serial.print(sz);
  //smartDelay(0);
}


double distanceBetween(long lat1_l, long long1_l, long lat2_l, long long2_l) {
  // Courtesy of Maarten Lamers
  double lat1 = (double)lat1_l / 10000000.0;  // Convert lat and long to degrees
  double long1 = (double)long1_l / 10000000.0;
  double lat2 = (double)lat2_l / 10000000.0;
  double long2 = (double)long2_l / 10000000.0;
  double delta = radians(long1 - long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);

  return delta * 20902335.8;  //Returns in FT
}
