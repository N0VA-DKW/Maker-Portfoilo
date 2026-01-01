//Libraries needed 
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Wire.h>
#include <ezButton.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

#include "Adafruit_VL53L0X.h"
#define RXD2 16
#define TXD2 17

SFE_MAX1704X lipo;
TaskHandle_t lidarTask;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

Servo servo;  // create servo object to control a servo
Servo bladeMotor;
ezButton leftLimit(25);
ezButton rightLimit(27);
ezButton backLimit(5);


volatile int range1 = -1;
volatile int range2 = -1;

//Battery
const int checkBattery = 33;
const int ledL = 13;
const int ledM = 35;
const int ledR = 34;
const int Relay24 = 33;
const int RelayFan = 13;
const int lipoBat = 14;
int valL = 444;
int valM = 555;
int valR = 666;  // t
int c = 0;
int d = 50;
int lr = 0;
int mr = 0;
int rr = 0;

int stateDec = 0;
int lastState = 0;
bool NoBatteryCheck = false;

long lastBatteryCheck = 0;

int lastServo = 0;
int servoMove = 0;
volatile bool lidarRunning = false;
int lastL = 0;
int ldr = 0;
int lastSent = 0;
//Lipo

double voltage = 0;  // Variable to keep track of LiPo voltage
double soc = 0;      // Variable to keep track of LiPo state-of-charge (SOC)
bool alert;          // Variable to keep track of whether alert has been triggered

int dock = 0;
int bl = 0;
int t = 0;


int lidarPinX1 = 4;  //BackUp is 13
int lidarPinX2 = 23;
int LidarDec = 0;

// 16 servo objects can be created on the ESP32
bool called = false;

int bladePin = 32;

int pin = 33;


int pos = 0;  // variable to store the servo position

int servoPin = 26;

//Wifi Data
const char* ssid = "CAPPUCCINO";
const char* password = "aselaNetwork";
//Delay
unsigned int Actual_Millis, Previous_Millis;
int refresh_time = 500;


int bladeHeight = 250;
int railHeight = 100;
int bladeSpeed = 0;



//Setting PWM freq
const int freq = 25000;
const int resolution = 8;
int dutyCycle = 200;
const int pwmCh = 4;

//Setting PWM Battt
const int battfreq = 5000;
const int batChannel = 0;
const int res = 8;

String incoming;

/*
  Iota V12
  Partner Controller to DeltaV17B

    Dinith Wijeratne

  6/19/25
  




  Controlers Blades, Limits, Servo/Rails


*/


void lidarLoop(void* parameter) {
  while (1) {
    if (lidarRunning == true) {
      int r1 = lox.readRange();
      int r2 = lox2.readRange();

      if (!lox.timeoutOccurred() && !lox2.timeoutOccurred()) {
        range1 = r1;
        range2 = r2;
      }

      vTaskDelay(150 / portTICK_PERIOD_MS);  // every 100ms
    }
  }
}

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);           // standard 50 hz servo
  servo.attach(servoPin, 500, 2500);  // attaches the servo on pin 18 to the servo object

  // Serial.println("Servo Moved");
  //Setup Blade
  bladeMotor.setPeriodHertz(50);
  bladeMotor.attach(bladePin, 900, 2100);

  //Setup Bumper

  leftLimit.setDebounceTime(75);
  rightLimit.setDebounceTime(75);
  backLimit.setDebounceTime(75);

  //Setup Lidar
  //ledcSetup(batChannel, battfreq, res);
  //ledcAttachPin(checkBattery, batChannel);
  Serial.begin(115200);
  pinMode(lidarPinX1, OUTPUT);
  pinMode(lidarPinX2, OUTPUT);
  digitalWrite(lidarPinX1, LOW);
  digitalWrite(lidarPinX2, LOW);
  delay(10);
  digitalWrite(lidarPinX1, HIGH);
  digitalWrite(lidarPinX2, HIGH);
  delay(50);
  digitalWrite(lidarPinX2, LOW);

  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin(0x30)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  Serial.print("Tests");
  digitalWrite(lidarPinX2, HIGH);

  if (!lox2.begin(0x31)) {
    Serial.println(F("Failed to boot VL53L0Xs"));
    while (1)
      ;
  }





  pinMode(33, OUTPUT);
  Serial.print("Testing Relay");
  digitalWrite(33, LOW);
  digitalWrite(33, HIGH);
  delay(1000);
  digitalWrite(33, LOW);


  //Relay
  pinMode(Relay24, OUTPUT);
  pinMode(RelayFan, OUTPUT);

  //Lipo
  Wire.begin();
  lipo.enableDebugging();  

  // Set up the MAX17043 LiPo fuel gauge:
  if (lipo.begin() == false)  // Connect to the MAX17043 using the default wire port
  {
    Serial.println(F("MAX17043 not detected. Please check wiring. Freezing."));
    NoBatteryCheck = true;
  } else {
    NoBatteryCheck = false;
  }

  // Quick start restarts the MAX17043 in hopes of getting a more accurate
  // guess for the SOC.
  lipo.quickStart();


  lipo.setThreshold(20); 



  //Setting up Blade
  bladeMotor.write(0);
  delay(10000);
  Serial.println("Blade is armed");
  ///servo.write(0);


  Serial.println("Started Receiver");
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  /*
    for (pos = 200; pos >= 50; pos -= 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);  // tell servo to go to position in variable 'pos'
    Serial.println(pos);
    delay(15);  // waits 15ms for the servo to reach the position
    }*/
  lox.startRangeContinuous(150);
  lox2.startRangeContinuous(150);
  lidarRunning = true;
  // Launch lidar reader task on Core 0
  xTaskCreatePinnedToCore(
    lidarLoop,
    "LIDAR Task",
    4096,
    NULL,
    1,
    &lidarTask,
    0  // Core 0
  );
}

void loop() {

  leftLimit.loop();
  rightLimit.loop();
  backLimit.loop();

  static int lastRange1 = -1;
  static int lastRange2 = -1;

  if (range1 != lastRange1 || range2 != lastRange2) {
    processLidar(range1, range2);

    lastRange1 = range1;
    lastRange2 = range2;

    Serial.print("Lidar 1: ");
    Serial.print(lastRange1);
    Serial.print(" Lidar 2: ");
    Serial.println(lastRange2);
  }

  if (dock == 4) {
    Serial2.print("8");
    Serial2.print("\n");
    Serial.println("hhgghghghgh");
    dock = 0;
  } else {
    //Left Side
    if (leftLimit.isPressed()) {
      Serial.println("LEFT --> PRESSED");
      //LidarDec = 7;
    }
    if (leftLimit.isReleased()) {
      Serial.println("LEFT --> RELEASED");
      //LidarDec = 8;
    }
    //Right Side
    if (rightLimit.isPressed()) {
      Serial.println("RIGHT --> PRESSED");
      // LidarDec = 9;
    }
    if (rightLimit.isReleased()) {
      Serial.println("RIGHT --> RELEASED");
      //LidarDec = 10;
    }
    //Back Side
    if (backLimit.isPressed()) {
    }
    if (backLimit.isReleased()) {
    }


    int leftState = leftLimit.getState();
    int rightState = rightLimit.getState();
    int backState = backLimit.getState();
    if (leftState == HIGH) {
      Serial.println("Left:  Down");
      called = true;
    } else {
      Serial.println("Left:  Open");
      called = false;
    }
    if (rightState == HIGH) {
      Serial.println("Right: Down");
      called = true;
    } else {
      Serial.println("Right: Open");
      called = false;
    }
    if (backState == LOW) {
      Serial.println("Back: Down");
    } else {
      Serial.println("Back: Open");
    }
    if (leftState == HIGH || rightState == HIGH) {
      stateDec = 7;
    } else {
      stateDec = 8;
    }
  }
  Serial.print("StateDec: ");
  Serial.println(stateDec);
  if (stateDec != lastState) {
    Serial2.print(stateDec);
    Serial2.print("\n");
    lastState = stateDec;
  }

  Serial.print("LidarDec: ");
  Serial.println(LidarDec);
  if (LidarDec != 0) {
    if (lastL == LidarDec) {
      ldr++;
    } else {
      ldr = 0;
    }
    if (ldr == 1 && lastSent != LidarDec) {
      Serial.println("SENT");
      Serial2.print(LidarDec);
      Serial2.print("\n");
      lastSent = LidarDec;
      ldr = 0;
    }
    lastL = LidarDec;
  } else {
    ldr = 0;
  }
  while (Serial2.available() > 0) {
    Serial.println("Cuh");
    // Serial.print(char(Serial2.read()));
    incoming = Serial2.readStringUntil('\n');
    Serial.println(incoming);
    int ig = incoming.toInt();
    Serial.println(ig);
    switch (ig) {
      case 999:
        digitalWrite(Relay24, HIGH);
        break;
      case 998:
        digitalWrite(Relay24, LOW);
        break;
      case 889:
        digitalWrite(RelayFan, HIGH);
        break;
      case 888:
        digitalWrite(RelayFan, LOW);
        break;
    }
    if (ig < 255 && called == false) {
      bladeMotor.write(ig);  // Normaly bladeMotor.write(ig);
    } else if (ig > 300 && ig <= 570) {
      servoMove = ig - 300;
    }
  }
  if(called == true){ 
    bladeMotor.write(0); 
  }
  //bladeMotor.write(90);
  //270 - 50  -270 is 0 | 50 is 96%

  if (lastServo != servoMove) {
    digitalWrite(lidarPinX1, LOW);
    digitalWrite(lidarPinX2, LOW);
    lidarRunning = false;
    delay(50);
    servo.write(servoMove);
    Serial.println("Servo Moved");
    lastServo = servoMove;
    delay(1000);
    lidarInit();
  }
  LidarDec = 0;
  //servoMove = 30; // Not supposed to be here normally.
}
void lidarInit() {
  digitalWrite(lidarPinX1, LOW);
  digitalWrite(lidarPinX2, LOW);
  delay(10);
  digitalWrite(lidarPinX1, HIGH);
  digitalWrite(lidarPinX2, HIGH);
  delay(50);
  digitalWrite(lidarPinX2, LOW);
  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin(0x31)) {
    Serial.println(F("Failed to boot VL53L0X"));
  }
  Serial.print("Tests");
  digitalWrite(lidarPinX2, HIGH);

  if (!lox2.begin()) {
    Serial.println(F("Failed to boot VL53L0Xs"));
  }
  lox.startRangeContinuous(150);
  lox2.startRangeContinuous(150);
  lidarRunning = true;
}
void processLidar(int l, int l2) {
  if (l < 30) {
    LidarDec = 1;  // Very Close
    return;
  } else if (l < 50) {
    LidarDec = 2;
    return;
  } else if (l < 60) {
    LidarDec = 3;
    return;
  } else if (l < 100) {
    LidarDec = 4;
    return;
  } else if (l < 135) {
    LidarDec = 5;
  } else {
    LidarDec = 6;
  }

  if (l2 < 30) {
    LidarDec = 11;
  } else if (l2 < 50) {
    LidarDec = 12;
    return;
  } else if (l2 < 60) {
    LidarDec = 13;
    return;
  } else if (l2 < 100) {
    LidarDec = 14;
    return;
  } else if (l2 < 135) {
    LidarDec = 15;
  } else {
    LidarDec = 6;
  }
}


//Gets the percantage - Depreciated 
/*
void getPerc(int a, int b, int c) {
  if (a == 1 && b == 1 && c == 1) {
    Serial.println("Full");
    dock = 0;
    if (bl == 5) {
      if (t == 1) {
        Serial2.print("11");
        Serial2.print("\n");
      } else if (t == 0) {
        t = 100;
        Serial2.print("11");
        Serial2.print("\n");
      } else {
        t--;
      }
    } else {
      t = 0;
    }
  } else if (a == 0 && b == 1 && c == 1) {
    Serial.println("75%");
    dock = 0;
  } else if (a == 0 && b == 0 && c == 1) {
    Serial.println("33.3%");
    dock++;
  } else if (a == 0 && b == 0 && c == 0) {
    Serial.println("0");
    dock++;
  } else {
    Serial.println("Error");
  }
  int d = a * 100 + b * 10 + c;
  if (d == 555) {
    Serial.println("Fully Charged");
  } else if (d == 655) {
    Serial.println("67% Charging");
  } else if (d == 265) {
    Serial.println("37% Charging");
  } else if (d == 226) {
    Serial.println("10% Charging");
  }
}
*/ 
