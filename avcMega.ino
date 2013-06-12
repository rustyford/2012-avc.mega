#include "Avc.h"
#include <Wire.h>
#include <TinyGPS.h>
#include <Streaming.h>
#include "AvcPid.h"
#include "AvcGps.h"
#include "AvcImu.h"
#include <LiquidCrystal.h>
#include "AvcLcd.h"
#include "AvcNav.h"
#include <EEPROM.h>
#include "AvcMenu.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#include "Gps.h"
#include "AvcSettings.h"
#include <AvcPath.h>
#include <PString.h>
#include "PathSetup.h"

AvcNav *nav;
AvcImu *imu;
AvcLcd *lcd;
AvcMenu *menu;
Gps location;
AvcSettings *settings;
AvcPath *path;
PathSetup* pathSetup;

volatile unsigned long rotations = 0;
volatile unsigned long previousOdometerMicros = 0;
volatile unsigned long odometerMicrosDelta = 0;
boolean refreshLcd = false;
unsigned long previousTime = 0;
unsigned long fiftyHertzTime = 0;
boolean andWereOff = false;

#if LOG_TO_CARD
  Print* logger = &Serial2;
#else
  Print* logger = &Serial;
#endif

void setup()
{
  // usb
  Serial.begin(115200);
  // input from imu
  Serial1.begin(57600);
  // logger output
  Serial2.begin(57600);
  // GPS input (deprecated)
  Serial3.begin(9600);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(MENU_SELECT_PIN, INPUT);
  pinMode(MENU_SCROLL_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(PUSH_BUTTON_START_PIN, INPUT);
  //enable internal pullup resistors
  digitalWrite(MENU_SELECT_PIN, HIGH);
  digitalWrite(MENU_SCROLL_PIN, HIGH);
  digitalWrite(HALL_SENSOR_PIN, HIGH);
  digitalWrite(PUSH_BUTTON_START_PIN, HIGH);
  AvcEeprom::init();
  Gps::init(&Serial3);
  path = AvcPath::getPath(50);
  settings = AvcSettings::getSettings();
  nav = new AvcNav(path, settings);
  imu = new AvcImu();
  lcd = new AvcLcd(path);
  menu = new AvcMenu(lcd, nav);
  pathSetup = new PathSetup();
  attachInterrupt(0, countRotations, CHANGE);
#if LOG_EEPROM
  AvcEeprom::logEeprom();
#endif
#if LOG_NAV
  nav->printHeading(*logger);
#endif
  
}

void loop() {
  while (Serial1.available()) {
    byte c = Serial1.read();
//    Serial.write(c);
    imu->parse(c);
    if (imu->isComplete()) {
      if (imu->isValid()) {
        switch (imu->getMode()) {
          case AvcImu::COMPASS:
            nav->updateCompass(imu);
            break;
          case AvcImu::GPS:
            nav->updateGps(imu);
            if (nav->isSampling()) {
              nav->sample(lcd);
              refreshLcd = true;
            }
            break;
          case AvcImu::IMU:
            nav->update(imu);
            break;
          case AvcImu::CAMERA:
            nav->processCamera(imu);
            break;
          case AvcImu::MPU:
            nav->updateMpu(imu);
            break;
          case AvcImu::GCM:
            nav->updateMpu(imu);
            nav->updateCompass(imu);
            nav->updateGps(imu);
            nav->updateSpeed(imu);
            if (nav->isSampling()) {
              nav->sample(lcd);
              refreshLcd = true;
            }
#if TRACK_PATH_DISTANCES
            nav->logPathDistances(*logger);
#endif
            break;
          case AvcImu::CM:
            nav->updateMpu(imu);
            nav->updateCompass(imu);
            nav->updateSpeed(imu);
            break;
        }
        if (!nav->isSampling()) {
          if (odometerMicrosDelta < 0) {
            odometerMicrosDelta = 4294967295 + odometerMicrosDelta;
          }
          nav->steer();
#if LOG_NAV
          nav->print(*logger);
#endif
        }
      }
      imu->reset();
      break;
    }
  }
  location.checkGps(&Serial3);
  if (location.isValid()) {
    nav->updateGps(&location);
    if (nav->isSampling()) {
      nav->sample(lcd);
      refreshLcd = true;
    } else {
      if (odometerMicrosDelta < 0) {
        odometerMicrosDelta = 4294967295 + odometerMicrosDelta;
      }
      nav->steer();
#if LOG_NAV
      nav->print(*logger);
#endif
    }
    imu->reset();
  }
  if (millis() - fiftyHertzTime > 20) {
    fiftyHertzTime = millis();
    if (!nav->isSampling()) {
//      nav->updateSpeed(odometerMicrosDelta * .000001);
      if (PUSH_BUTTON_START && !andWereOff) {
        if (digitalRead(PUSH_BUTTON_START_PIN) == LOW) {
          andWereOff = true;
          nav->setRampUpSpeed(true);
        } else {
          nav->nuetral();
        }
      } else {
        nav->drive();
      }
    }
  }
  unsigned long mls = millis();
  if (nav->getOdometerSpeed() == 0) {
    if ((mls - previousTime) > (1000 / LOOP_SPEED)) {
      previousTime = mls;
      lcd->display();
      if (!nav->isSampling()) {
        menu->checkButtons(refreshLcd);
        refreshLcd = false;
      }
    }
    pathSetup->updateCommandIO (&Serial, path);
  }
}

void countRotations () {
  rotations++;
  odometerMicrosDelta = micros() - previousOdometerMicros;
  previousOdometerMicros = micros();
}
