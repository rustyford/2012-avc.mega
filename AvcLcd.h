#ifndef AvcLcd_h
#define AvcLcd_h

#include "Arduino.h"
#include <Streaming.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include "Avc.h"
#include "AvcEeprom.h"
#include "AvcPath.h"

class AvcLcd {
public:
  enum Mode {NONE, SAMPLING, WAYPOINTS};

  AvcLcd(AvcPath*);
  void printHello();
  void setMode(Mode);
  void display();
  void resetMode();
  void printGps(long,long,float,boolean,int);
  void printStartSampling(byte waypoints);
  void askReset(byte waypoints);
  void askWaypointSlideshow(byte waypoints);
  void askSetMaxSpeed(float maxSpeed, boolean refresh);
  void trackHeading (int GpsHeading, int compassHeading, boolean refresh);
  void askSetOffset(int, int, boolean);
  void showRunLocation(byte runLoc);
  void cameraValues(int, int, int, int, boolean);

  inline Mode getMode () {return mode;}
  static inline float getPotSpeed(int iterations) {
    int maxPI = 0;
//    for (int ii = 0; ii < iterations; ii++) {
      maxPI = max(pulseIn(SPEED_CONTROL_PIN, HIGH, 100000), maxPI);
//    }
    return (maxPI - 1500.0) / 500.0;
  }

private:
  LiquidCrystal lcd;
  Mode mode;
  unsigned long time;
  boolean init;
  boolean backlit;
  Mode previousMode;
  byte waypointIndex;
  AvcPath* path;

  void sampling();
  void waypointSlideshow ();
};
#endif
