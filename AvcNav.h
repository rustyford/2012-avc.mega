#ifndef AvcNav_h
#define AvcNav_h

#include "Arduino.h"
#include <Streaming.h>
#include <TinyGPS.h>
#include "AvcImu.h"
#include "Avc.h"
#include "AvcPid.h"
#include "AvcLcd.h"
//#include "AvcEeprom.h"
#include "Gps.h"
#include <AvcPath.h>
#include "AvcSettings.h"

#if USE_SERVO_LIBRARY
#include <Servo.h>
#endif

#define BUF_SIZE 256
#define RAMP_UP_FACTOR .004
#define RAMP_UP_INIT .4

class AvcNav {
  long latitude;
  long longitude;
  float hdop;
  float distanceTraveled;
  unsigned long fixTime;
  unsigned long previousFixTime;
  float speed;
  boolean waasLock;
  int heading;
  AvcGps *tempWaypoint;
  AvcPath* path;
  AvcSettings* settings;
  int waypointSamplingIndex;
  int numWaypointsSet;
  int nextWaypoint;
  int bestKnownHeading;
  boolean sampling;
  int samples;
  AvcPid pid;
  boolean goodHdop;
  boolean reorienting;
  long sLat, sLon, dLat, dLon;
  float odometerSpeed;
  unsigned long previousOdometerCount;
  unsigned long previousOdometerMillis;
  boolean gpsUpdated;
  byte previousPidOffset;
  byte pidOffset;
  int previousSteering;
#if USE_SERVO_LIBRARY
  Servo steeringServo;
  Servo speedServo;
#endif
  float previousCte;
  float maxSpeed;
  float previousSpeed;
  byte runLocation;
  int steerHeading;
  boolean startBreaking;
  long breakingStartTime;
  boolean killIt;
  boolean rampUpSpeed;
  float previousRampUpSpeed;
  boolean startSpeeding;
  long speedingStartTime;
  int cameraX1;
  int cameraY1;
  int cameraX2;
  int cameraY2;
  int previousCameraX1;
  int previousCameraY1;
  int previousCameraX2;
  int previousCameraY2;
  boolean objectDetected;
  unsigned long timeObjectDetected;
  int accelX, accelY, accelZ;
  int gyroX, gyroY, gyroZ;
  unsigned int temp;
  unsigned int kalmanTime;

  inline float toFloat (long fixed) {return fixed / 1000000.0;}
  inline boolean checkHdop() {return hdop > .0001 && hdop < 5.0;}
  inline float percentOfServo (float percent) {return (MAX_SERVO - SERVO_CENTER) * percent;}

  float crossTrackError ();
  void pickWaypoint();
  int lineCircle(long,long,long,long,long,long);
  int lineCircle();

#if LOG_MAPPER
  void logMapper () {
    long dec;
    int integ;
    printableL (longitude, &integ, &dec);
    // Log car's lat, lon and bearing to next waypoint
    Serial << "$CP," <<
        _FLOAT(toFloat(latitude), 6) <<                  // Car's current latitude
        "," <<
        integ << "." << dec <<                  // Car's current longitude
        "," <<
        heading << "," <<         // Car's facing angle (with declination correction)
        steerHeading <<        // Angle to next waypoint
        endl;
//    Serial1.print(F(","));
//    Serial1.print(steerAngle, DEC);       // error that drives steering                    
//    Serial1.print(F(","));
//    Serial1.print(getOdoInc(), DEC);      // Odo ticks since last check
//    Serial1.print(F(","));
//    Serial1.print(hdop, 2);               // HDOP
//    Serial1.print(F(","));
//    Serial1.print(sats, DEC);             // Sats
//    Serial1.print(F(","));
//    Serial1.write(fix_type);              // Fix Type
//    Serial1.println();
  }
  
  inline void printableL (long l, int *i, long *d) {
    i[0] = int(l / 1000000);
    d[0] = abs(l - *i * 1000000);
  }
#endif

public:
  AvcNav (AvcPath*, AvcSettings*);
  void steer ();
  void update (AvcImu*);
  void sample (AvcLcd*);
  void resetWaypoints();
  void startSampling(AvcLcd*);
  void updateSpeed(float);
  void updateSpeed(AvcImu*);
  void updateCompass(AvcImu*);
  void updateGps(AvcImu*);
  //pass value between 0 and 1
  void setSpeed(float);
  void setMaxSpeed();
  void drive();
  int getLatPotentialOffset ();
  int getLonPotentialOffset ();
//  void setOffset ();
  void nextRunLocation();
  void updateGps (Gps *loc);
  void updateMpu(AvcImu*);
  void setRampUpSpeed (boolean);
  void nuetral();
  void processCamera(AvcImu*);
  void updateWaypoints();

  inline long getLatitude() {return latitude;}
  inline long getLongitude() {return longitude;}
  inline float getHdop() {return hdop;}
  inline float getDistanceTraveled() {return distanceTraveled;}
  inline unsigned long getFixTime() {return fixTime;}
  inline float getSpeed() {return speed;}
  inline boolean hasWaasLock() {return waasLock;}
  inline int getHeading() {return heading;}
  inline boolean isSampling() {return sampling;}
  inline byte getNumWaypoints() {return numWaypointsSet;}
  inline float getOdometerSpeed() {return odometerSpeed;}
  inline float getGpsSpeed() {return speed;}
  inline float getMaxSpeed() {return maxSpeed;}
  inline byte getRunLocation() {return runLocation;}
  inline int getCameraX1() { return cameraX1;}
  inline int getCameraY1() { return cameraY1;}
  inline int getCameraX2() { return cameraX2;}
  inline int getCameraY2() { return cameraY2;}
  inline int getNextWaypoint() {return nextWaypoint;}
  inline int getHeadingToWaypoint () {
    long lat = path->getLatitude(nextWaypoint);
    long lon = path->getLongitude(nextWaypoint);
//    AvcEeprom::readLatLon (nextWaypoint, &lat, &lon);
    return (int) TinyGPS::course_to(toFloat(latitude), 0.0f, toFloat(lat), toFloat(lon - longitude));
  }

#if LOG_NAV
  inline void printHeading(Print& logger) {
    logger << "NAV" << "\t" <<
        "latitude" << "\t" <<
        "longitude" << "\t" <<
        "hdop" << "\t" <<
        "distanceTraveled" << "\t" <<
        "fixTime" << "\t" <<
        "speed" << "\t" <<
        "odometerSpeed" << "\t" <<
        "waasLock" << "\t" <<
        "heading" <<
        endl;    
  }
  inline void print(Print &logger) {
    logger << "NAV" << "\t" <<
        latitude << "\t" <<
        longitude << "\t" <<
        hdop << "\t" <<
        distanceTraveled << "\t" <<
        fixTime << "\t" <<
        speed << "\t" <<
        odometerSpeed << "\t" <<
        waasLock << "\t" <<
        heading <<
        endl;
  }
#endif

#if TRACK_PATH_DISTANCES
  inline void logPathDistances(Print &logger) {
    float distanceFromWaypoint = TinyGPS::distance_between(toFloat(latitude), 0.0f, 
        toFloat(dLat), toFloat(dLon - longitude));
    float distanceBetweenWaypoints = TinyGPS::distance_between(toFloat(sLat), 0.0f, 
        toFloat(dLat), toFloat(dLon - sLon));
    float distanceFromStartWaypoint = TinyGPS::distance_between(toFloat(sLat), 0.0f, 
        toFloat(latitude), toFloat(longitude - sLon));
    logger << nextWaypoint << "\t" <<
        numWaypointsSet << "\t" <<
        distanceFromWaypoint << "\t" <<
        distanceBetweenWaypoints << "\t" <<
        distanceFromStartWaypoint << "\t"<<
    endl;
  }
#endif
  inline void printWaypoints() {
    if (numWaypointsSet > 0) {
      for (int ii = 0; ii < numWaypointsSet; ii++) {
        long lat = path->getLatitude(ii);
        long lon = path->getLongitude(ii);
//        AvcEeprom::readLatLon (ii, &lat, &lon);
        Serial << 
            lat << "\t" <<
            lon << "\t";
      }
      Serial << endl;
    }
  }
  
#if LOG_HEADING
  void logHeadingData(int headingToDest, int steering, float cte, int steeringAdj) {
    Serial <<
    _FLOAT(dLat/1000000.0, 6) << "\t" <<
    _FLOAT(dLon/1000000.0, 6) << "\t" <<
    _FLOAT(latitude/1000000.0, 6) << "\t" <<
    _FLOAT(longitude/1000000.0, 6) << "\t" <<
    headingToDest << "\t" <<
    heading << "\t" <<
    cte << "\t" <<
    steering << "\t" <<
    steeringAdj << "\t" <<
    hdop << "\t" <<
    nextWaypoint <<
    endl;
  }
#endif
#if LOG_EKF
  void logEkfData () {
    Serial2 <<
    _FLOAT(latitude/1000000.0, 6) << "\t" <<
    _FLOAT(longitude/1000000.0, 6) << "\t" <<
    _FLOAT(hdop, 2) << "\t" <<
    odometerSpeed << "\t" <<
    heading << "\t" <<
    gyroZ << 
    _FLOAT((millis() - kalmanTime) / 1000.0, 3) <<
    endl;
    kalmanTime = millis();
  }
#endif
};
#endif
