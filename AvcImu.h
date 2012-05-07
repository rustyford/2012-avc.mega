#ifndef AvcImu_h
#define AvcImu_h

#include "Arduino.h"
#include <Streaming.h>

#define BUF_SIZE 256
#define NUM_ELEMENTS 8

class AvcImu {
  long latitude;
  long longitude;
  float hdop;
  float distanceTraveled;
  unsigned long fixTime;
  float speed;
  boolean waasLock;
  int heading;
  char buf[BUF_SIZE];
  byte checksumIdx;
  byte charCount;
  byte elementCount;
  boolean invalid;
  boolean objectComplete;
public:
  AvcImu ();
  void parse (char c);
  
  inline boolean isValid() {return invalid == false;}
  inline boolean isComplete() {return objectComplete;}
  inline long getLatitude() {return latitude;}
  inline long getLongitude() {return longitude;}
  inline float getHdop() {return hdop;}
  inline float getDistanceTraveled() {return distanceTraveled;}
  inline unsigned long getFixTime() {return fixTime;}
  inline float getSpeed() {return speed;}
  inline boolean hasWaasLock() {return waasLock;}
  inline int getHeading() {return heading;}
  
  inline void print() {
    Serial <<
        latitude << "\t" <<
        longitude << "\t" <<
        hdop << "\t" <<
        distanceTraveled << "\t" <<
        fixTime << "\t" <<
        speed << "\t" <<
        waasLock << "\t" <<
        heading <<
        endl;
  }
};
#endif