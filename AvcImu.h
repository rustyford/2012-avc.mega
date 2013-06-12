#ifndef AvcImu_h
#define AvcImu_h

#include "Arduino.h"
#include <Streaming.h>
#include "AvcGps.h"
#include <TinyGPS.h>
#include "Avc.h"

#define BUF_SIZE 256

class AvcImu {
public:
  enum Mode {IMU, GPS, COMPASS, CAMERA, MPU, GCM, CM};
  // GCM = GPS,COMPASS,MPU | CM = COMPASS,MPU

private:
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
  boolean invalid;
  boolean objectComplete;
  Mode mode;
  byte xOffset;
  byte yOffset;
  byte zOffset;
  float odometerSpeed;
  int cameraX1;
  int cameraY1;
  int cameraX2;
  int cameraY2;
  int gyroX;
  int gyroY;
  int gyroZ;
  int accelX;
  int accelY;
  int accelZ;
  unsigned int temp;

public:
  AvcImu ();
  void parse (char c);
  void reset();
  
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
  inline Mode getMode() {return mode;}
  inline float getOdometerSpeed() {return odometerSpeed;}
  inline int getCameraX1() {return cameraX1;}
  inline int getCameraY1() {return cameraY1;}
  inline int getCameraX2() {return cameraX2;}
  inline int getCameraY2() {return cameraY2;}
  inline int getAccelX() {return accelX;}
  inline int getAccelY() {return accelY;}
  inline int getAccelZ() {return accelZ;}
  inline int getGyroX() {return gyroX;}
  inline int getGyroY() {return gyroY;}
  inline int getGyroZ() {return gyroZ;}
  inline int getTemp() {return temp;}
#if LOG_IMU
  inline void print() {
    Serial << "IMU" << "\t" <<
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
#endif
};
#endif
