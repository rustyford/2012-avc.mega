#ifndef Eeprom_h
#define Eeprom_h

#include "Arduino.h"
#include <EEPROM.h>
#include "Avc.h"

#define WAY_COUNT  0
#define MAX_SPEED 1 //int
#define RUN_LOCATION 3
//#define LOG_TYPE   4
//#define LINE_STEER 5
//#define WAY_BASE   16
#define PERRY_HIGH (LOC_PERRY_HIGH + 1)  * 50
// run locations
// byte number of waypoints 0
// int lat offset 1 - 2
// int lon offset 3 - 4
// unsed 5 - 7
// lat lon 8 +

class AvcEeprom {
  static void writeLatLon (int idx, long *lat, long *lon, int loc) {
    Serial << idx << " " << *lat << " " << *lon << " " << loc << endl;
    idx = idx * 2;
    writeLong(idx, lat, loc * 50 + 8);
    writeLong(idx + 1, lon, loc * 50  + 8);
  }
  
//  static void readLatLon (int idx, long *lat, long *lon, int loc) {
//    idx = idx * 2;
//    readLong(idx, lat, loc * 50 + 8);
//    readLong(idx + 1, lon, loc * 50 + 8);
//  }

public:
  static void init () {
#if RESET_RUN_LOCATIONS
//    int z = 0;
//    EEPROM.write(PERRY_HIGH, byte(4));
//    writeInt(PERRY_HIGH + 1, &z);
//    writeInt(PERRY_HIGH + 2, &z);
//    long lat = 33261509, lon = -111751025;
//    writeLatLon(0, &lat, &lon, PERRY_HIGH);
//    lon = -111750192;
//    writeLatLon(1, &lat, &lon, PERRY_HIGH);
//    lat = 33261814;
//    writeLatLon(2, &lat, &lon, PERRY_HIGH);
//    lon = -111751025;
//    writeLatLon(3, &lat, &lon, PERRY_HIGH);
    
    setWayCount(9);
//    int latOffset = 0;
//    int lonOffset = 0;
//    setRunOffset(latOffset,lonOffset);
//    long lat = 40065198, lon = -105210021;
//    writeLatLon(0, &lat, &lon);
//    lat = 40065158;
//    lon = -105209770;
//    writeLatLon(1, &lat, &lon);
//    lat = 40064823;
//    writeLatLon(2, &lat, &lon);
//    lat = 40064483;
//    writeLatLon(3, &lat, &lon);
//    lon = -105210067;
//    writeLatLon(4, &lat, &lon);
//    lon = -105210449;
//    writeLatLon(5, &lat, &lon);
//    lat = 40064521;
//    writeLatLon(6, &lat, &lon);
//    lat = 40065166;
//    writeLatLon(7, &lat, &lon);
//    lat = 40065158;
//    lon = -105210075;
//    writeLatLon(8, &lat, &lon);

//40.065158,-105.210021
//40.065158,-105.209770
//40.064823,-105.209808
//40.064483,-105.209831
//40.064483,-105.210067
//40.064514,-105.210449
//40.064521,-105.210449
//40.065166,-105.210449
//40.065158,-105.210075


//40.065158,-105.210021
//40.065158,-105.209770
//40.064823,-105.209770
//40.064483,-105.209770
//40.064483,-105.210067
//40.064514,-105.210449
//40.064521,-105.210449
//40.065166,-105.210449
//40.065158,-105.210075



//40.065288,-105.210067
    long lat = 40065288, lon = -105210067;
    writeLatLon(0, &lat, &lon);
//40.065219,-105.209770
//40.065219,-105.209790
    lat = 40065219;
    lon = -10520790;
    writeLatLon(1, &lat, &lon);
//40.064884,-105.209808
    lat = 40064884;
    lon = -105209808;
    writeLatLon(2, &lat, &lon);
//40.064529,-105.209831
    lat = 40064529;
    lon = -105209831;
    writeLatLon(3, &lat, &lon);
//40.064529,-105.210144
    lat = 40064529;
    lon = -105210144;
    writeLatLon(4, &lat, &lon);
//40.064529,-105.210449
    lat = 40064529;
    lon = -105210449;
    writeLatLon(5, &lat, &lon);
//40.064697,-105.210433
    lat = 40064697;
    lon = -105210433;
    writeLatLon(6, &lat, &lon);
//40.065230,-105.210433
    lat = 40065230;
    lon = -105210433;
    writeLatLon(7, &lat, &lon);
//40.065254,-105.209770
    lat = 40065254;
    lon = -105209770;
    writeLatLon(8, &lat, &lon);
#endif
  }
  static void writeLong (int idx, long *val, int offset) {
    byte *bytes = (byte *) val;
    for (int ii = 0; ii < sizeof(long); ii++) {
      EEPROM.write((idx * sizeof(long)) + ii + offset, bytes[ii]);
    }
  }
  
  static void readLong (int idx, long *val, int offset) {
    byte *bytes = (byte *) val;
    for (int ii = 0; ii < sizeof(long); ii++) {
      bytes[ii] = EEPROM.read((idx * sizeof(long)) + ii + offset);
    }
  }
  
  static void writeInt (int idx, int *val) {
    byte *bytes = (byte *) val;
    EEPROM.write(idx, bytes[0]);
    EEPROM.write(idx + 1, bytes[1]);
  }

  static void readInt (int idx, int *val) {
    byte *bytes = (byte *) val;
    bytes[0] = EEPROM.read(idx);
    bytes[1] = EEPROM.read(idx + 1);
  }

//  static byte getWayCount () {
//    return EEPROM.read(getRunLocation() * 50 + WAY_COUNT);
//  }
  
//  static void setWayCount (byte cnt) {
//    EEPROM.write(getRunLocation() * 50 + WAY_COUNT, cnt);
//  }
  
//  static float getMaxSpeed () {
//    int val;
//    readInt(MAX_SPEED, &val);
//    float r = val  * .001;
//    return r;
//  }
  
//  static void setMaxSpeed (float maxSpeed) {
//    int val = maxSpeed * 1000.0;
//    writeInt(MAX_SPEED, &val);
//  }
  
//  static byte getRunLocation () {
////    return EEPROM.read(RUN_LOCATION);
//    return 1;
//  }
  
//  static void setRunLocation (byte loc) {
//    // locations must start at 1
//    EEPROM.write(RUN_LOCATION, loc + 1);
//  }
  
//  static byte getLogType () {
//    return EEPROM.read(LOG_TYPE);
//  }
  
//  static void setLogType (byte type) {
//    EEPROM.write(LOG_TYPE, type);
//  }
//  
//  static boolean getLineSteer () {
//    return EEPROM.read(LINE_STEER) == '1';
//  }
//  
//  static void setLineSteer (boolean enable) {
//    EEPROM.write(LINE_STEER, enable ? '1' : '0');
//  }
  
//  static void writeLatLon (int idx, long *lat, long *lon) {
//    writeLatLon(idx, lat, lon, getRunLocation());
//  }
  
//  static void readLatLon (int idx, long *lat, long *lon) {
//    readLatLon(idx, lat, lon, getRunLocation());
//  }
  
//  static void readOffsetLatLon (int idx, long *lat, long *lon) {
//    readLatLon(idx, lat, lon, getRunLocation());
//    int latOffset = 0;
//    int lonOffset = 0;
//    getRunOffset(&latOffset, &lonOffset);
//    *lat += latOffset;
//    *lon += lonOffset;
//  }
  
//  static void setRunOffset (int lat, int lon) {
//    writeInt(getRunLocation() * 50 + 1, &lat);
//    writeInt(getRunLocation() * 50 + 3, &lon);
//  }
  
//  static void getRunOffset (int *lat, int *lon) {
//    readInt(getRunLocation() * 50 + 1, lat);
//    readInt(getRunLocation() * 50 + 3, lon);
//  }
  
  static void logEeprom () {
    for (int ii = 0; ii < 500; ii++) {
      Serial << ii << " " << EEPROM.read(ii) << endl;
    }
//    int olat = 0;
//    int olon = 0;
////    AvcEeprom::getRunOffset(&olat, &olon);
//    Serial << olat << endl;
//    Serial << olon << endl;
//    long lat, lon;
////    readLatLon(0, &lat, &lon);
//    Serial << lat << endl;
//    Serial << lon << endl;
////    readOffsetLatLon(0, &lat, &lon);
//    Serial << lat << endl;
//    Serial << lon << endl;
  }
};
#endif
