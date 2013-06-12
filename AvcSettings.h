#ifndef AvcSettings_h
#define AvcSettings_h

#include "Arduino.h"
#include <EEPROM.h>
#include <Streaming.h>
#include "AvcEeprom.h"

class AvcSettings {
  byte spacer;
  int maximumSpeed;
  byte runLocation;
public:
  inline float getMaximumSpeed() {return float(maximumSpeed) * .001;}
  void setMaximumSpeed(float);
  void setRunLocation(byte);
  void log(Stream*);
  void writeToEeprom();

  static AvcSettings* getSettings () {
    byte size = sizeof(AvcSettings);
    byte *hold = new byte[size];
    for (int ii = 0; ii < size; ii++) {
      hold[ii] = EEPROM.read(ii);
    }
    return (AvcSettings*) hold;
  }  
};
#endif
