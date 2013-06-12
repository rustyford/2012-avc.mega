#include "AvcSettings.h"

void AvcSettings::setMaximumSpeed(float speed) {
  maximumSpeed = speed * 1000.0;
}

void AvcSettings::setRunLocation(byte loc) {
  runLocation = loc;
}

void AvcSettings::log(Stream* stream) {
  stream->print("max speed: ");
  stream->println(maximumSpeed);
  stream->print("run Location: ");
  stream->println(runLocation);
}

void AvcSettings::writeToEeprom() {
  byte *settings = (byte*) this;
  for (int ii = 0; ii < sizeof(*this); ii++) {
    EEPROM.write(ii, settings[ii]);
  }
}

