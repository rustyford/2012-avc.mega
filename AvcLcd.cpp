#include "AvcLcd.h"

AvcLcd::AvcLcd (AvcPath* avcpath): lcd(0) {
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  time = 0;
  init = true;
  backlit = false;
  mode = NONE;
  previousMode = NONE;
  waypointIndex = 0;
  path = avcpath;
}

void AvcLcd::setMode (Mode m) {
  previousMode = mode;
  mode = m;
  init = true;
  if (mode == NONE) {
    lcd.clear();
  }
}

void AvcLcd::display () {
  switch(mode) {
    case SAMPLING:
      sampling();
      break;
    case WAYPOINTS:
      waypointSlideshow();
      break;
  }
}

void AvcLcd::sampling () {
  if (init) {
    lcd.clear();
    lcd.print("SAMPLING");
    init = false;
    time = millis();
    lcd.setBacklight(HIGH);
    backlit = true;
  }
  if ((millis() - time) > 200) {
    if (backlit) {
      lcd.setBacklight(LOW);
      backlit = false;
    } else {
      lcd.setBacklight(HIGH);
      backlit = true;
    }
    time = millis();
  } 
}

void AvcLcd::resetMode () {
  mode = previousMode;
  display();
}

void AvcLcd::printGps (long lat, long lon, float hdop, boolean refresh, int nextWaypoint) {
  if (refresh) {
    lcd.clear();
  }
  lcd.home();
  lcd << _FLOAT(lat/1000000.0, 6);
  lcd.setCursor(0, 1);
  lcd << _FLOAT(lon/1000000.0, 6);
  lcd.setCursor(12, 0);
  lcd << _FLOAT(hdop, 2);
  lcd.setCursor(12, 1);
  lcd << nextWaypoint << "  ";
  lcd.setBacklight(HIGH);
}

void AvcLcd::printStartSampling(byte waypoints) {
  lcd.clear();
  lcd.home();
  lcd << "START SAMPLING?";
  lcd.setCursor(0, 1);
  lcd << "WAYPOINTS: " << waypoints;
  lcd.setBacklight(HIGH);
}

void AvcLcd::askReset(byte waypoints) {
  lcd.clear();
  lcd.home();
  lcd << "RESET WAYPOINTS?";
  lcd.setCursor(0, 1);
  lcd << "WAYPOINTS: " << waypoints;
  lcd.setBacklight(HIGH);
}

void AvcLcd::askWaypointSlideshow(byte waypoints) {
  lcd.clear();
  lcd << "SHOW WAYPOINTS?";
  lcd.setCursor(0, 1);
  lcd << "WAYPOINTS: " << waypoints;
  lcd.setBacklight(HIGH);
}

void AvcLcd::waypointSlideshow () {
  if (init) {
    lcd.clear();
    init = false;
    time = millis();
    waypointIndex = 0;
    lcd.setBacklight(HIGH);
  }
  if ((millis() - time) > 1000) {
    byte count = path->getNumberOfWaypointsSet();
    long lat = path->getLatitude(waypointIndex);
    long lon = path->getLongitude(waypointIndex);
    lcd.home();
    lcd << _FLOAT(lat/1000000.0, 6);
    lcd.setCursor(0, 1);
    lcd << _FLOAT(lon/1000000.0, 6);
    lcd.setCursor(12, 0);
    lcd << waypointIndex;
    time = millis();
    waypointIndex = (waypointIndex + 1) % count;
  } 
}

void AvcLcd::askSetMaxSpeed(float maxSpeed, boolean refresh) {
  if (refresh) {
    lcd.clear();
    lcd.home();
    lcd << "SET MAX SPEED?";
  }
  lcd.setCursor(0, 1);
  float n = getPotSpeed(1);
  lcd << "C " << _FLOAT(maxSpeed, 3) << " ";
  if (n > 0)
    lcd << "N " << _FLOAT(n, 3)  << "  ";
  lcd.setBacklight(HIGH);
}

void AvcLcd::trackHeading (int GpsHeading, int compassHeading, boolean refresh) {
  if (refresh) {
    lcd.clear();
  }
  lcd.home();
  lcd << "GPS: " << GpsHeading << "  ";
  lcd.setCursor(0, 1);
  lcd << "Compass: " << compassHeading << "  ";
  lcd.setBacklight(HIGH);
}

void AvcLcd::askSetOffset(int latOffset, int lonOffset, boolean refresh) {
  if (refresh) {
    lcd.clear();
    lcd.setBacklight(HIGH);
  }
  lcd.home();
  lcd << "SET OFFSET?";
  lcd.setCursor(0, 1);
  lcd << "T: " << latOffset << " N: " << lonOffset << "   ";
}

void AvcLcd::showRunLocation(byte runLoc) {
  lcd.clear();
  lcd.home();
  lcd << "LOCATION";
  lcd.setCursor(0, 1);
  lcd << runLocNames[runLoc - 1];
}

void AvcLcd::cameraValues(int x1, int y1, int x2, int y2, boolean refresh) {
  if (refresh) {
    lcd.clear();
  }
  lcd.home();
  lcd << "X " << x1 << " ";
  lcd << "Y " << y1 << " ";
  lcd.setCursor(0, 1);
  lcd << "X " << x2 << " ";
  lcd << "Y " << y2 << " ";
  lcd.setBacklight(HIGH);
}

