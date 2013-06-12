#include "AvcNav.h"

AvcNav::AvcNav (AvcPath* avcpath, AvcSettings* avcsettings): pid() {
  path = avcpath;
  settings = avcsettings;
  latitude = 0;
  longitude = 0;
  hdop = 0;
  distanceTraveled = 0;
  fixTime = 0;
  speed = 0;
  waasLock = false;
  heading = 0;
  waypointSamplingIndex = -1;
  bestKnownHeading = 0;
  sampling = false;
  samples = 0;
  goodHdop = true;
  reorienting = false;
  odometerSpeed = 0;
  previousOdometerCount = 0;
  previousOdometerMillis = 0;
  gpsUpdated = false;
  previousPidOffset = 0;
  previousSteering = SERVO_CENTER;
  runLocation = 1;//AvcEeprom::getRunLocation();
  if (runLocation == 255) runLocation = 1;
  nextWaypoint = 0;
  updateWaypoints();
  pidOffset = 0;
#if USE_SERVO_LIBRARY
  steeringServo.attach(SERVO_PIN);
  speedServo.attach(MOTOR_PIN);
#endif
  previousCte = 0;
  maxSpeed = settings->getMaximumSpeed();//AvcEeprom::getMaxSpeed();
  previousSpeed = 0;
  steerHeading = 0;
  startBreaking = false;
  killIt = false;
  rampUpSpeed = false;
  previousRampUpSpeed = RAMP_UP_INIT;
  cameraX1 = 0;
  cameraY1 = 0;
  cameraX2 = 0;
  cameraY2 = 0;
  previousCameraX1 = 0;
  previousCameraY1 = 0;
  previousCameraX2 = 0;
  previousCameraY2 = 0;
  objectDetected = false;
  timeObjectDetected = 0;
  kalmanTime = millis();
}

void AvcNav::updateWaypoints() {
  numWaypointsSet = max(path->getNumberOfWaypointsSet(), 0);
//#if SKIP_FIRST_WAYPOINT
//  nextWaypoint = 1 % numWaypointsSet;
//#else
//  nextWaypoint = 0;
//#endif
//  if (numWaypointsSet > 0 + SKIP_FIRST_WAYPOINT) {
//    AvcEeprom::readOffsetLatLon (nextWaypoint, &dLat, &dLon);
//    if (numWaypointsSet > 1 + SKIP_FIRST_WAYPOINT) {
//      AvcEeprom::readOffsetLatLon ((nextWaypoint - 1 + numWaypointsSet - RACE_MODE) % numWaypointsSet, &sLat, &sLon);
//    }
//  }
  nextWaypoint = 1;
  dLat = path->getLatitude(nextWaypoint);
  dLon = path->getLongitude(nextWaypoint);
  sLat = path->getLatitude(0);
  sLon = path->getLongitude(0);
  killIt = false;
//  AvcEeprom::readOffsetLatLon (nextWaypoint, &dLat, &dLon);
//  AvcEeprom::readOffsetLatLon (numWaypointsSet - 2, &sLat, &sLon);
}

void AvcNav::pickWaypoint() {
  if (numWaypointsSet >= 2) {
    float distanceFromWaypoint = TinyGPS::distance_between(toFloat(latitude), 0.0f, 
        toFloat(dLat), toFloat(dLon - longitude));
    float distanceBetweenWaypoints = TinyGPS::distance_between(toFloat(sLat), 0.0f, 
        toFloat(dLat), toFloat(dLon - sLon));
    float distanceFromStartWaypoint = TinyGPS::distance_between(toFloat(sLat), 0.0f, 
        toFloat(latitude), toFloat(longitude - sLon));
#if BREAK_BEFORE_TURN
    if (distanceFromWaypoint < odometerSpeed) {
      startBreaking = true;
      breakingStartTime = millis();
    }
//    if (distanceFromWaypoint < max(WAYPOINT_RADIUS, odometerSpeed / 2.0) || distanceFromStartWaypoint > distanceBetweenWaypoints) {
#else
//    if (distanceFromWaypoint < WAYPOINT_RADIUS || distanceFromStartWaypoint > distanceBetweenWaypoints) {
#endif
    if (distanceFromWaypoint < WAYPOINT_RADIUS || distanceFromStartWaypoint > distanceBetweenWaypoints) {
#if SPEED_THROUGH_TURN
      startSpeeding = true;
      speedingStartTime = millis();      
#endif
      if (RACE_MODE) {
        nextWaypoint++;
        if (nextWaypoint == numWaypointsSet) {
          killIt = true;
          nextWaypoint--;
        }
//        nextWaypoint = (nextWaypoint + 1) % numWaypointsSet;
      } else {
        nextWaypoint = (nextWaypoint + 1) % numWaypointsSet;
#if SKIP_FIRST_WAYPOINT && nextWaypoint == 0
        nextWaypoint = 1;
#endif
      }
      startBreaking = false;
      sLat = dLat;
      sLon = dLon;
      dLat = path->getLatitude(nextWaypoint);
      dLon = path->getLongitude(nextWaypoint);
      //AvcEeprom::readOffsetLatLon (nextWaypoint, &dLat, &dLon);
      reorienting = true;
    }
  }
}

void AvcNav::steer () {
  pickWaypoint();
  if (numWaypointsSet > 0) {
    int h = getHeadingToWaypoint();
    if (h >= 0 && goodHdop) {
      bestKnownHeading = h;
      steerHeading = bestKnownHeading;
#if USE_LINE_INTERSECT
//      if (!reorienting) {
        steerHeading = lineCircle(latitude, longitude, sLat, sLon, dLat, dLon);
//      }
#endif
    } else {
      steerHeading = bestKnownHeading;
    }
  } else {
    bestKnownHeading = 0;
  }
//  float cte = crossTrackError();
  // there might be a problem with fixtime and previous fix time if this code runs a long time. or maybe even a short time.
//  if (gpsUpdated) {
//    pidOffset = pid.compute(cte, (fixTime - previousFixTime)/1000.0, odometerSpeed);
//    previousPidOffset = pidOffset;
//  }
  int steering = SERVO_CENTER;
  int adj = 0;
  if (!goodHdop || reorienting || COMPASS_ONLY || odometerSpeed < 1 || DISTANCE_FROM_LINE_CORRECTION) {
    pidOffset = 0;
    int headingOffset = (heading - steerHeading + 360) % 360;
    if (abs(heading - bestKnownHeading) < 30) {
      reorienting = false;
    }
#if AGRESSIVE_STEERING
    if(headingOffset >= 180) {
      headingOffset = constrain(360 - (360 - headingOffset) * 2, 180, 360);
      
      steering = map(headingOffset, 180, 360, MAX_SERVO_75_PERCENT, SERVO_CENTER);
    } else {
      headingOffset = constrain(headingOffset * 2, 0, 180);
      steering = map(headingOffset, 180, 0, MIN_SERVO_75_PERCENT, SERVO_CENTER);
    }
#else
    if(headingOffset >= 180) {
      steering = map(headingOffset, 180, 360, MAX_SERVO, SERVO_CENTER);
    } else {
      steering = map(headingOffset, 180, 0, MIN_SERVO, SERVO_CENTER);
    }
#endif
    byte maxSteeringDiff = percentOfServo(.75);//(.05);
//#if DISTANCE_FROM_LINE_CORRECTION
////  float cte = crossTrackError();
//    // if approaching the line then allow for faster steering
////    if (abs(cte) < abs(previousCte)) {
//      maxSteeringDiff = percentOfServo(.3);
////    } 
//    if (cte > 0) {
//      adj = int(min(percentOfServo(.2), int(cte * percentOfServo(.05))));
//    } else {
//      adj = int(max(-percentOfServo(.2), int(cte * percentOfServo(.05))));
//    }
//    previousCte = cte;
////  Serial << bestKnownHeading << "\t";
////  Serial << heading << "\t";
////  Serial << cte << "\t";
////  Serial << adj << "\t";
////  Serial << steering - SERVO_CENTER << "\t";
//    steering = constrain(steering + adj, MIN_SERVO, MAX_SERVO);
////  Serial << steering - SERVO_CENTER << endl;
//#endif
//    if (steering - previousSteering > maxSteeringDiff && steering > SERVO_CENTER) {
//      steering = previousSteering + maxSteeringDiff;
//    } else if (steering - previousSteering < -maxSteeringDiff && steering < SERVO_CENTER) {
//      steering = previousSteering - maxSteeringDiff;
//    }
    previousSteering = steering;
//#if !COMPASS_ONLY
//  } else { // use pid
//    if (!gpsUpdated) {
//      pidOffset = previousPidOffset;
//    }
//    if(pidOffset <= 0) {
//      steering = map(pidOffset, -100, 0, MAX_SERVO, SERVO_CENTER);
//    } else {
//      steering = map(pidOffset, 100, 0, MIN_SERVO, SERVO_CENTER);
//    }
//    const byte maxSteeringDiff = 2;
//    if (steering - previousSteering > maxSteeringDiff) {
//      steering = previousSteering + maxSteeringDiff;
//    } else if (steering - previousSteering < -maxSteeringDiff) {
//      steering = previousSteering - maxSteeringDiff;
//    }
//    previousSteering = steering;
//    goodHdop = checkHdop();
//#endif
  }
  goodHdop = checkHdop();
#if LOG_HEADING
  logHeadingData(bestKnownHeading, steering - SERVO_CENTER, /*cte*/0.0, adj);
#endif
#if LOG_EKF
  logEkfData();
#endif

#if GO_STRAIGHT
  steeringServo.writeMicroseconds(SERVO_CENTER);
#else
  steeringServo.writeMicroseconds(steering);
#endif
}

void AvcNav::update (AvcImu *imu) {
  gpsUpdated = false;
  if (imu->getHdop() > .0001 && imu->getFixTime() != fixTime) {
    updateGps(imu);
  }
  updateCompass(imu);
}

void AvcNav::updateCompass (AvcImu *imu) {
  gpsUpdated = false;
  heading = imu->getHeading();
}

void AvcNav::updateMpu (AvcImu *imu) {
  accelX = imu->getAccelX();
  accelY = imu->getAccelY();
  accelZ = imu->getAccelZ();
  gyroX = imu->getGyroX();
  gyroY = imu->getGyroY();
  gyroZ = imu->getGyroZ();
  temp = imu->getTemp();
}

void AvcNav::updateGps (AvcImu *imu) {
#if LOG_MAPPER
  logMapper();
#endif
  latitude = imu->getLatitude();
  longitude = imu->getLongitude();
  hdop = imu->getHdop();
  distanceTraveled = imu->getDistanceTraveled();
  previousFixTime = fixTime;
  fixTime = imu->getFixTime();
  speed = imu->getSpeed();
  waasLock = imu->hasWaasLock();
  gpsUpdated = true;
}

void AvcNav::updateGps (Gps *loc) {
#if LOG_MAPPER
  logMapper();
#endif
  latitude = loc->getLatitude();
  longitude = loc->getLongitude();
  hdop = loc->getHdop();
  distanceTraveled = loc->getDistanceTraveled();
  previousFixTime = fixTime;
  fixTime = loc->getFixTime();
  speed = loc->getSpeed();
  waasLock = loc->hasWaasLock();
  gpsUpdated = true;
}

void AvcNav::sample (AvcLcd *lcd) {
  if (fixTime == previousFixTime || hdop < .0001) {
    return;
  }
  if (samples >= MAX_SAMPLES) {
    long lat = tempWaypoint->getLatitude();
    long lon = tempWaypoint->getLongitude();
    path->addWaypoint(lat, lon);
    path->writeToEeprom(50);
//    AvcEeprom::writeLatLon (numWaypointsSet - 1, &lat, &lon);
    if (numWaypointsSet == 1) {
      dLat = lat;
      dLon = lon;
    } else if (numWaypointsSet > 2) {
      sLat = lat;
      sLon = lon;
    } 
    delete tempWaypoint;
    tempWaypoint = NULL;
    sampling = false;
    lcd->resetMode();
    return;
  }
  samples++;
  tempWaypoint->sample(getLatitude(), getLongitude(), getHdop());
}

void AvcNav::resetWaypoints() {
  path->resetWaypoints();
//  AvcEeprom::setWayCount(0);
//  AvcEeprom::setRunOffset (0, 0);
//  delete tempWaypoint;
  waypointSamplingIndex = -1;
  numWaypointsSet = 0;
  nextWaypoint = 0;
  sampling = false;
  sLat = sLon = dLat = dLon = 0;
}

void AvcNav::startSampling(AvcLcd *lcd) {
  if (sampling) {
    return;
  }
  waypointSamplingIndex++;
  numWaypointsSet++;
//  AvcEeprom::setWayCount(numWaypointsSet);
  sampling = true;
  samples = 0;
  tempWaypoint = new AvcGps();
  lcd->setMode(lcd->SAMPLING);
}

float AvcNav::crossTrackError () {
  int multiplier = 1;
  // this will tell me if the car is on the right or left of the line
  float plusminus = ((dLat - sLat) * (sLon - getLongitude()) - (sLat - getLatitude()) * (dLon - sLon));
  if (plusminus < 0) {
    multiplier = -1;
  }
  float a = TinyGPS::distance_between(toFloat(sLat), 0.0f, toFloat(dLat), toFloat(sLon - dLon));
  float b = TinyGPS::distance_between(toFloat(getLatitude()), 0.0f, toFloat(dLat), toFloat(getLongitude() - dLon));
  float c = TinyGPS::distance_between(toFloat(getLatitude()), 0.0f, toFloat(sLat), toFloat(getLongitude() - sLon));
  
  float cosc = (sq(a) + sq(b) - sq(c)) / (2 * a * b);
  float C = acos(cosc);
  float d = sin(C) * b;
  return float(multiplier) * d;
}

void AvcNav::updateSpeed(float timeDelta) {
  odometerSpeed = (.1222 / 2.0) / timeDelta;
}

void AvcNav::updateSpeed(AvcImu *imu) {
  odometerSpeed = imu->getOdometerSpeed();
}

void AvcNav::setSpeed(float fraction) {
//  if (abs(previousSpeed - fraction) > .0001) {
    previousSpeed = fraction;
#if BREAK_BEFORE_TURN
    float p = 0.0;
    if (fraction < 0) {
      p = max(fraction, -1.0);
    } else {
      p = min(fraction, 1.0);
    }
#else
    float p = constrain(fraction, -1.0, 1.0);
#endif
    speedServo.writeMicroseconds(1500 + int(p * 500.0));
//  }
}

void AvcNav::setMaxSpeed() {
  float pot = AvcLcd::getPotSpeed(200);
  maxSpeed = pot;
  settings->setMaximumSpeed(maxSpeed);
  settings->writeToEeprom();
//  AvcEeprom::setMaxSpeed(maxSpeed);
}

void AvcNav::drive () {
  if (killIt) {
    setSpeed(-0.5);
    return;
  }
#if BREAK_BEFORE_TURN
  if (startBreaking && millis() - breakingStartTime < 1000) {
    setSpeed(.25);
    return;
  } else {
    startBreaking = false;
  }
#endif
#if SPEED_THROUGH_TURN
  if (startSpeeding && millis() - speedingStartTime < 500) {
    setSpeed(min(maxSpeed + .2, 1.0));
    return;
  } else {
    startSpeeding = false;
  }
#endif
  if (maxSpeed > 0) {
    if (rampUpSpeed && previousRampUpSpeed < maxSpeed) {
      previousRampUpSpeed = min(previousRampUpSpeed + RAMP_UP_FACTOR, maxSpeed);
      setSpeed(previousRampUpSpeed);
    } else {
      setSpeed(maxSpeed);
    }
  } else {
    setSpeed(.25);
  }
}

void AvcNav::nuetral() {
  setSpeed(0);
}

#if USE_LINE_INTERSECT
// Compute bearing to point that intersects circle of radius nn
int AvcNav::lineCircle(long cLat, long cLon, long sLat, long sLon, long dLat, long dLon) {
  sLat -= cLat;
  sLon -= cLon;
  dLat -= cLat;
  dLon -= cLon;
  float feet = 40;
  float radius = (60 * feet) / 31000000;
  float dx1 = toFloat(dLon - sLon);
  float dy1 = toFloat(dLat - sLat);
  // compute the euclidean distance between A and B
  float lab = sqrt(dx1 * dx1 + dy1 * dy1);
  // compute the direction vector D from A to B
  float dvx = dx1 / lab;
  float dvy = dy1 / lab;
  // Now the line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.
  float t = dvx * toFloat(0.0 - sLon) + dvy * toFloat(0.0 - sLat);
  // compute the coordinates of the point E on line and closest to C
  float ex = t * dvx + toFloat(sLon);
  float ey = t * dvy + toFloat(sLat);
  // compute the euclidean distance from E to C
  float lec = sqrt(ex * ex + ey * ey);
#if EVASIVE_ACTION
  if (objectDetected && millis() - timeObjectDetected < 1000 && !reorienting) {
    lec += 6;
  } else {
    objectDetected = false;
  }
#endif
  // test if the line intersects the circle
  if (lec < radius) {
    // compute distance from t to circle intersection point
    float dt = sqrt(radius * radius - lec * lec);
    float iLon = (t + dt) * dvx + toFloat(sLon);
    float iLat = (t + dt) * dvy + toFloat(sLat);
    return (int) TinyGPS::course_to(0.0, 0.0, iLat, iLon);
  }
  // Return bearing from car to point where perpendicular intersects line
  return (int) TinyGPS::course_to(0.0, 0.0, ey, ex);
}

// Compute bearing to point that intersects circle of radius nn
int AvcNav::lineCircle() {
  
//  sLat -= cLat;
//  sLon -= cLon;
//  dLat -= cLat;
//  dLon -= cLon;
  float feet = 15;
  float radius = (60 * feet) / 31000000;
  float distanceBetweenWaypoints = abs(TinyGPS::distance_between(toFloat(sLat), 0.0f, 
      toFloat(dLat), toFloat(dLon - sLon)));
  float dx1 = toFloat(dLon - sLon);
  float dy1 = toFloat(dLat - sLat);
//  // compute the euclidean distance between A and B
//  float lab = sqrt(dx1 * dx1 + dy1 * dy1);
  // compute the direction vector D from A to B
  float dvx = dx1 / distanceBetweenWaypoints;
  float dvy = dy1 / distanceBetweenWaypoints;
  // Now the line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.
  float t = dvx * toFloat(longitude - sLon) + dvy * toFloat(latitude - sLat);
  // compute the coordinates of the point E on line and closest to C
  float ex = t * dvx + toFloat(sLon - longitude);
  float ey = t * dvy + toFloat(sLat);
  // compute the euclidean distance from E to C
  float distanceToLine = abs(TinyGPS::distance_between(toFloat(latitude), 0.0f, 
      toFloat(ey), toFloat(longitude - ex)));
//  float lec = sqrt(ex * ex + ey * ey);
  // test if the line intersects the circle
  if (distanceToLine < radius) {
    // compute distance from t to circle intersection point
    float dt = sqrt(radius * radius - distanceToLine * distanceToLine);
    float iLon = (t + dt) * dvx + toFloat(longitude - sLon);
    float iLat = (t + dt) * dvy + toFloat(sLat);
    return (int) TinyGPS::course_to(latitude, 0.0, iLat, iLon);
  }
  // Return bearing from car to point where perpendicular intersects line
  return (int) TinyGPS::course_to(latitude, 0.0, ey, ex);
}
#endif

int AvcNav::getLatPotentialOffset () {
  long eLat = path->getLatitude(0);
//  AvcEeprom::readLatLon(0, &eLat, &eLon);
  return latitude - eLat;
}

int AvcNav::getLonPotentialOffset () {
  long eLon = path->getLongitude(0);
//  AvcEeprom::readLatLon(0, &eLat, &eLon);
  return longitude - eLon;
}

//void AvcNav::setOffset () {
//  long eLat = path->getLatitude(0);
//  long eLon = path->getLongitude(0);
////  AvcEeprom::readLatLon(0, &eLat, &eLon);
//  int oLat, oLon;
//  oLat = latitude - eLat;
//  oLon = longitude - eLon;
//  AvcEeprom::setRunOffset(oLat, oLon);
//}

void AvcNav::nextRunLocation () {
  runLocation = (runLocation + 1) % LOC_COUNT;
//  AvcEeprom::setRunLocation(runLocation);
//  runLocation = AvcEeprom::getRunLocation();
  updateWaypoints();
  pickWaypoint();
}

void AvcNav::setRampUpSpeed(boolean r) {
  rampUpSpeed = r;
}

void AvcNav::processCamera(AvcImu* imu) {
  cameraX1 = imu->getCameraX1();
  cameraY1 = imu->getCameraY1();
  cameraX2 = imu->getCameraX2();
  cameraY2 = imu->getCameraY2();
  if (cameraX1 < 1023 && cameraX1 > 0 
      && cameraY1 < 1023 && cameraY1 > 0
      || cameraX2 < 1023 && cameraX2 > 0 
      && cameraY2 < 1023 && cameraY2 > 0) {
    if (abs(cameraX1 - previousCameraX1) < 10
        && abs(cameraY1 - previousCameraY1) < 10
        || abs(cameraX2 - previousCameraX2) < 10
        && abs(cameraY2 - previousCameraY2) < 10) {
      objectDetected = true;
      timeObjectDetected = millis(); 
    }
    previousCameraX1 = cameraX1;
    previousCameraY1 = cameraY1;
    previousCameraX2 = cameraX2;
    previousCameraY2 = cameraY2;
  }
}

