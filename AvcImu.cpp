#include "AvcImu.h"

AvcImu::AvcImu () {
  reset();
}

void AvcImu::parse (char c) {
  if (invalid) {
    if (c == '\n') {
      objectComplete = true;
    }
    return;
  }
  charCount++;
  if (charCount > BUF_SIZE) {
    invalid = true;
    return;
  }
  buf[charCount-1] = c;
  switch (c) {
    case '\n':
      objectComplete = true;
#if LOG_IMU
      print();
#endif
      return;
    case '\r': {
      buf[charCount-1] = NULL;
      // get checksum
      int count = atoi(&buf[checksumIdx + 1]);
      // test sum
      if (count != checksumIdx) {
        invalid = true;
        return;
      }
      char * pch;
      pch = strtok (buf,",");
      byte convertCount = 0;
      while (pch != NULL)
      {
        switch (convertCount) {
          case 0:
            if (strcmp(pch, "COMP") == 0) {
              mode = COMPASS;
            } else if (strcmp(pch, "GPS") == 0) {
              mode = GPS;
            } else if (strcmp(pch, "CAM") == 0) {
              mode = CAMERA;
            } else if (strcmp(pch, "MPU") == 0) {
              mode = MPU;
            } else if (strcmp(pch, "GCM") == 0) {
              mode = GCM;
            } else if (strcmp(pch, "CM") == 0) {
              mode = CM;
            } else {
              mode = IMU;
              latitude = atol(pch);
            }
            break;
          case 1:
            if (mode == COMPASS) {
              heading = atoi(pch);
            } else if (mode == GPS) {
              latitude = atol(pch);
            } else if (mode == IMU) {
              longitude = atol(pch);
            } else if (mode == CAMERA) {
              cameraX1 = atol(pch);
            } else if (mode == MPU) {
              accelX = atoi(pch);
            } else if (mode == GCM) {
              latitude = atol(pch);
            } else if (mode == CM) {
              heading = atoi(pch);
            }
            break;
          case 2:
            if (mode == COMPASS) {
              xOffset = atoi(pch);
            } else if (mode == GPS) {
              longitude = atol(pch);
            } else if (mode == IMU) {
              hdop = atof(pch);
            } else if (mode == CAMERA) {
              cameraY1 = atol(pch);
            } else if (mode == MPU) {
              accelY = atoi(pch);
            } else if (mode == GCM) {
              longitude = atol(pch);
            } else if (mode == CM) {
              xOffset = atoi(pch);
            }
            break;
          case 3:
            if (mode == COMPASS) {
              yOffset = atoi(pch);
            } else if (mode == GPS) {
              hdop = atof(pch);
            } else if (mode == IMU) {
              distanceTraveled = atof(pch);
            } else if (mode == CAMERA) {
              cameraX2 = atol(pch);
            } else if (mode == MPU) {
              accelZ = atoi(pch);
            } else if (mode == GCM) {
              hdop = atof(pch);
            } else if (mode == CM) {
              yOffset = atoi(pch);
            }
            break;
          case 4:
            if (mode == COMPASS) {
              zOffset = atoi(pch);
            } else if (mode == GPS) {
              distanceTraveled = atof(pch);
            } else if (mode == IMU) {
              fixTime = atol(pch);
            } else if (mode == CAMERA) {
              cameraY2 = atol(pch);
            } else if (mode == MPU) {
              gyroX = atoi(pch);
            } else if (mode == GCM) {
              distanceTraveled = atof(pch);
            } else if (mode == CM) {
              zOffset = atoi(pch);
            }
            break;
          case 5:
            if (mode == GPS) {
              fixTime = atol(pch);
            } else if (mode == IMU) {
              speed = atof(pch);
            } else if (mode == MPU) {
              gyroY = atoi(pch);
            } else if (mode == GCM) {
              fixTime = atol(pch);
            } else if (mode == CM) {
              accelX = atoi(pch);
            }
            break;
          case 6:
            if (mode == GPS) {
              speed = atof(pch);
            } else if (mode == IMU) {
              waasLock = atoi(pch);
            } else if (mode == MPU) {
              gyroZ = atoi(pch);
            } else if (mode == GCM) {
              speed = atof(pch);
            } else if (mode == CM) {
              accelY = atoi(pch);
            }
            break;
          case 7:
            if (mode == GPS) {
              waasLock = atoi(pch);
            } else if (mode == IMU) {
              heading = atoi(pch);
            } else if (mode == MPU) {
              temp = atoi(pch);
            } else if (mode == GCM) {
              waasLock = atoi(pch);
            } else if (mode == CM) {
              accelZ = atoi(pch);
            }
            break;
          case 8:
            if (mode == GCM) {
              heading = atoi(pch);
            } else if (mode == CM) {
              gyroX = atoi(pch);
            }
            break;
          case 9:
            if (mode == GCM) {
              xOffset = atoi(pch);
            } else if (mode == CM) {
              gyroY = atoi(pch);
            }
            break;
          case 10:
            if (mode == GCM) {
              yOffset = atoi(pch);
            } else if (mode == CM) {
              gyroZ = atoi(pch);
            }
            break;
          case 11:
            if (mode == GCM) {
              zOffset = atoi(pch);
            } else if (mode == CM) {
              temp = atoi(pch);
            }
            break;
          case 12:
            if (mode == GCM) {
              accelX = atoi(pch);
            } else if (mode == CM) {
              odometerSpeed = atof(pch);
            }
            break;
          case 13:
            if (mode == GCM) {
              accelY = atoi(pch);
            }
            break;
          case 14:
            if (mode == GCM) {
              accelZ = atoi(pch);
            }
            break;
          case 15:
            if (mode == GCM) {
              gyroX = atoi(pch);
            }
            break;
          case 16:
            if (mode == GCM) {
              gyroY = atoi(pch);
            }
            break;
          case 17:
            if (mode == GCM) {
              gyroZ = atoi(pch);
            }
            break;
          case 18:
            if (mode == GCM) {
              temp = atoi(pch);
            }
            break;
          case 19:
            if (mode == GCM) {
              odometerSpeed = atof(pch);
            }
            break;
        }
        convertCount++;
        pch = strtok (NULL, ",*");
      }
      return;
    }
    case '*':
      checksumIdx = charCount - 1;
      return;
  }
}

void AvcImu::reset () {
  latitude = 0;
  longitude = 0;
  hdop = 0;
  distanceTraveled = 0;
  fixTime = 0;
  speed = 0;
  waasLock = false;
  heading = 0;
  checksumIdx = 0;
  charCount = 0;
  invalid  = false;
  objectComplete = false;
  odometerSpeed = 0.0;
  mode = IMU;
  xOffset = 0;
  yOffset = 0;
  zOffset = 0;
  cameraX1 = 0;
  cameraY1 = 0;
  cameraX2 = 0;
  cameraY2 = 0;
}

