#include "AvcMenu.h"

AvcMenu::AvcMenu (AvcLcd *l, AvcNav *n) {
  lcd = l;
  nav = n;
  currentMenu = 0;
  refreshTime = millis();
  debouncing = false;
}

int AvcMenu::buttonPressed () {
  if (isButtonPressed(MENU_SCROLL_PIN)) {
    for (int ii = 1; ii < 4; ii++) {
      bounce[ii-1] = bounce[ii];
    }
    bounce[3] = MENU_SCROLL_PIN;
  } else if (isButtonPressed(MENU_SELECT_PIN)) {
    for (int ii = 1; ii < 4; ii++) {
      bounce[ii-1] = bounce[ii];
    }
    bounce[3] = MENU_SELECT_PIN;
  } else {
    for (int ii = 1; ii < 4; ii++) {
      bounce[ii-1] = bounce[ii];
    }
    bounce[3] = 0;
  }
  if (bounce[0] != bounce[3] && bounce[1] == bounce[3] && bounce[2] == bounce[3]) {
    return bounce[3];
  }
  return 0;
}

// send values SET_WAYPOINT_BUTTON or RESET_BUTTON
boolean AvcMenu::isButtonPressed (int button) {
  int buttonPressed = digitalRead(button);
  if (buttonPressed == LOW) {
    return true;
  }
  return false;
}

void AvcMenu::checkButtons (boolean refresh) {
  byte button = buttonPressed();
  if (button == MENU_SCROLL_PIN) {
    currentMenu = (currentMenu + 1) % ENUM_COUNT;
    refresh = true;
    lcd->setMode(lcd->NONE);
  } else if (button == MENU_SELECT_PIN) {
    switch(currentMenu) {
      case GPS:
        nav->updateWaypoints();
        break;
      case SAMPLE:
        nav->startSampling(lcd);
        break;
      case RESET:
        nav->resetWaypoints();
        break;
      case SLIDESHOW:
        lcd->setMode(lcd->WAYPOINTS);
        break;
      case SPEED:
        nav->setMaxSpeed();
        break;
//      case OFFSET:
//        nav->setOffset();
//        break;
      case MENU_RUN_LOC:
        nav->nextRunLocation();
        break;
    }
    refresh = true;
  }
  if (refresh || (millis() - refreshTime) > 500 && lcd->getMode() == lcd->NONE) {
    refreshTime = millis();
    switch(currentMenu) {
      case GPS:
        lcd->printGps(nav->getLatitude(), nav->getLongitude(), nav->getHdop(), refresh, nav->getNextWaypoint());
        break;
      case SAMPLE:
        if (refresh) {
          lcd->printStartSampling(nav->getNumWaypoints());
        }
        break;
      case RESET:
        if (refresh) {
          lcd->askReset(nav->getNumWaypoints());
        }
        break;
      case SLIDESHOW:
        if (refresh) {
          lcd->askWaypointSlideshow(nav->getNumWaypoints());
        }
        break;
      case SPEED:
        lcd->askSetMaxSpeed(nav->getMaxSpeed(), refresh);
        break;
      case HEADING:
        lcd->trackHeading(nav->getHeadingToWaypoint(), nav->getHeading(), refresh);
        break;
      case OFFSET:
        lcd->askSetOffset(nav->getLatPotentialOffset(), nav->getLonPotentialOffset(), refresh);
        break;
      case MENU_RUN_LOC:
        if (refresh) {
          lcd->showRunLocation(nav->getRunLocation());
        }
        break;
      case CAMERA:
        lcd->cameraValues(nav->getCameraX1(), nav->getCameraY1(), nav->getCameraX2(), nav->getCameraY2(), refresh);
        break;
    }
  }
}

