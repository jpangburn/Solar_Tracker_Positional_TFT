/** This program operates a linear actuator to move a solar tracker based on time of day
 * 
 * 
 */

//TODO: sleep doesn't work on the Due, maybe need to replace with another MEGA 2560
//#include <avr/sleep.h>
#include <RTClib.h>
#include <math.h>
#include <SolarPosition.h>
#include "enum_types.h"
// this include pulls settings specific to a given installation:
// FULL_WEST_POSITION, LATITUDE, LONGITUDE, X0-X3 for polynomial regression calculation
// and MINIMUM_ACTUATOR_MOVEMENT
#include "install_specific_settings.h"

// display/touch screen imports
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

// START wake reason and system status declarations
// keep a list of reasons why the device wakes up so we know what to do
enum WakeReason { RESET,
                  RTC_ALARM,
                  STATUS_UPDATE,
                  UNKNOWN,
                  LAST };
// maintain this array with values that will be logged as Serial.print (only shows in IDE so no length limit) (do not put an entry for LAST)
const char* wakeReasonStrings[] = { "Reset", "RTC Alarm", "Status Update", "Unknown" };
// make sure our wakeReasonStrings list size matches our enum size except for LAST.  This is a compile time check.
static_assert(sizeof(wakeReasonStrings)/sizeof(wakeReasonStrings[0]) == LAST, "bad wakeReasonStrings array size");
volatile WakeReason currentWakeReason = RESET;
// system status possibilities
enum SystemStatus { NEEDS_SETUP,
                    TRACKING,
                    TRACKING_DISABLED,
                    MOTOR_SENSING_ERROR, // statuses at this position and later are fatal and block tracking (see isFatalError below)
                    RTC_LOST_POWER, // this status actually requires the device to be reprogrammed to set the correct time again
                    LAST_STATUS // never use this status, it's just for compile checks.  It should always be actually last in the list.
                  };
// maintain this array with values that can be displayed as a status in a 26 character TFT display (do not put an entry for LAST)
const char* systemStatusStrings[] = { "Setup Required", "Tracking ON", "Tracking Disabled", "Motor Sensing Error", "RTC Lost Power"};
// make sure our systemStatusStrings list size matches our enum size except for LAST.  This is a compile time check.
static_assert(sizeof(systemStatusStrings)/sizeof(systemStatusStrings[0]) == LAST_STATUS, "bad systemStatusStrings array size");
// keep track of the current system status
SystemStatus currentSystemStatus = NEEDS_SETUP;
// some shortcut functions to do common checks of system status
bool isTracking() {
  return currentSystemStatus == TRACKING;
}
bool isFatalError() {
  return currentSystemStatus >= MOTOR_SENSING_ERROR;
}
// END wake reason and system status declarations

// START external serial display declarations
MCUFRIEND_kbv tft;
#define MINPRESSURE 100
#define MAXPRESSURE 800

// ALL Touch panels and wiring are DIFFERENT
// copy-paste results from TouchScreen_Calibr_native.ino
// though for me that program didn't work on my Due
//const int XP = 6, XM = A2, YP = A1, YM = 7; //ID=0x9341
//const int TS_LEFT = 907, TS_RT = 136, TS_TOP = 942, TS_BOT = 139;
const int XP=8,XM=A2,YP=A3,YM=9; //240x320 ID=0x9341
const int TS_LEFT=118,TS_RT=931,TS_TOP=72,TS_BOT=887;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

int pixel_x, pixel_y;     //Touch_getXY() updates global vars
bool Touch_getXY(void)
{
  // the point pressure is unreliable- it may read 200 several times then a 0
  // so we need to take several readings and see if more than 1 fall in the bounds
  // because not touching gets spurious readings like 0, -1, -11000, 13000, etc
  const int READ_COUNT = 10;
  int hitCount = 0;
  TSPoint p;
  for (int i=0; i<READ_COUNT; i++){
    p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    //Serial.print("p.z: ");
    //Serial.println(p.z);
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE){
      hitCount++;
    }
  }

    
  // did we get enough hit counts?
  if (hitCount >  2){
    // yes, so use the last hit's x/y because x/y is reliable
    pixel_x = map(p.y, TS_TOP, TS_BOT, 0, 320);
    pixel_y = map(p.x, TS_RT, TS_LEFT, 0, 240);
    // return true to indicate pressed
    return true;
  } else {
    // return false to indicate no press
    return false;
  }
/* 
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    Serial.print("p.z: ");
    Serial.print(p.z);
    Serial.print(" p.x: ");
    Serial.print(p.x);
    Serial.print(" p.y: ");
    Serial.println(p.y);
    if (pressed) {
      // portrait mapping
      //pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me
      //pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
      // landscape mapping
      pixel_x = map(p.y, TS_TOP, TS_BOT, 0, 320);
      pixel_y = map(p.x, TS_RT, TS_LEFT, 0, 240);
      /*
      PORTRAIT  CALIBRATION     240 x 320
x = map(p.x, LEFT=118, RT=931, 0, 240)
y = map(p.y, TOP=72, BOT=887, 0, 320)

LANDSCAPE CALIBRATION     320 x 240
x = map(p.y, LEFT=72, RT=887, 0, 320)
y = map(p.x, TOP=931, BOT=118, 0, 240)
    }
    return pressed;*/
}

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Adafruit_GFX_Button east_btn, west_btn, east_auto_btn, west_auto_btn;
const int DISPLAY_POWER_PIN = 53;
const int SCREEN_SIZE = 40;
// how long does it take the screen after receiving power to be ready to receive commands
const int POWERUP_DELAY = 750;
// keep track if the screen is already on so we know when the user is watching if we should send display updates
bool screenAlreadyOn = 0;
// these buffers hold data to go to the screen
char line1Buffer[SCREEN_SIZE];
char line2Buffer[SCREEN_SIZE];

void updateTextArea(){
  tft.fillRect(0,0,320,70, WHITE);
  tft.setCursor(2,3);
  tft.print(line1Buffer);
  tft.setCursor(2,23);
  tft.print(line2Buffer);
}
// call this to turn on the display and clear it (just clears if the display is already on)
void turnOnClearDisplay(){
  //digitalWrite(DISPLAY_POWER_PIN, HIGH);
  if (!screenAlreadyOn) {
    // need to wait before writing stuff to the display
    delay(POWERUP_DELAY);
    screenAlreadyOn = true;
    // initialize the screen
    uint16_t ID = tft.readID();
    Serial.print("TFT ID = 0x");
    Serial.println(ID, HEX);
    tft.begin(ID);
    tft.setRotation(1);   //landscape
    // initialize the buttons
    east_btn.initButton(&tft,  70, 105, 120, 40, WHITE, CYAN, BLACK, (char *)"MOVE EAST", 2);
    west_btn.initButton(&tft, 250, 105, 120, 40, WHITE, CYAN, BLACK, (char *)"MOVE WEST", 2);
    east_auto_btn.initButton(&tft,  70, 180, 120, 40, WHITE, CYAN, BLACK, (char *)"AUTO EAST", 2);
    west_auto_btn.initButton(&tft, 250, 180, 120, 40, WHITE, CYAN, BLACK, (char *)"AUTO WEST", 2);
  }
  // clear the screen and draw the UI
  tft.fillScreen(BLACK);
  east_btn.drawButton(false);
  west_btn.drawButton(false);
  east_auto_btn.drawButton(false);
  west_auto_btn.drawButton(false);

  updateTextArea();
}

void turnOffDisplay() {
  //digitalWrite(DISPLAY_POWER_PIN, LOW);
  tft.fillScreen(BLACK);
  
  //screenAlreadyOn = false;
}

// Array of display button addresses to behave like a list
Adafruit_GFX_Button *buttons[] = {&east_btn, &west_btn, &east_auto_btn, &west_auto_btn, NULL};

/* update the state of a button and redraw as reqd
 *
 * main program can use isPressed(), justPressed() etc
 */
bool updateButton(Adafruit_GFX_Button *b, bool down)
{
    b->press(down && b->contains(pixel_x, pixel_y));
    if (b->justReleased())
        b->drawButton(false);
    if (b->justPressed()){
      //Serial.println("touchscreen button just pressed");
        b->drawButton(true);
    }
    if (b->isPressed()){
      b->drawButton(true);
    }
    return down;
}

/* most screens have different sets of buttons
 * life is easier if you process whole list in one go
 */
bool updateButtonList(Adafruit_GFX_Button **pb)
{
    bool down = Touch_getXY();
    for (int i = 0 ; pb[i] != NULL; i++) {
        updateButton(pb[i], down);
    }
    return down;
}
// END external display declarataions

// START Control button declarations
// status button
const int STATUS_BUTTON_PIN = 18;  // input pin (for a pushbutton switch)
// when you push the status button, this ISR fires.  It sets wake reason so loop() code knows what woke it up.
void onStatusButtonISR() {
  currentWakeReason = STATUS_UPDATE;
}
// buttons are pressed if they return a 1, so invert the logic to return true/false
bool isStatusButtonPressed() {
  return !digitalRead(STATUS_BUTTON_PIN);
}

bool isAnyButtonPressed() {
  bool displayButtonPressed = updateButtonList(buttons);
  return isStatusButtonPressed() || displayButtonPressed;
}
// END Control button declarations

// START actuator declarations
// use an impossible position for unknown actuator position
const int ACTUATOR_POSITION_UNKNOWN = -10000;
// keep track of the actuator position
int actuatorPosition = ACTUATOR_POSITION_UNKNOWN;
// define the pins involved
const int MOTOR_SIGNALPOWER_PIN = 22;
const int MOTOR_IN1_PIN = 26;
const int MOTOR_IN2_PIN = 28;
const int MOTOR_ENA1_PIN = 24;
const int MOTOR_GROUND_PIN = 30;
const int ACTUATOR_COUNTER_PIN = 32;
void moveEast() {
  // IN1 high, IN2 low
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}
void moveWest() {
  // IN1 low, IN2 high
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
}
void stopMoving() {
  // both high is float (both low is brake, which we don't need to waste power on)
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
}
void disableTracking() {
  // to set the right status we need to know if we have a known actuator position or not
  if (actuatorPosition == ACTUATOR_POSITION_UNKNOWN){
    currentSystemStatus = NEEDS_SETUP;
  } else {
    // actuator position is known, so just set status to disabled so user can re-enable
    currentSystemStatus = TRACKING_DISABLED;
  }
}
// actuator sensor pin averaging
const float ALPHA = 0.1; // lower number means changing values take longer to take effect (spikes must last longer to be counted)
float accumulator = 0; // keeps a running "average" of the sensor value
const int ACCUMULATOR_PRIME_LENGTH = 30; // about how many milliseconds to prime the accumulator with before starting the motor
// remember the last actuator sensor status (so we can tell when it flips)
bool lastActuatorStatus = LOW;
const float HIGH_SWITCH_LEVEL = 0.7; // threshold level to switch to a high state
const float LOW_SWITCH_LEVEL = 0.3;
// read the actuator counter pin sensor and update the accumulator with a new value
void updateAccumulator(){
  accumulator = (ALPHA * digitalRead(ACTUATOR_COUNTER_PIN)) + (1.0 - ALPHA) * accumulator;
}
// prime the accumulator before starting the motor so it starts out at the current state of the sensor value
void setStartingStatus(){
  for (int i=0; i < ACCUMULATOR_PRIME_LENGTH; i++){
    updateAccumulator();
    delay(1);
  }
  lastActuatorStatus = accumulator >= 0.5 ? HIGH : LOW;
}
// watch the status pin for a certain number of ticks (~ milliseconds) while updating actuatorPosition based on movement direction
void watchStatusPin(const int ticks, const MoveDirection movementDirection){
  for (int i=0; i < ticks; i++){
    updateAccumulator();
    // did it flip?
    if ( (lastActuatorStatus == LOW && accumulator > HIGH_SWITCH_LEVEL) ||
          (lastActuatorStatus == HIGH && accumulator < LOW_SWITCH_LEVEL)) {
      // either case is a flip, so change lastActuatorStatus
      lastActuatorStatus = !lastActuatorStatus;
      // caller tells us if we're moving east or west (can't just read motor because it can still be moving shortly after stop command)
      // but we don't update the position if actuatorPosition is currently unknown
      if (actuatorPosition != ACTUATOR_POSITION_UNKNOWN){
        if (movementDirection == EAST){
          actuatorPosition--;
        } else {
          // else we were moving west, so add to the actuator position
          actuatorPosition++;
        }
      }
    }
    delay(1);
  }
}
// helper move function that moves until a condition is met, or until an abort condition is met
// if aborted or error, tracking will be disabled
// returns true if aborted or error, false if move completed normally
template <class Callable1, class Callable2>
bool moveUntilConditionOrAbort(const MoveDirection moveDirection, Callable1 moveUntilConditionFunction, Callable2 abortFunction){
  // remember if we aborted or not
  bool aborted = false;
  // read actuator sensor pin for a bit to get starting value
  setStartingStatus();
  // watch the motor based on time to make sure it's moving
  unsigned long lastCheckMillis = millis();
  int lastCheckActuatorPosition = actuatorPosition;
  // turn on the motor
  if (moveDirection == WEST){
    moveWest();
  } else {
    moveEast();
  }
  // wait until we get to our destination
  while( !moveUntilConditionFunction() ){
    // watch the status pin for 20 ms, and tell it what direction we're moving so it updates actuatorPosition correctly
    watchStatusPin(20, moveDirection);
    // check the abort condition so the user can halt movement (or other abort condition)
    if (abortFunction()){
      aborted = true;
      // disable tracking
      disableTracking();
      // break out of the while loop
      break;
    }
    // if the motor has moved since the last time we checked, update the lastCheckMillis
    // but only do this if the motor position is known when we started
    if (actuatorPosition != ACTUATOR_POSITION_UNKNOWN){
      if (lastCheckActuatorPosition != actuatorPosition){
        // motor has moved
        lastCheckActuatorPosition = actuatorPosition;
        lastCheckMillis = millis();
      } else if (millis() - lastCheckMillis > 750) {
        // motor hasn't moved for 3/4 of a second, way too long!  Set motor sensing error (a fatal error) and break out of the while loop
        currentSystemStatus = MOTOR_SENSING_ERROR;
        // assume we've lost actuator position
        actuatorPosition = ACTUATOR_POSITION_UNKNOWN;
        // this also should result in the abort flag being returned
        aborted = true;
        break;
      }
    }
  }
  // stop the motor
  stopMoving();
  // watch the status pin for a while while motor is stopping (motor stops quickly, so this is probably overly cautious)
  watchStatusPin(300, moveDirection);
  // return the aborted status
  return aborted;
}

// calculate position from desired azimuth
float calculatePositionFromAzimuth(float azimuth){
  float position = X3 * pow(azimuth, 3.0) + X2 * pow(azimuth, 2.0) + X1*azimuth + X0;
  return position;
}
// END actuator declarations

// START RTC declarations
// define the RTC device (ours is a DS3231)
RTC_DS3231 rtc;
// the pin that is connected to RTC's SQW
const int CLOCK_INTERRUPT_PIN = 19;
// there's not a free 3.3v power pin to power the clock, so we use a digital pin to power it (5V power to clock causes spurious errors because Due expects 3.3)
const int CLOCK_POWER_PIN = 23;
// RTC alarm counter (volatile because it's modified by an ISR)
volatile byte alarmCounter = 0;
// interrupt service routine that fires when the clock sends us a tick
// the clock sends us a tick every minute, the keeps a count of those ticks and sets current wake reason so the loop() code knows what woke it up
void onRTCAlarmISR() {
  currentWakeReason = RTC_ALARM;
  alarmCounter += 1;
}
// END RTC declarations

// Show the time, position, tracker status, and whatever message is sent by caller
// DO NOT CALL this when motor is moving.  It can take a long time if screen is off so you'll miss motor ticks
void showTimePosMessage(const char* message) {
  // block this if motor is on
  if (digitalRead(MOTOR_IN1_PIN) == 0 || digitalRead(MOTOR_IN2_PIN) == 0){
    Serial.println(F("BAD PROGRAMMER: showTimePosMessage called while motor is engaged, ignoring"));
    return;
  }
  // turn on and clear the display
  turnOnClearDisplay();
  // if there's a fatal error (like RTC lost power), only display an error message
  if (isFatalError()){
    snprintf(line1Buffer, SCREEN_SIZE, "FATAL ERROR!");
    snprintf(line2Buffer, SCREEN_SIZE, systemStatusStrings[currentSystemStatus]);
  } else {
    // load current time, position, and tracking status to line 1
    DateTime now = rtc.now();
    // is there a conditional with formatting?  Without that need IF statement to print UNK for actuator position
    if (actuatorPosition != ACTUATOR_POSITION_UNKNOWN){
      snprintf(line1Buffer, SCREEN_SIZE, "%02d:%02d Pos %04d Track: %s", now.hour(), now.minute(), actuatorPosition, isTracking() ? "Y" : "N");
    } else {
      snprintf(line1Buffer, SCREEN_SIZE, "%02d:%02d Pos UNKNOWN Track: %s", now.hour(), now.minute(), isTracking() ? "Y" : "N");
    }
    // copy message to line2Buffer and print
    snprintf(line2Buffer, SCREEN_SIZE, "%s", message);
  }
  // in either case, update the print area
  updateTextArea();
}

// this is called at the end of the main loop()
void sleepAtLoopEnd() {
  // sleeping too fast after sending output to the serial port will result in partially transmitted data
  // so flush it and wait 10 ms before sleeping
  Serial.flush();
  delay(10); // TODO: this is likely not necessary, remove with testing
  // turn off the builtin LED when we're asleep
  digitalWrite(LED_BUILTIN, LOW);
  // turn off display
  turnOffDisplay();
  // set sleep mode to minimum power
  // TODO: fix this power down
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //sleep_enable();  // enables the sleep bit in the mcucr register
  // enable our interrupts that are disabled while we're awake
  attachInterrupt(digitalPinToInterrupt(STATUS_BUTTON_PIN), onStatusButtonISR, FALLING);
  // clock interrupts are only used if a fatal error has not occurred (normal state of affairs)
  if (!isFatalError()){
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onRTCAlarmISR, FALLING);
  }
  // clear alarm 1 in case it went off while we were awake
  // Note that it doesn't really matter if we miss one while doing something, we just don't
  // want it to be stuck in the alarm state forever because it triggered while awake
  rtc.clearAlarm(1);
  // set the wake reason to UNKNOWN so if something wakes us up without setting a reason, it will be unknown
  currentWakeReason = UNKNOWN;
  // here the device is actually put to sleep
  //sleep_mode();
  // TODO: remove this while loop once sleep is implemented properly
  while(!rtc.alarmFired(1) && currentWakeReason == UNKNOWN){
    //Serial.print("doing fake delay sleep, get rid of this: ");
    //Serial.println(alarmCounter);
    delay(200);
  }
  // disable these interrupts during loop processing
  detachInterrupt(digitalPinToInterrupt(STATUS_BUTTON_PIN));
  detachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN));
  //sleep_disable();
  // turn on the builtin LED when we're awake
  digitalWrite(LED_BUILTIN, HIGH);
}

// the time based movement can call this to send movement instructions
void moveToDesiredPosition(int desiredPosition) {
  // verify the percentage is within 0 and 1 so we don't exceed limits
  if (desiredPosition < 0) {
    Serial.println(F("INFO: received instruction to move to a negative position, ignoring"));
  } else if (desiredPosition > FULL_WEST_POSITION) {
    Serial.println(F("INFO: received instruction to move to position beyond the full west position, ignoring"));
  } else if (!isTracking()) {
    Serial.println(F("WARNING: received instruction to move to a position but tracking is not enabled, ignoring"));
  } else {
    // it's a valid position, we should move
    // only move if the requested move is greater than the minimum
    if ((unsigned int)abs(desiredPosition - actuatorPosition) > MINIMUM_ACTUATOR_MOVEMENT){
      // big enough move requested, do it
      Serial.print(F("Moving to desired position "));
      Serial.print(desiredPosition);
      Serial.print(F(" from "));
      Serial.print(actuatorPosition); // safe to read here when we're not changing it in ISRs
      Serial.print(F("... "));
      // what's our desired move direction?  Typically west.
      MoveDirection moveDirection = WEST;
      if (desiredPosition < actuatorPosition) {
        // nope, it's east this time!
        moveDirection = EAST;
      }
      const char* moveString = moveDirection == WEST ? "moving west... " : "moving east... ";
      Serial.print(moveString);
      // if the screen is already on, user is watching and will want to know this
      if (screenAlreadyOn){
        showTimePosMessage(moveString);
      }

      // move in the move direction until we get to our desiredPosition, or the user hits any button
      moveUntilConditionOrAbort(moveDirection, [=](){
        return (moveDirection == WEST && actuatorPosition >= desiredPosition) || (moveDirection == EAST && actuatorPosition <= desiredPosition);
      }, isAnyButtonPressed);
      
      // if tracking is disabled, user aborted in the middle of tracking or there was an error.  We've stopped the motor and kept track of position, notify user what they did
      if (!isTracking()){
        // if this was user initiated (status is tracking disabled then notify them)
        if (currentSystemStatus == TRACKING_DISABLED){
          // user initiated abort
          showTimePosMessage("Tracking Disabled");
          // give them a few seconds to read it
          delay(2000);
          // wait for them to release the button (which may still be held, this lets them keep the message up longer if they want)
          while(isAnyButtonPressed());
        } else if (screenAlreadyOn) {
          // it must have been a fatal error and the screen is on, so notify the user (fatal error takes over message)
          showTimePosMessage("");
          // give them a few seconds to read it
          delay(4000);
        } // else fatal error when screen is off, do nothing
      } else {
        // this is the normal exit flow for tracking that's not interrupted by the user
        Serial.println(F("reached desired position for the current time"));
        // if the screen is already on, user is watching and will want to know this
        if (screenAlreadyOn){
          showTimePosMessage("move complete");
          // give them a few seconds to read it (longer than if they interrupted, because not watching so closely)
          delay(4000);
        }
      }
    } else {
      // movement not needed
      Serial.print(F("Movement not needed at this time"));
      // if the screen is already on, user is watching and will want to know this
      if (screenAlreadyOn){
        showTimePosMessage("move not needed");
        // give them a few seconds to read it
        delay(3000);
      }
    }
  }
}

// this looks at the current solar position, and moves as necessary depending on if tracking is enabled or not
SolarPosition* solarLocation;
void moveToCorrectPositionForCurrentTime() {
  Serial.println(F("In moveToCorrectPositionForCurrentTime"));
  if (isTracking()){
    Serial.println(F("tracking enabled"));
    // get the current solar position
    SolarPosition_t currentSolarPosition = solarLocation->getSolarPosition();
    // if the sun is below the horizon then we should be at the farthest east we can go
    // (so when the sun goes down at the end of the day, you move ASAP while battery is up)
    if (currentSolarPosition.elevation < 0){
      moveToDesiredPosition(0);
    } else {
      // the sun is up, so use the polynomial regression to calculate the desired position given the azimuth
      int desiredPosition = (int)calculatePositionFromAzimuth(currentSolarPosition.azimuth);

      // clamp values from 0 - FULL_WEST_POSITION
      if (desiredPosition < 0){
        desiredPosition = 0;
      } else if (desiredPosition > FULL_WEST_POSITION){
        desiredPosition = FULL_WEST_POSITION;
      }
      // request movement to this position (going to the same position over and over is ok, it just won't do anything when it sees there's no movement to make)
      moveToDesiredPosition(desiredPosition);
    }
  } else {
    // tracking is not enabled, so we should not move based on time
    Serial.println(F("tracking not enabled"));
  }
  Serial.println(F("Exit moveToCorrectPositionForCurrentTime"));
}

// wait a bit to see if the user pushes any of the setup buttons
void awaitSetupButtons() {
  Serial.println(F("In awaitSetupButtons"));
  // This can be called on startup or by the status button interrupt, make sure status button is released before proceeding
  while(isStatusButtonPressed()); // status button is hardware debounced, no need for delay for software debounce
  // at this point status button is released
  // keep track of the last time the user pushed a button so we can exit after they stop interacting with the device
  unsigned long lastInteraction = millis();
  // watch for button presses if the user has interracted with the device in the last 15 seconds
  while (millis() - lastInteraction < 15000) {
    // check for buttons being pressed
    updateButtonList(buttons);
    if (east_btn.isPressed() || west_btn.isPressed()) {
      delay(30); // these buttons are not hardware debounced, so delay before further checks if they're pressed or not
      // update tracking mode since user has manually moved
      // if status is TRACKING then set to TRACKING_DISABLED, otherwise leave it as it is
      if (isTracking()){
        disableTracking();
      }
      // what's our desired move direction? 
      MoveDirection moveDirection = west_btn.isPressed() ? WEST : EAST;
      const char* moveString = moveDirection == WEST ? "Manual move west" : "Manual move east";
      Serial.println(moveString);
      showTimePosMessage(moveString);
      // move east or west until corresponding button is released
      moveUntilConditionOrAbort(moveDirection, [=](){
        updateButtonList(buttons);
        return (moveDirection == WEST && !west_btn.isPressed()) || (moveDirection == EAST && !east_btn.isPressed());
      }, [=](){
        // no abort, just wait until they release the button
        return false;
      });
      showTimePosMessage("stopped moving");
      // in some cases the user might still be holding a button down, wait for all buttons to be released
      while(isAnyButtonPressed());
      // keep track of last interaction time
      lastInteraction = millis();
    } else if (east_auto_btn.isPressed() || west_auto_btn.isPressed()) {
      // they want to auto move
        // auto move is only allowed if actuator position is known
        if (actuatorPosition != ACTUATOR_POSITION_UNKNOWN){
          // auto move is ok
        MoveDirection moveDirection = west_auto_btn.isPressed() ? WEST : EAST;
        const char* moveString = moveDirection == WEST ? "Auto move west" : "Auto move east";
        Serial.println(moveString);
          // notify user of auto move
        showTimePosMessage(moveString);
          // wait for them to release the buttons
          while(isAnyButtonPressed());
          // auto move in the same direction until we reach the end position, or user presses any button to abort
          moveUntilConditionOrAbort(moveDirection, [=](){
            return (moveDirection == WEST && actuatorPosition >= (int)FULL_WEST_POSITION) ||
                    (moveDirection == EAST && actuatorPosition <= 0);
          }, isAnyButtonPressed);
          // we don't really care if they aborted or not, either way we've stopped moving and no need to do more
        showTimePosMessage("stopped moving");
        // in some cases the user might still be holding a button down (cancelling auto move), wait for all buttons to be released
        while(isAnyButtonPressed());
        } else {
          // auto move is not ok because actuator position is unknown
        showTimePosMessage("Can't auto move- position unknown");
      // in some cases the user might still be holding a button down (cancelling auto move), wait for all buttons to be released
      while(isAnyButtonPressed());
      }
      // keep track of last interaction time
      lastInteraction = millis();
    } else if (isStatusButtonPressed()){
      // if we're already tracking, then this aborts
      if (isTracking()){
        // disable tracking
        disableTracking();
        showTimePosMessage("Tracking disabled");
        // wait until button is released
        while (isStatusButtonPressed());
      } else {
        // tell user to release status button
        showTimePosMessage("Release button");
        // button is hardware debounced, wait until it's released
        while (isStatusButtonPressed());
        // tell user to press it again to track
        bool pressMeansSetZero;
        if (actuatorPosition == ACTUATOR_POSITION_UNKNOWN || actuatorPosition < 0){
          showTimePosMessage("Press to zero");
          pressMeansSetZero = true;
        } else {
          showTimePosMessage("Press to enable");
          pressMeansSetZero = false;
        }
        lastInteraction = millis();
        // wait for status button to be pressed or 5 seconds to elapse
        while (!isStatusButtonPressed() && millis() - lastInteraction < 5000);
        // if user was given the option to enable, but didn't take it, give them the option to zero
        if (!isStatusButtonPressed() && !pressMeansSetZero){
          showTimePosMessage("Press to zero");
          // sleep for a bit in case user was just pressing button
          // so they don't accidentally zero when they wanted to enable
          delay(1000);
          pressMeansSetZero = true;
          lastInteraction = millis();
        }
        // wait for status button to be pressed or 5 seconds to elapse (expires immediately if user was given option to zero the first time)
        while (!isStatusButtonPressed() && millis() - lastInteraction < 5000);
        // if the button is still pressed then start tracking
        if (isStatusButtonPressed()) {
          // wait until button is released
          while (isStatusButtonPressed());
          // if pressMeansSetZero is set, then set position to zero
          if (pressMeansSetZero){
            // user is manually setting zero point
            actuatorPosition = 0;
            showTimePosMessage("Position zeroed");
            // alert user to watch out, the system will soon move to current location for the time
            delay(3000);
            // system is ready to track, enable it
            currentSystemStatus = TRACKING;
            showTimePosMessage("Danger! Tracking");
          } else {
            // else the position is already set so button press means to just enable
            // system is ready to track, enable it
            currentSystemStatus = TRACKING;
            showTimePosMessage("Tracking enabled");
            // alert user to watch out, the system will soon move to current location for the time
            delay(3000);
            showTimePosMessage("Danger! Tracking");
          }
        } else {
          // user did not confirm in time allotted, alert them tracking is aborted
          showTimePosMessage("Tracking aborted");
          // leave tracking disabled
        }
      }
      // update lastInteraction time so the loop waits
      lastInteraction = millis();
    }

    // no delay between loops is necessary because buttons should be released before getting here
  }
  Serial.println(F("exit awaitSetupButtons"));
  // move to the position specified by the current time (it will not move if tracking is disabled)
  moveToCorrectPositionForCurrentTime();
}

// the SolarTime library needs UTC unixtime seconds for its calculations
time_t getUTCSeconds() {
  return rtc.now().unixtime();
}

// setup the arduino
void setup() {
  // setup communication with IDE
  Serial.begin(57600);
#ifndef ESP8266
  while (!Serial)
    ;  // wait for serial port to connect. Needed for native USB
#endif
  Serial.println(F("Tracker program reset"));
  // setup LED builtin pin and turn it on to show awake
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // START actuator setup
  digitalWrite(MOTOR_ENA1_PIN, HIGH);
  pinMode(MOTOR_ENA1_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, HIGH);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_GROUND_PIN, LOW);
  pinMode(MOTOR_GROUND_PIN, OUTPUT);
  digitalWrite(MOTOR_SIGNALPOWER_PIN, HIGH);
  pinMode(MOTOR_SIGNALPOWER_PIN, OUTPUT);
  pinMode(ACTUATOR_COUNTER_PIN, INPUT_PULLUP);
  // END actuator setup

  // START external display setup
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  turnOnClearDisplay();
  snprintf(line1Buffer, SCREEN_SIZE, "%s", "Initializing");
  snprintf(line2Buffer, SCREEN_SIZE, "line two available");
  updateTextArea();
  Serial.println(F("connected to external display"));
  // END external display setup

  // START RTC setup
  // turn power on to the clock, give it a couple seconds to startup though probably doesn't need it because of the battery
  digitalWrite(CLOCK_POWER_PIN, HIGH);
  pinMode(CLOCK_POWER_PIN, OUTPUT);
  delay(2000);
  // initialize communication with the rtc
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC!"));
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    currentSystemStatus = RTC_LOST_POWER;
    Serial.println(F("**************"));
    Serial.println(F("RTC LOST POWER"));
    Serial.println(F("**************"));
    // this will adjust to the date and time at compilation
    // if the RTC has lost power, and you've fixed it, uncomment the following line
    // then upload the program to the Arduino.  This will set the time from your computer.
    // Then comment the line out again and upload again.
    // You do NOT want this line executing at the tracker
    // when it's reset and the RTC has lost power or the clock will be set wrong
    // WARNING: You must also set the timezone offset here so it computes UTC time correctly when it compiles
    // because the DATE and TIME macros are local time.
    /*
    #define TZ_OFFSET -8 // Pacific Standard Time (Pacific Daylight Time is -7)
    DateTime localDateTime = DateTime(F(__DATE__), F(__TIME__));
    time_t localSeconds = localDateTime.unixtime();
    // convert local seconds to UTC seconds and date time
    time_t utcSeconds = localSeconds - TZ_OFFSET * SECS_PER_HOUR;
    DateTime utcDateTime = DateTime(utcSeconds);
    // set the RTC with utcDateTime
    rtc.adjust(utcDateTime);
    */
  }

  //we don't need the RTC's 32K Pin, so disable it
  rtc.disable32K();

  // set RTC interrupt pin as INPUT_PULLUP
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);

  // set alarm 1 flag to false (so alarm 1 didn't happen so far)
  // disable alarm2 because it's not used
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disableAlarm(2);

  // stop oscillating signals at SQW Pin
  // otherwise setAlarm1 will fail
  rtc.writeSqwPinMode(DS3231_OFF);

  // schedule alarm1 10 seconds in the future to go over minute- we use this to move every 10 minutes or so
  if (!rtc.setAlarm1(
        rtc.now() + TimeSpan(10),
        DS3231_A1_Second  // this mode triggers the alarm when the seconds match. See Doxygen for other options
        )) {
    Serial.println(F("Error, alarm1 wasn't set!"));
  } else {
    Serial.println(F("Alarm1 will happen in 10 seconds!"));
  }

  Serial.flush();
  // END RTC setup

  // START SolarPosition setup
  // initialize a SolarPosition class instance (which seems more like a "solar location") with lat/long for the tracker's actual location
  solarLocation = new SolarPosition(LATITUDE, LONGITUDE);
  // tell it where to get time from (the RTC initialized above)
  solarLocation->setTimeProvider(getUTCSeconds);
  // END SolarPosition setup

  // START Control button setup
  pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP);
  // END Control button setup
}

void loop() {
  DateTime now;
  char timeBuffer[] = "hh:mm:ss";
  Serial.print(F("Wakeup for reason: "));
  Serial.println(wakeReasonStrings[currentWakeReason]);
  // if there's a fatal error, then the only interrupt will be the status button
  // so in that case just show the message (which will show an error)
  // TODO: maybe should rewrite this fatal error thing so user can still use east/west buttons to move tracker
  //   in case there's an error and they want to leave the tracker in a good position while resolving the fatal error?
  //   Although in that case, maybe they're disconnecting the controller and can just move it by touching battery?
  if (isFatalError()){
    showTimePosMessage("");
    // don't do anything else
    delay(10000);
    return;
  }
  // check the currentWakeReason to see what we should be doing
  switch (currentWakeReason) {
    case RESET:
      Serial.println(F("Entering loop from RESET"));
      showTimePosMessage("Setup Required");
      awaitSetupButtons();
      break;

    case RTC_ALARM:
      // print current time
      now = rtc.now();
      sprintf(timeBuffer, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      Serial.println(timeBuffer);
      // resetting SQW and alarm 1 flag
      if (rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
        Serial.print(F("Alarm1 cleared, counter is: "));
        Serial.println(alarmCounter);
      }
      // how many times we've alarmed, if it's 10 then it's been 10 minutes and we should move
      if (alarmCounter >= 10) {
        alarmCounter = 0;
        // move to the position specified by the current time (it will not move if tracking is disabled)
        moveToCorrectPositionForCurrentTime();
      }

      break;

    case STATUS_UPDATE:
      Serial.println(F("Status button wakeup"));
      // show the current system status
      showTimePosMessage(systemStatusStrings[currentSystemStatus]);
      awaitSetupButtons();
      break;

    case UNKNOWN:
      Serial.println(F("ERROR: currentWakeReason was left at UNKNOWN"));
      break;

    default:
      Serial.println(F("ERROR: currentWakeReason setting not defined in loop's switch statement"));
      break;
  }

  // at the end of every loop, we go back to sleep until woken up again (by RTC, or interrupt button)
  sleepAtLoopEnd();
}
