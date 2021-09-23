/** This program operates a linear actuator to move a solar tracker based on time of day
 * 
 * 
 */

#include <avr/sleep.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include "enum_types.h"

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
// maintain this array with values that can be displayed as a status in a 16 character LCD display (do not put an entry for LAST)
const char* systemStatusStrings[] = { "Setup Required", "Tracking", "TrackingDisabled", "Motor Sense Err", "RTC Lost Power"};
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
// We don't receive, use an invalid pin for RX_PIN
const int RX_PIN = 255;
const int TX_PIN = 43;
const int SERIAL_POWER_PIN = 42;
// the display uses invert logic
const int INVERTED = 1;
/* 
Set up a new serial output using the pin definitions above. Note the 
argument "inverted," which instructs SoftwareSerial to output BPI/BPK-
compatible inverted-TTL serial (like RS-232, but without the +/- 
voltage swing).*/
SoftwareSerial mySerial = SoftwareSerial(RX_PIN, TX_PIN, INVERTED);
// these are commands to tell the screen to do something other than display a character
const char CLEAR_SCREEN[] = { (char)(254), 1, (char)(254), (char)(128), 0 };
const char GOTO_LINE_TWO[] = { (char)(254), (char)(192), 0 };
// SCREEN_SIZE is the displayable area plus 1 for '\0' null terminator
const int SCREEN_SIZE = 17;
// how long does it take the screen after receiving power to be ready to receive commands
const int POWERUP_DELAY = 750;
// keep track if the screen is already on so we know when the user is watching if we should send display updates
bool screenAlreadyOn = 0;
// these buffers hold data to go to the screen
char line1Buffer[SCREEN_SIZE];
char line2Buffer[SCREEN_SIZE];
char timeBuffer[] = "hh:mm:ss";

// call this to turn on the display and clear it (just clears if the display is already on)
void turnOnClearDisplay(){
  digitalWrite(SERIAL_POWER_PIN, HIGH);
  if (!screenAlreadyOn) {
    // need to wait before writing stuff to the display
    delay(POWERUP_DELAY);
    screenAlreadyOn = true;
  }
  mySerial.print(CLEAR_SCREEN);
}

void turnOffDisplay() {
  digitalWrite(SERIAL_POWER_PIN, LOW);
  screenAlreadyOn = false;
}
// END external serial display declarataions

// START actuator declarations
// use an impossible position for unknown actuator position
const int ACTUATOR_POSITION_UNKNOWN = -10000;
// what is the minimum amount of ticks to move?  (don't move if going to move less than this)
const unsigned int MINIMUM_ACTUATOR_MOVEMENT = 4;
// what is the full west position
const unsigned int FULL_WEST_POSITION = 910;
// what is our start and end time of day (24 hour time)?
const unsigned int EAST_START_HOUR = 11;
const unsigned int EAST_START_MINUTE = 7;
const unsigned int EAST_START_MINUTES = convertTimeToNumberOfMinutes(EAST_START_HOUR, EAST_START_MINUTE);
const unsigned int WEST_END_HOUR = 15; // don't forget 24 hour time
const unsigned int WEST_END_MINUTE = 54;
const unsigned int WEST_END_MINUTES = convertTimeToNumberOfMinutes(WEST_END_HOUR, WEST_END_MINUTE);
const unsigned int TOTAL_DAY_MINUTES = WEST_END_MINUTES - EAST_START_MINUTES;
static_assert( WEST_END_HOUR > EAST_START_HOUR, "the west end time must be later than the east start time (the sun moves from east to west you know)");
// what time of day do we do our move back to the east?  (MUST be later than the WEST_END_HOUR)
const unsigned int EAST_RETURN_HOUR = 20;
const unsigned int EAST_RETURN_MINUTE = 0;
static_assert( (EAST_RETURN_HOUR > WEST_END_HOUR) || (EAST_RETURN_HOUR == WEST_END_HOUR && EAST_RETURN_MINUTE > WEST_END_MINUTE), "east return time must be after west end time" );
// keep track of the actuator position
int actuatorPosition = ACTUATOR_POSITION_UNKNOWN;
const int MOVE_EAST_PIN = 6;
const int MOVE_WEST_PIN = 5;
const int ACTUATOR_COUNTER_PIN = 13;
void moveEast() {
  // make sure west movement pin is off
  digitalWrite(MOVE_WEST_PIN, LOW);
  // enable east movement pin
  digitalWrite(MOVE_EAST_PIN, HIGH);
}
void moveWest() {
  // make sure east movement pin is off
  digitalWrite(MOVE_EAST_PIN, LOW);
  // enable west movement pin
  digitalWrite(MOVE_WEST_PIN, HIGH);
}
void stopMoving() {
  digitalWrite(MOVE_EAST_PIN, LOW);
  digitalWrite(MOVE_WEST_PIN, LOW);
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
// END actuator declarations

// START RTC declarations
// define the RTC device (ours is a DS3231)
RTC_DS3231 rtc;
// the pin that is connected to RTC's SQW
const int CLOCK_INTERRUPT_PIN = 19;
// RTC alarm counter (volatile because it's modified by an ISR)
volatile byte alarmCounter = 0;
// interrupt service routine that fires when the clock sends us a tick
// the clock sends us a tick every minute, the keeps a count of those ticks and sets current wake reason so the loop() code knows what woke it up
void onRTCAlarmISR() {
  currentWakeReason = RTC_ALARM;
  alarmCounter += 1;
}
// END RTC declarations

// START Control button declarations
// status button
const int STATUS_BUTTON_PIN = 18;  // input pin (for a pushbutton switch)
// east and west buttons
const int EAST_BUTTON_PIN = 11;
const int WEST_BUTTON_PIN = 12;
// when you push the status button, this ISR fires.  It sets wake reason so loop() code knows what woke it up.
void onStatusButtonISR() {
  currentWakeReason = STATUS_UPDATE;
}
// buttons are pressed if they return a 1, so invert the logic to return true/false
bool isStatusButtonPressed() {
  return !digitalRead(STATUS_BUTTON_PIN);
}

bool isEastButtonPressed() {
  return !digitalRead(EAST_BUTTON_PIN);
}
bool isWestButtonPressed() {
  return !digitalRead(WEST_BUTTON_PIN);
}
bool isAnyButtonPressed() {
  return isStatusButtonPressed() || isEastButtonPressed() || isWestButtonPressed();
}
// END Control button declarations

// Show the time, position, tracker status, and whatever message is sent by caller
// DO NOT CALL this when motor is moving.  It can take a long time if screen is off so you'll miss motor ticks
void showTimePosMessage(const char* message) {
  // block this if motor is on
  if (digitalRead(MOVE_EAST_PIN) || digitalRead(MOVE_WEST_PIN)){
    Serial.println(F("BAD PROGRAMMER: showTimePosMessage called while motor is engaged, ignoring"));
    return;
  }
  // turn on and clear the display
  turnOnClearDisplay();
  // if there's a fatal error (like RTC lost power), only display an error message
  if (isFatalError()){
    mySerial.print(F("FATAL ERROR!"));
    mySerial.print(GOTO_LINE_TWO);
    mySerial.print(systemStatusStrings[currentSystemStatus]);
  } else {
    // load current time, position, and tracking status to line 1
    DateTime now = rtc.now();
    // is there a conditional with formatting?  Without that need IF statement to print UNK for actuator position
    if (actuatorPosition != ACTUATOR_POSITION_UNKNOWN){
      snprintf(line1Buffer, SCREEN_SIZE, "%02d:%02d P %03d T:%s", now.hour(), now.minute(), actuatorPosition, isTracking() ? "Y" : "N");
    } else {
      snprintf(line1Buffer, SCREEN_SIZE, "%02d:%02d P UNK T:%s", now.hour(), now.minute(), isTracking() ? "Y" : "N");
    }
    mySerial.print(line1Buffer);
    // copy message to line2Buffer and print
    snprintf(line2Buffer, SCREEN_SIZE, "%s", message);
    mySerial.print(GOTO_LINE_TWO);
    mySerial.print(line2Buffer);
  }
}

// this is called at the end of the main loop()
void sleepAtLoopEnd() {
  // sleeping too fast after sending output to the serial port will result in partially transmitted data
  // so flush it and wait 10 ms before sleeping
  Serial.flush();
  mySerial.flush();
  delay(10); // TODO: this is likely not necessary, remove with testing
  // turn off the builtin LED when we're asleep
  digitalWrite(LED_BUILTIN, LOW);
  // turn off display
  turnOffDisplay();
  // set sleep mode to minimum power
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();  // enables the sleep bit in the mcucr register
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
  sleep_mode();
  // disable these interrupts during loop processing
  detachInterrupt(digitalPinToInterrupt(STATUS_BUTTON_PIN));
  detachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN));
  sleep_disable();
  // turn on the builtin LED when we're awake
  digitalWrite(LED_BUILTIN, HIGH);
}

// the time based movement can call this to send movement instructions, or the nightly alarm to move east calls it too
void moveToRelativePercentage(float relativePercentage) {
  // verify the percentage is within 0 and 1 so we don't exceed limits
  if (relativePercentage < 0) {
    Serial.println(F("WARNING: received instruction to move to a negative percentage, ignoring"));
  } else if (relativePercentage > 1) {
    Serial.println(F("WARNING: received instruction to move to a percentage over 100%, ignoring"));
  } else if (!isTracking()) {
    Serial.println(F("WARNING: received instruction to move to a percentage but tracking is not enabled, ignoring"));
  } else {
    // it's a valid percentage, we should move
    // what's our desired position?
    int desiredPosition = FULL_WEST_POSITION * relativePercentage;
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
      while( (moveDirection == WEST && actuatorPosition < desiredPosition) || (moveDirection == EAST && actuatorPosition > desiredPosition) ){
        // watch the status pin for 20 ms, and tell it what direction we're moving so it updates actuatorPosition correctly
        watchStatusPin(20, moveDirection);
        // check the status button so the user can halt movement
        if (isStatusButtonPressed()){
          // disable tracking
          currentSystemStatus = TRACKING_DISABLED;
          // break out of the while loop
          break;
        }
        // if the motor has moved since the last time we checked, update the lastCheckMillis
        if (lastCheckActuatorPosition != actuatorPosition){
          // motor has moved
          lastCheckActuatorPosition = actuatorPosition;
          lastCheckMillis = millis();
        } else if (millis() - lastCheckMillis > 750) {
          // motor hasn't moved for 3/4 of a second, way too long!  Set motor sensing error (a fatal error) and break out of the while loop
          currentSystemStatus = MOTOR_SENSING_ERROR;
          // assume we've lost actuator position
          actuatorPosition = ACTUATOR_POSITION_UNKNOWN;
          break;
        }
      }
      // stop the motor
      stopMoving();
      // watch the status pin for a while while motor is stopping (motor stops quickly, so this is probably overly cautious)
      watchStatusPin(300, moveDirection);
      // if tracking is disabled, user aborted in the middle of tracking or there was an error.  We've stopped the motor and kept track of position, notify user what they did
      if (!isTracking()){
        // if this was user initiated (status is tracking disabled then notify them)
        if (currentSystemStatus == TRACKING_DISABLED){
          // user initiated abort
          showTimePosMessage("Tracking Disabled");
          // give them a few seconds to read it
          delay(2000);
          // wait for them to release the button (which may still be held, this lets them keep the message up longer if they want)
          while(isStatusButtonPressed());
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

// we need to do computations in proportion to hours and minutes, so make them into a number
unsigned int convertTimeToNumberOfMinutes(unsigned int hour, unsigned int minute) {
  return hour * 60 + minute;
}

// this looks at the current time and makes movements if it's within the daylight hours that we track, and tracking is enabled
void moveToCorrectPositionForCurrentTime() {
  Serial.println(F("In moveToCorrectPositionForCurrentTime"));
  if (isTracking()){
    Serial.println(F("tracking enabled"));
    // get the current time
    DateTime now = rtc.now();
    // convert to number of minutes relative to the EAST start
    int relativeMinutes = convertTimeToNumberOfMinutes(now.hour(), now.minute()) - EAST_START_MINUTES;
    // find what percent we are of the total minutes (can be negative or greater than 1 if outside the movement time)
    float relativePercentage = ((float)relativeMinutes) / TOTAL_DAY_MINUTES;
    /* debug code for computing how much to move
    Serial.print(F("EAST_START_MINUTES, TOTAL_DAY_MINUTES, relativeMinutes, relativePercentage: "));
    Serial.print(EAST_START_MINUTES);
    Serial.print(F(", "));
    Serial.print(TOTAL_DAY_MINUTES);
    Serial.print(F(", "));
    Serial.print(relativeMinutes);
    Serial.print(F(", "));
    Serial.println(relativePercentage);*/
    if (relativePercentage < 0) {
      Serial.println(F("Too early to start tracking"));
    } else if (relativePercentage > 1) {
      Serial.println(F("It's after hours, kicking back"));
    } else {
      // it's a valid percentage, we should move
      moveToRelativePercentage(relativePercentage);
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
    if (isEastButtonPressed() || isWestButtonPressed()) {
      delay(30); // these buttons are not hardware debounced, so delay before further checks if they're pressed or not
      // update tracking mode since user has manually moved
      // if status is TRACKING then set to TRACKING_DISABLED, otherwise leave it as it is
      if (isTracking()){
        currentSystemStatus = TRACKING_DISABLED;
      }
      // what's our desired move direction? 
      MoveDirection moveDirection = isWestButtonPressed() ? WEST : EAST;
      const char* moveString = moveDirection == WEST ? "Manual move west" : "Manual move east";
      Serial.println(moveString);
      showTimePosMessage(moveString);
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
      // wait until user lets go of the button
      bool autoMove = false;
      while( (moveDirection == WEST && isWestButtonPressed()) || (moveDirection == EAST && isEastButtonPressed()) ||
              (autoMove && moveDirection == WEST && actuatorPosition < (int)FULL_WEST_POSITION) ||
              (autoMove && moveDirection == EAST && actuatorPosition > 0) ){
        // if autoMove is false, then we keep watching for user to let go of the direction button above.  But if user presses status button while
        // still holding a move button then we switch to autoMove and keep moving until the end of the movement direction
        // do not allow automove if actuator position is unknown
        if (!autoMove && isStatusButtonPressed() && actuatorPosition != ACTUATOR_POSITION_UNKNOWN){
          // engage autoMove
          autoMove = true;
          // stop the motor so we can have time to update the screen to tell the user automove is on
          stopMoving();
          // watch the status pin for a while while motor is stopping (motor stops quickly, so this is probably overly cautious)
          watchStatusPin(300, moveDirection);
          // notify user of auto move
          showTimePosMessage("auto move ON");
          // wait for them to release the buttons
          while(isAnyButtonPressed());
          // update the motor movement variables since user could have held the button for a while, don't want that to falsely indicate sensor failure
          lastCheckActuatorPosition = actuatorPosition;
          lastCheckMillis = millis();
          // restart motor
          if (moveDirection == WEST){
            moveWest();
          } else {
            moveEast();
          }
        } else if (autoMove && isAnyButtonPressed()) {
          // any button halts, though if they hold down the same direction button it will be manually moving that way
          autoMove = false;
          // with no buttons pressed now the while loop will exit
        }
        // if the motor has moved since the last time we checked, update the lastCheckMillis
        if (lastCheckActuatorPosition != actuatorPosition){
          // motor has moved
          lastCheckActuatorPosition = actuatorPosition;
          lastCheckMillis = millis();
        } else if (millis() - lastCheckMillis > 750) {
          // motor hasn't moved for 3/4 of a second, way too long!  Set motor sensing error (a fatal error) and break out of the while loop
          currentSystemStatus = MOTOR_SENSING_ERROR;
          // assume we've lost actuator position
          actuatorPosition = ACTUATOR_POSITION_UNKNOWN;
          break;
        }
        // watch the status pin for 20 ms, and tell it what direction we're moving so it updates actuatorPosition correctly
        // it will leave actuatorPosition alone if it's unknown
        watchStatusPin(20, moveDirection);
      }
      // stop the motor
      stopMoving();
      // watch the status pin for a while while motor is stopping (motor stops quickly, so this is probably overly cautious)
      watchStatusPin(300, moveDirection);
      // notify user of new position
      showTimePosMessage("stopped moving");
      // in some cases the user might still be holding a button down (cancelling auto move), wait for all buttons to be released
      while(isAnyButtonPressed());
      // update lastInteraction time so the loop waits
      lastInteraction = millis();
    } else if (isStatusButtonPressed()){
      // if we're already tracking, then this aborts
      if (isTracking()){
        // set TRACKING_DISABLED instead of NEEDS_SETUP because we know position and it hasn't changed, so tracking can resume without setup
        currentSystemStatus = TRACKING_DISABLED; 
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
          pressMeansSetZero = true;
          lastInteraction = millis();
        }
        // wait for status button to be pressed or 5 seconds to elapse (expires immediately if user was given option to zero the first time)
        while (!isStatusButtonPressed() && millis() - lastInteraction < 5000);
        // if the button is still pressed then start tracking
        if (isStatusButtonPressed()) {
          // wait until button is released
          while (isStatusButtonPressed());
          // set system status to tracking
          currentSystemStatus = TRACKING;
          // if pressMeansSetZero is set, then set position to zero
          if (pressMeansSetZero){
            actuatorPosition = 0;
            showTimePosMessage("Position zeroed");
            // alert user to watch out, the system will soon move to current location for the time
            delay(3000);
            showTimePosMessage("Danger! Tracking");
          } else {
            // else the position is already set so button press means to just enable
            showTimePosMessage("Tracking enabled");
            // alert user to watch out, the system will soon move to current location for the time
            delay(3000);
            showTimePosMessage("Danger! Tracking");
          }
        } else {
          // user did not confirm in time allotted, alert them tracking is aborted
          showTimePosMessage("Tracking aborted");
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
  digitalWrite(MOVE_EAST_PIN, LOW);
  pinMode(MOVE_EAST_PIN, OUTPUT);
  digitalWrite(MOVE_WEST_PIN, LOW);
  pinMode(MOVE_WEST_PIN, OUTPUT);
  pinMode(ACTUATOR_COUNTER_PIN, INPUT_PULLUP);
  // END actuator setup

  // START external display setup
  digitalWrite(SERIAL_POWER_PIN, HIGH);
  pinMode(SERIAL_POWER_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);  // Stop bit state for inverted serial
  pinMode(TX_PIN, OUTPUT);
  mySerial.begin(9600);  // Set the data rate
  turnOnClearDisplay();
  snprintf(line1Buffer, SCREEN_SIZE, "%s", "Initializing");
  mySerial.print(line1Buffer);
  mySerial.print(GOTO_LINE_TWO);
  mySerial.print(F("line two available"));
  Serial.println(F("connected to external display"));
  // END external display setup

  // START RTC setup
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
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //we don't need the RTC's 32K Pin, so disable it
  rtc.disable32K();

  // set RTC interrupt pin as INPUT_PULLUP
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);

  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

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

  // schedule alarm2 to go every night to move back east
  if (!rtc.setAlarm2(
        DateTime(2021, 9, 11, EAST_RETURN_HOUR, EAST_RETURN_MINUTE, 0), // date doesn't matter, just hours/minutes/seconds
        DS3231_A2_Hour  // this mode triggers the alarm when the hour/minutes/seconds match. See Doxygen for other options
        )) {
    Serial.println(F("Error, alarm2 wasn't set!"));
  } else {
    Serial.println(F("Alarm2 will happen once per day to move east"));
  }
  Serial.flush();
  // END RTC setup

  // START Control button setup
  pinMode(STATUS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(EAST_BUTTON_PIN, INPUT_PULLUP);
  pinMode(WEST_BUTTON_PIN, INPUT_PULLUP);
  // END Control button setup
}

void loop() {
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
      rtc.now().toString(timeBuffer);
      Serial.println(timeBuffer);// resetting SQW and alarm 1 flag
      // clear the alarm 1 flag
      if (rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
        Serial.println(F("Alarm1 cleared"));
      }
      // how many times we've alarmed, if it's 10 then it's been 10 minutes and we should move
      if (alarmCounter >= 10) {
        alarmCounter = 0;
        // move to the position specified by the current time (it will not move if tracking is disabled)
        moveToCorrectPositionForCurrentTime();
      }
      // if alarm 2 fired, then it's time to do our nightly east move
      if (rtc.alarmFired(2)) {
        rtc.clearAlarm(2);
        Serial.println(F("Alarm2 cleared, moving east for the night"));
        // east is at 0% of the movement range
        moveToRelativePercentage(0);
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
