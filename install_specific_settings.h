// these settings are specific to a given solar tracker installation

// what is the full west position
const int FULL_WEST_POSITION = 855;
// what is the minimum amount of ticks to move?  (don't move if going to move less than this)
const unsigned int MINIMUM_ACTUATOR_MOVEMENT = 10;
// what is the lat/long of this tracker
const float LATITUDE = 33.11;
const float LONGITUDE = -116.98;
// polynomial regression constants for converting desired azimuth to actuator position
float X3 = 0.0003685;
float X2 = -0.2881681;
float X1 = 81.6435101;
float X0 = -7322;