// these settings are specific to a given solar tracker installation

// what is the full west position
const int FULL_WEST_POSITION = 2300;
// what is the minimum amount of ticks to move?  (don't move if going to move less than this)
const unsigned int MINIMUM_ACTUATOR_MOVEMENT = 10;
// what is the lat/long of this tracker
const float LATITUDE = 33.11;
const float LONGITUDE = -116.98;
// polynomial regression constants for converting desired azimuth to actuator position
float X3 = -0.0010593807;
float X2 = 0.6189061527;
float X1 = -96.9547832692;
float X0 = 4516.4461316026;