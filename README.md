# Solar_Tracker_Positional

This is an Arduino program which uses a linear actuator with a reed switch to make a single axis solar tracker move with no visual reference to the sun.  Since the sun is in the same spot east/west every day at the same time, it's easy to track based on time of day.

## End user guide

If anyone else wants to use this, let me know with an issue and I'll try to make the documentation more generic.  For now, the documentation is meant as a reference for me or my dad to re-configure the device if power is lost or the clock's battery dies.  It's at [User notes](https://jpangburn.github.io/solartracker/)

## How to update

If a code change becomes necessary, follow these steps:

1. Download this repository
2. Edit code in Arduino IDE and compile to make sure the code is ok
3. Move tracker to east limit
4. Plug USB into laptop, select port and board (MEGA 2560 currently), and click upload
5. Disconnect USB
6. Follow setup instructions in the [User notes](https://jpangburn.github.io/solartracker/) under the "Setup After Reset" section
7. Upload modified code to GitHub so it's not lost
