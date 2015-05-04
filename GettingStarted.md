# Compiling the library #
Dependencies:
  * Boot > 1.40 (for asio serial), headers, boost-thread-mt, boost-system-mt
Build System:
  * CMake > 2.60
Build Steps:
  * svn checkout http://ardupilotmegacomm.googlecode.com/svn/trunk/ ardupilotmegacomm-read-only
  * go to the ardupilotmegacomm-read-only directory
  * create a build directory and launch cmake from it, then make:
  * mkdir build; cd build; cmake ..;make
  * try out the test program

# Hardware in the Loop Walkthrough #

Install latest ardupilot-mega release from svn onto ardupilotmega board:
  * svn checkout http://ardupilot-mega.googlecode.com/svn/Sketchbook/trunk/ ardupilot-mega-read-only
  * Copy APM\_Config\_xplane.h to APM\_Config.h
  * Use arduino ide to compile and upload to the board

Execute the test program:

  * ./test/hilcomm /dev/ttyUSB0 38400

# ArduPilotMega Binary Protocol Walkthrough #

Install latest ardupilot-mega release from svn onto ardupilotmega board:
  * svn checkout http://ardupilot-mega.googlecode.com/svn/Sketchbook/trunk/ ardupilot-mega-read-only
  * Copy APM\_Config\_xplane.h to APM\_Config.h
  * Change GCS\_PROTOCOL to GCS\_PROTOCOL\_STANDARD
  * Change ENABLE\_HIL ENABLED to ENABLED\_HIL DISABLED
  * Optionally change other settings based on your setup
  * Use arduino ide to compile and upload to the board

Execute the test program:

  * ./test/bincomm /dev/ttyUSB0 38400

# Using the library for your project #
Install Option 1: Build from source
  * In the build directory build the code and then sudo make install.
Install Option 2: Use a package
  * Linux: download .deb file, dpkg -i ardupilotmegacomm-0.0.1.deb
  * Windows: don't have a packaged yet
How to build against it:
  * Manual
    * LDFLAGS : -lardupilotmegacomm
    * CPPFLAGS : -I/usr/include/ardupilotmegacomm
  * pkg-config
    * ardupilotmegacomm.pc included in install

# Packaging #
Linux:
  * cpack -G DEB

Windows:
  * cpack -G NSIS (If someone on windows can make a package for 0.0.1 it would be much appreciated.)