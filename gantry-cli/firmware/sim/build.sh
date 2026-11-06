#!/bin/sh
# Host build of firmware/gantry_fw/gantry_fw.ino against stub Arduino headers and a plant model.
# proto.py inserts prototypes above the first function, as the Arduino IDE does, so a sketch that
# compiles here also survives the IDE's preprocessing.
#   ./build.sh && ./fwsim unit        scripted checks of every command
#   python3 ../../test_e2e.py         the Gantry CLI against this simulator over a pseudo-terminal
set -e
cd "$(dirname "$0")"
python3 proto.py ../gantry_fw/gantry_fw.ino pp.ino
g++ -std=c++17 -O2 -Wall -Wno-unused-function -I. fwsim.cpp -o fwsim -lutil
echo "built ./fwsim"
