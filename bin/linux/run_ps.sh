#!/bin/bash

# Run the two executables with the PTYs as arguments
./serial_responder /dev/pts/2 &
./serial_sender /dev/pts/3


