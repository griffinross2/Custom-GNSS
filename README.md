# Custom-GNSS
My work to create a custom FPGA GNSS receiver. This repository is to contain code for complete simulation and design of a multi-constellation GNSS receiver. The receiver will be built in hardware using a front-end digitizer module, an FPGA for demodulation and tracking with a soft CPU implemented on-chip, and a microcontroller for high-level computation and solving.

## Simulation
Directory with code used to simulate and test the receiver in software.

### TrackerSim
C++ Simulation of GNSS Recevier. To use, open in vscode and use the CMake file to build and run. A binary file with 1-bit I samples like [gnss-20170427-L1.1bit.I.bin](https://drive.google.com/file/d/158aSbdcyE3B8lAzl-4mJcwwZusJo11b2/view?usp=sharing) is required.

## Hardware
Directory with hardware design files.

### GNSS-board-v0
Initial GNSS receiver board design in KiCAD 9.

## Logs
General place for storing logs of simulation runs and other relevant test data.
