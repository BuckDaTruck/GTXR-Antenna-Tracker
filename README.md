# GTXR Antenna Tracker

Antenna tracker and ground station software built for the Georgia Tech Experimental Rocketry (GTXR) team. The project combines Python utilities, Arduino firmware, and CAD models for a two-axis tracking platform.

## Overview

The system reads GPS telemetry from a vehicle or Featherweight tracker and calculates the azimuth and elevation required to aim a directional antenna. A stepper-motor driven gimbal points the antenna while a Python application provides a simple user interface and communication with the Arduino controller.

This repository contains everything needed to build the tracker:

- **Python application** (`App/`) for reading GPS data and sending commands to the gimbal
- **Firmware** (`Tracker-Hardware/Firmware/`) for an Arduino-based controller
- **CAD files** (`cad/`) for printing or machining mechanical parts
- **Reference materials and test tools** (`resources/`, `test tools/`)
- **Hardware list** (`PARTS_LIST.md`)

## Repository Layout

```
App/                  Python scripts and GUI assets
Tracker-Hardware/     Arduino firmware
cad/                  STL, STEP, and DXF models for the tracker frame
Assets/               Logos and icons used by the app
FilteredData/         Example telemetry log
resources/            Reference projects and documentation
```

## Software Setup

1. Install Python 3.x
2. Install dependencies:
   ```bash
   pip install numpy pynmea2 pyserial wxPython
   ```
   (Depending on your system you may need additional packages.)
3. Connect the GPS receiver and Arduino-based tracker to your computer.

## Running the Tracker

The main Python application is `App/v2.py`. Launch it with:

```bash
python3 App/v2.py
```

The program listens to GPS data on a serial port and outputs azimuth and elevation commands to the microcontroller. Configure the serial port names in the script if they differ from your setup.

## Hardware and Firmware

The hardware design uses an Arduino with a CNC shield to drive two stepper motors. CAD models for the frame and gears live under `cad/`. Electronic components are listed in `PARTS_LIST.md`.

To load the firmware:

1. Open `Tracker-Hardware/Firmware/Firmware.ino` in the Arduino IDE.
2. Select your board and port.
3. Compile and upload.

## Data and Resources

Example telemetry logs are provided in `FilteredData` and `data.csv`. Additional research code and documentation can be found in the `resources` directory.

## Contact

Developed by Buckley Wiley for GTXR in the fall of 2023. For questions email [buckley@buckleywiley.com](mailto:buckley@buckleywiley.com).
