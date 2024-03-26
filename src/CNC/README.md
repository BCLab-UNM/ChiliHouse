# G-code Maker
Made by Andrei Popa-Simil  
NASA MINDS Spring 2024  

## Libraries
- pyserial
- virtualenv

## Virtual Environment (venv)
python virtual environment in .venv directory  

### Creation of Virtual Environment
pipx install virtualenv  
virtualenv .venv  



### Generic Activation/Deactivation
source .venv/bin/activate  
which python3  
deactivate  

### Scripting Activation/Deactivation
Activate using ./venvToggle.sh !! doesn't work rn  
Activate/Deactivate using local variables $venvA and $venvD  



# Leapfrog Usage
## Printer Tasks
- Hold Sensors
  - Camera
    - 2 IR cameras for depth and 1 visible for image
  - Temperature
    - Themister
  - Photosensor
- Hold Tools
  - Q-tip
- Read sensor data

## Code Tasks
-  Sends G-code and M-code to printer
- Controls/Interrupts Printer
- Change/Choose path based on z-data
### Control Features
- Code can go to intermediates points
- Raster Scan with dive points
- Centriod Detection


## Accessibility
- Computer can connect via tty
- Connect to computer via network (maybe)
- Computer is Connected to sensor and reads sensor data realtime.

## Goals
- Autonomously Control CNC head to monitor plants, via sensors
- Create a remotely controlled autonomous environment that:
  - monitors plants with various sensors
  - chooses different fertilization/watering schemes to keep plants health based on monitored data.
  - Easy and Safe to interact
  - Maximizes efficiency, growth, water preservation, or other metrics.
- Can change path based on z-images
- Easily Interruptible and modifiable by end user.