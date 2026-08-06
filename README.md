# DIY Virtual Reality Headset
DIY PCVR headset using Monado fork for interfacing with OpenXR VR applications.

## Components
- Wisecoco 2.9 Inch 1440x1440 VR LCD Display VR Dual Screen Display MIPI 90Hz Driver Board
- Samsung gear VR HMD
- Arduino pro micro
- Adafruit TDK InvenSense ICM-20948
- 3D printed components

## Software
- Arduino code to read values off of adafruit IMU and pass values over usb to Monado runtime
- Monado runtime with custom driver for DIY headset (forked Monado repo)

## Usage

Developed on:
- OS: CachyOS
- GPU: Discrete AMD RX570
- Graphics Platform: X11 (not Wayland yet) 


## To Do
- [ ] Get Wayland working
- [ ] Remove permant addressing for file access (config file)
- [ ] Arduino + IMU code and integration into Monado driver
