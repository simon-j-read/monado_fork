# DIY Virtual Reality Headset
3DoF tracked DIY PCVR headset using Monado fork for interfacing with OpenXR VR applications.

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

## How to Make
1. Source parts see Table ??
2. Print 3d files, see Table ??
3. Assemble HMD
3. Load Arduino software (RawHID.ino)
   4. Configure HID permissions on the local computer 
   5. echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="YOUR_VENDOR_ID", ATTRS{idProduct}=="YOUR_PRODUCT_ID", MODE="0666"' | sudo tee /etc/udev/rules.d/99-diy-vr.rules
   6. sudo udevadm control --reload-rules
      sudo udevadm trigger
   7. Can unplug device, then replug device, then run
   8. ls -l /dev/hidraw*
   9. check for crw-rw-rw for read write permissions (can't run monado as sudo)
4. Configure the Monado
   5. VID (target_lists.c), PID, X11 (configuration script), USB permissions
5. Build this project's Monado

## To Do
- [ ] Get Wayland working
- [ ] Remove permant addressing for file access (config file)
- [ ] Arduino + IMU code and integration into Monado driver

## Improvements for Version 2
- Consolidate connections to computer down to one (likely usb-c)
- Remove dependency of Gear VR donor shell, and do a fully 3D printed design with lenses.
- 
- 