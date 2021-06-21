# Timing for Boson and Blackfly
- The timing of the cameras is acheived by sending a separate signal to each camera. The Blackfly processes images much faster then the boson. 
- The Orange wire on pin 3 is for the LWIR 
- The Red wire on pin 4 is the delayed signal to the Blackfly RGB.
- The Brown wire is the shared ground wire 
-  5hz and 10hz Using this system the average time between images is less than 1ms. The standard deviation is 7ms.
- 15hz and 20hz Using this system the average time between images is less than 1ms. The standard deviation is 4ms.

# Adding ability to read the serial input from the arduino trinket
- Added a udev rule for the trinket (directory is /etc/udev/rules.d):  
`SUBSYSTEM=="tty", ATTRS{idVendor}=="239a", ATTRS{idProduct}=="801e", GROUP="dialout", SYMLINK="trinket"`  
- Installed the cpp library for serial communication by cloning the repo and following the repo readme.
`git clone https://github.com/gbmhunter/CppLinuxSerial.git`
- 
