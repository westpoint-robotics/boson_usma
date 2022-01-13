# Timing for Boson and Blackfly
- The timing of the cameras is acheived by sending a separate signal to each camera. The Blackfly processes images much faster then the boson. 
- The Orange wire on pin 3 is for the LWIR 
- The Red wire on pin 4 is the delayed signal to the Blackfly RGB.
- The Brown wire is the shared ground wire 
-  5hz and 10hz Using this system the average time between images is less than 1ms. The standard deviation is 7ms.
- 15hz and 20hz Using this system the average time between images is less than 1ms. The standard deviation is 4ms.

# Adding ability to read the serial input from the arduino trinket
- To avoid conflict with other devices using the same subsystems you can use the below UDEV rules.
    - `echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="239a", ATTRS{idProduct}=="801e", GROUP="dialout", SYMLINK="trinket"' | sudo tee -a /etc/udev/rules.d/90-trinket.rules > /dev/null`
- Then reload the udev rules
    - `sudo udevadm control --reload-rules`
