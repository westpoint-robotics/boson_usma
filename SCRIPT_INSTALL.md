# Install instructions for usma_bhg using the install script

## 1. Install script usage:
1. Install Ubuntu 18.04 on a computer  
2. Download the install script located in the bhg repo:  
`wget https://raw.githubusercontent.com/westpoint-robotics/usma_bhg/master/resources/bhg_install.sh bhg_install.sh`  
3. Make it executable:  
`chmod +x bhg_install.sh`  
4. Run the script and be patient; this may take a while:  
`./bhg_install.sh`  
5. If the last line says, "!! Successfully Ran to completion !!", it is likely it installed successfully.  

## 2. Update BIOS settings in the NUC:
1. Press 'F2' Button on boot to enter bios
2. Goto the 'Power' tab and select 'Secondary Power Settings', scroll down to 'After Power Failure' change to 'Power On'
3. Goto the 'Boot' tab and select 'Secure Boot', Change 'Secure Boot' to 'Disabled'
4. Goto the 'Cooling' tab and change 'Fan Control Mode' to 'Cool'
5. Press F10 button and then OK to save the settings and reboot.    


## 3. Enable auto login:   
1. Go to System | Users menu.   
2. Click Users to open the panel.  
3. Select the user account that you want to log in to automatically at startup.  
4. Press Unlock in the top right corner and type in your password when prompted.  
5. Switch the Automatic Login switch to on.  

## 4. Create a network connection for the GOBI camera
1. Go to Settings > Network > Wired, and click (+) button to add Gobi camera  
2. In `Identity` tab, put the camera name (ex. `gobi`)  
3. In `IPv4` tab, select `Manual` and enter the following information  
4. Address: `169.254.107.22`  
5. Netmask: `255.255.0.0`  

## 5. Create a hotspot access point
- Open the network manager connection editor:  
`nm-connection-editor`  
- Choose the plus sign to add a connections  
- Choose Wi-Fi for "Connection Type" and click "Create..."  
- On the Wi-Fi tab:
    - Set "Connection Name:" to NUC42Hotspot
    - Set "Mode:" to Hotspot
    - Set "SSID:" to NUC42
- On the Wi-Fi Security Tab
    - Set "Security:" to WPA & WPA2 Personal
    - Set the password: nuc42access
- On the General Tab
    - Make sure 'Automatically connect ...' is checked

## 6. Install Spinnaker SDK and dependencies:  
1. Download the SDK from https://flir.app.boxcn.net/v/SpinnakerSDK/folder/69083919457  
    - Extract the downloaded file
    - run the install script:
        `./install_spinnaker.sh`
2. During the install process, add a new member to `flirimaging`
<pre>
Would you like to add a udev entry to allow access to USB hardware? If a udev entry is not added, your cameras may only be accessible by running Spinnaker as sudo.  
[Y/n] $ <b>y</b>  
Adding new members to usergroup flirimaging… To add a new member please enter username (or hit Enter to continue):  
<b>Type your computer name (ex. $ user1)</b>  
Writing the udev rules file… Do you want to restart the udev daemon?  
[Y/n] $ <b>y</b>  
Would you like to set USB-FS memory size to 1000 MB at startup (via /etc/rc.local)?  
[Y/n] $ <b>y</b>  
Would you like to make a difference by participating in the Spinnaker feedback program?  
[Y/n] $ <b>n</b>  
</pre>
3. IMPORTANT: You will need to reboot the system for these changes to have full effect.
4. Install PySpin: Spinnaker Python packages, download from the same location as the SDK.
5. The file name is: spinnaker_python-2.0.0.146-cp27-cp27mu-linux_x86_64.tar.gz
6. Download the package (spinnaker_python-2.0.0.146-cp27-cp27mu-linux_x86_64.tar.gz) that corresponds to the python version (2.7) and Spinnaker version (spinnaker-2.0.0.146) from https://flir.app.boxcn.net/v/SpinnakerSDK/folder/74728781416
7. Uncompress the folders and move to the location in which the 'whl' file is located, and run  
`python -m pip install spinnaker_python-2.0.0.146-cp27-cp27mu-linux_x86_64.whl`  
8. The examples are located in the Examples folder of the extracted tarball. Run with:  
ex. `python Examples/Python3/DeviceEvents.py`  

## 7. Setup Arduino to work with the Trinket
1. Open the Arduino IDE and if prompted, choose to Trust it.
2. Enable ROS libraries:
	- cd ~/Arduino/libraries/
	- rm -rf ros_lib
	- rospack profile
	- rosrun rosserial_arduino make_libraries.py .
2. Add the Trinket to the Boards Manager:  
    - Go to: File | Preferences  
    - In the field labeled: 'Additional Boards Manager URLs:' paste the following:  
    `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`  
    - Exit out of Preferences by choosing the 'OK' button.  
3. Open the 'Boards manager'
    - Go to: Tools | Board .... | Boards Manager ...
    - Install 'Arduino SAMD Boards'
    - Install 'Adafruit SAMD'
    - Reference: https://learn.adafruit.com/adafruit-trinket-m0-circuitpython-arduino/arduino-ide-setup

## 8. Install chrome markdown viewer 
1. visit: https://chrome.google.com/webstore/detail/markdown-viewer/ckkdlimhmcjmikdlpkmbgfkaikojcbjk?hl=en  
2. Choose to add to Chrome.  
3. After Markdown Viewer is installed:  
    - Click on the "M" icon to the right of the address bar.  
    - Choose advanced options  
    - Choose Allow Access to File  
    - Turn on the option called "Allow access to file URLs"  
    
## 9. Extend the length of History
- In the ~/.bashrc file, change the below settings to lengthen the history file. Just add a couple of zero’s to each setting.
- HISTSIZE=100000
- HISTFILESIZE=200000

## 10. Modify Power Saving
1. Go to System Settings -> Power -> Blank screen 
2. Change to 'Never'
3. Set "Turn screen off when inactive for: 1 hour"
4. Set "Bluetooth" to off

## 11a. Edit Terminal's Default Profile (default terminal app)
1. Open Terminal. Click on Edit -> Profile Preferences -> Scrolling
2. On the Scrolling tab, uncheck the box "Limit scrollback to:"

## 11b. Edit Terminator's Default Profile (alternate terminal app)
1. Open Terminal. Right Click in the Terminal Window, and select Preferences.
2. Click on the Profiles tab.
3. On the Scrolling tab, check the box "Infinite Scrollback"

## 12. GEDIT Preferences.
1. Open a text file using Gedit or type `gedit` in a terminal window and hit enter. This brings up the text editor.
2. Click Edit -> Preferences -> Editor. 
3. Change Tab width to 4, Check the box for "Insert spaces instead of tabs"
4. Enable block commenting. Click Edit -> Preferences -> Plugins, and check the box for "Code Comment"
5. Enable highlight matching brackets. Click Edit -> Preferences -> View, and check the box for "Highlight matching brackets"

## 13. Setup git
- Use the following commands but substitute the computer name for nucXX in the top two commands.  
    - `git config --global user.email "user1@nucXX.com"`  
    - `git config --global user.name "User1 NucX"`  
    - `git config --global push.default simple`    
