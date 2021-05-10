#!/bin/bash
# BHG install script. Start from a clean install of Ubuntu 18.04.

# exit when any command fails
set -e
UPDATE_OS=true
INSTALL_CHROME=true
INSTALL_ROS=true
UPDATE_DIALOUT=true
INSTALL_GIT_GLOBAL=true
INSTALL_MAVROS=true
INSTALL_XENICS=true
INSTALL_NET_SETTINGS=true
INSTALL_SPINNAKER_DEPENDS=true
INSTALL_ARDUINOIDE=true

echo "Setting up OS for user: $USER"

if $UPDATE_OS
then
	echo "========= UPDATING UBUNTU =========="
	# Update the package list and os
	sudo apt-get update
	sudo apt-get -y upgrade
	sudo apt-get -y dist-upgrade
	sudo timedatectl set-timezone America/New_York
fi

if $INSTALL_CHROME
then
	# Install google chrome
	echo "========= INSTALLING CHROME =========="
	wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
	echo 'deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main' | sudo tee /etc/apt/sources.list.d/google-chrome.list
	sudo apt-get update
	sudo apt-get -y install google-chrome-stable
fi 

if $INSTALL_ROS
then
	echo "========= INSTALLING ROS =========="
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt-get update
	sudo apt-get -y install ros-melodic-desktop-full
	source /opt/ros/melodic/setup.bash

	sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential meld minicom ant git gitk openssh-server terminator gparted git-core python-argparse python-wstool python-vcstools build-essential gedit-plugins dkms python-rosdep gedit-plugins libcanberra-gtk-module libusb-0.1-4 ros-melodic-camera-info-manager ros-melodic-dynamic-reconfigure ros-melodic-libsensors-monitor
	sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
	sudo rosdep init
	rosdep update

	echo "========= CREATING AND CONFIGURING ROS WORKSPACE =========="
	cd
	mkdir -p catkin_ws/src
	source /opt/ros/melodic/setup.bash
	cd catkin_ws
	catkin_make
	LINE='source ~/catkin_ws/devel/setup.bash'
	FILE='~/.bashrc'
	grep -qxF -- "$LINE" "$FILE" || echo "$LINE" >> "$FILE"
	source ~/catkin_ws/devel/setup.bash
	rospack profile
fi

if $UPDATE_DIALOUT
then
	echo "========= Adding user to dialout =========="
	sudo adduser $USER dialout
fi


if $INSTALL_GIT_GLOBAL
then
	echo "========= Setting git global user and push simple =========="
	git config --global user.email "$USER@nucXX.com"
	git config --global user.name "$USER NucXX"
	git config --global push.default simple
fi


if $CLONE_BHG
then
	echo "========= CLONING USMA_BHG =========="
	cd ~/catkin_ws/src
	git clone https://github.com/westpoint-robotics/usma_bhg.git
	LINE='source ~/catkin_ws/src/usma_bhg/resources/.bhg_aliases.sh'
	FILE='~/.bashrc'
	grep -qxF -- "$LINE" "$FILE" || echo "$LINE" >> "$FILE"
	source ~/catkin_ws/src/usma_bhg/resources/.bhg_aliases.sh
fi

if $INSTALL_MAVROS
then
	echo "========= INSTALLING MAVROS =========="
	sudo apt-get -y install ros-melodic-mavros
	sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh
	echo "===== Installing Mavproxy dependencies ====="
	sudo apt-get -y autoremove
	sudo apt-get -y install python3-dev python3-opencv python3-wxgtk4.0 libxml2-dev python3-pip python3-matplotlib python3-lxml python-pip 
	pip3 install future
	pip3 install pymavlink
	pip3 install mavproxy
fi

if $INSTALL_XENICS
then
	# Xenics install 
	echo "========= INSTALLING XENETH SDK FOR GOBI CAMERA =========="
	cd ~/Downloads
	wget http://support.xenics.com/Support/Linux_SDK_27.zip
	rm -rf xenethSDK
	mkdir xenethSDK
	cd xenethSDK
	unzip ../Linux_SDK_27.zip
	sudo dpkg -i xeneth_2.7.0-181_amd64.deb
fi

if $INSTALL_NET_SETTINGS
then
	echo "===== Installing Wired connection to Gobi camera ====="
	cd 
	sudo cp ~/catkin_ws/src/usma_bhg/resources/Gobi /etc/NetworkManager/system-connections/Gobi
	sudo chown root:root /etc/NetworkManager/system-connections/Gobi
fi

if $INSTALL_SPINNAKER_DEPENDS
then
	echo "===== Installing Spinnaker Dependencies ====="
	sudo apt-get -y install libavcodec57 libavformat57 libswscale4 libswresample2 libavutil55 libusb-1.0-0 libgtkmm-2.4-dev
	python -m pip install --upgrade numpy matplotlib  
	python2 -m pip install enum34  
fi 

if $INSTALL_ARDUINOIDE
then
	## 12. Install Arduino IDE
	echo "========= INSTALLING ARDUINO IDE =========="
	cd ~/Downloads  
	wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
	tar -xf arduino-1.8.13-linux64.tar.xz  
	cd arduino-1.8.13/  
	sudo ./install.sh  
	sudo apt-get -y install ros-melodic-rosserial-arduino ros-melodic-rosserial
	chmod +x ~/Desktop/arduino-arduinoide.desktop 
	gio set ~/Desktop/arduino-arduinoide.desktop "metadata::trusted" yes
fi

# Last catkin make
cd ~/catkin_ws
catkin_make

echo '========================================================================='
echo '========================================================================='
echo '========================================================================='
echo 'Start on post BHG additions'
gsettings set org.gnome.shell favorite-apps "['google-chrome.desktop', 'org.gnome.Nautilus.desktop', 'terminator.desktop', 'org.gnome.gedit.desktop']"

echo "========= Using Snap to install Visual Code =========="
user1@nucay221:~/Data$ sudo snap install --classic code

echo '-'
echo 'To complete the install follow the instructions found at '
echo '-'
echo '========================================================================='
echo '========================================================================='
echo '========================================================================='
echo "!! Successfully Ran to completion !!"
