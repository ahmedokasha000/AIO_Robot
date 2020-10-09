#This will install ROS melodic desktop version on the Jetson Nano
./installROS/installROS.sh -p ros-melodic-desktop-full

#Set up catkin workspace 
./installROS/setupCatkinWorkspace.sh
#automatically source the workspace when new shell is launched
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


###################### Install hector_SLAM package #########################
sudo apt-get install ros-melodic-hector-slam
sudo apt-get install ros-melodic-teleop-twist*
sudo apt-get install ros-melodic-move-base*
#--------------------------------------------------------------64
sudo pip install Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -a -G gpio nvidia
#sudo cp lib/python/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
pip3 install rospkg
sudo apt-get install ros-melodic-rplidar-ros