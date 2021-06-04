![](./images/banner.png)

## Noetic Installation Instructions

1. Install a partition with Ubuntu 20.04 Desktop Edition
    1. Download and write the [Ubuntu 20.04 iso file](https://releases.ubuntu.com/20.04/ubuntu-20.04.2.0-desktop-amd64.iso) to a USB key.
    2. Backup all of your critical files from the existin Ubuntu 18.04 partition on your robot.
    3. Make sure you have sufficient space on your robot’s solid state drive (SSD) for a new partition with Ubuntu 20.04. 
        1. The Stretch RE1 typically ships with a 500 GB SSD drive. The default ROS Melodic installation typically takes up less than 20 GB when shipped. However, your robot may be using much more space due to your use. 
        2. The default ROS Noetic installation typically takes up less than 20 GB, but you will want additional space while using your robot.
    5. Boot the robot with the USB key plugged into a USB port in the robot’s trunk.
    6. The installer should detect your Ubuntu 18.04 installation and provide the option to install Ubuntu 20.04 alongside Ubuntu 18.04. This option will be selected by default. ![20.04 alongside 18.04](./images/ubuntu_installation_1.jpg)
    7. Proceed with the option to install Ubuntu 20.04 alonside Ubuntu 18.04. This will result in a new partition with Ubuntu 20.04. You will have the opportunity to decide how much space to devote to each partition. 
2. Copy materials from your Ubuntu 18.04 partition
   1. Boot into your Ubuntu 18.04 partition.
   2. Open a file explorer to `/etc/hello-robot` and copy the folder called "stretch-re1-xxxx" onto a USB key.
   3. Boot into your Ubuntu 20.04 partition.
   4. Copy the "stretch-re1-xxxx" folder into your home folder (i.e. `/home/$USER`).
3. Run the robot setup script.
   1. `sudo apt install git`
   2. `git clone https://github.com/hello-robot/stretch_install.git -b dev/install_20.04`
   3. `cd ./stretch_install/factory && ./stretch_install_nonfactory.sh`
     - Supply your password if asked for it
4. Clean up and Verify
   1. `cd ~ && rm -rf ./stretch-re1-xxxx`
   2. `stretch_robot_home.py` and `stretch_robot_system_check.py`
