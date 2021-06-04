![](./images/banner.png)

# Noetic Installation Instructions

### Install a partition with Ubuntu 20.04 Desktop Edition
1. Download and write the [Ubuntu 20.04 iso file](https://releases.ubuntu.com/20.04/ubuntu-20.04.2.0-desktop-amd64.iso) to a USB key.
2. Backup all of the critical files from the existing Ubuntu 18.04 partition on the robot.
3. Make sure you have sufficient space on the robot’s solid state drive (SSD) for a new partition with Ubuntu 20.04. 
    1. The Stretch RE1 typically ships with a 500 GB SSD drive. The default ROS Melodic installation typically takes up less than 20 GB when shipped. However, the robot may be using much more space due to use. 
    2. The default ROS Noetic installation typically takes up less than 20 GB, but you will want additional space while using the robot.
4. Boot the robot with the USB key plugged into a USB port in the robot’s trunk.
5. The installer should detect the Ubuntu 18.04 installation and provide the option to install Ubuntu 20.04 alongside Ubuntu 18.04. This option will be selected by default. ![20.04 alongside 18.04](./images/ubuntu_installation_1.jpg)
6. Proceed with the option to install Ubuntu 20.04 alonside Ubuntu 18.04. This will result in a new partition with Ubuntu 20.04. You will have the opportunity to decide how much space to devote to each partition. 

### Copy materials from the robot's original Ubuntu 18.04 partition
1. Boot into the robot's original Ubuntu 18.04 partition.
2. Copy the `/etc/hello-robot` to a USB key. 
    1. For example, you can run `cp -r /etc/hello-robot /media/$USER/USBKEY` from the command line where USBKEY is the mounted USB key.
    2. Or, you can open the file explorer to copy the directory. Make sure you look for `/etc/hello-robot` on the original Ubuntu 18.04 partition rather than the new partition. 
3. Boot into the robot's new Ubuntu 20.04 partition.
4. Copy the folder of the form "stretch-re1-xxxx" found in the `hello-robot` directory into the home folder (i.e. `/home/$USER/`).
    1. For example, you can run a command similar to `cp -r /media/$USER/USBKEY/hello-robot/stretch-re1-xxxx /home/$USER/`
    2. Or, you can use the visual file explorer. 

### Run the setup script with the robot's new Ubuntu 20.04 installation
1. Go to the home directory of the new Ubuntu 20.04 installation with `cd`. Then run the following commands.
2. `sudo apt install git`
3. `git clone https://github.com/hello-robot/stretch_install.git -b dev/install_20.04`
4. `cd ./stretch_install/factory && ./stretch_install_nonfactory.sh`
    - You will need to provide input when prompted, including your robot's number and your password. The password request can come late in the process and freeze installation until you provide it.

### Clean up and verify the robot's Noetic ROS installation
1. `cd ~ && rm -rf ./stretch-re1-xxxx`
2. `stretch_robot_home.py` and `stretch_robot_system_check.py`
