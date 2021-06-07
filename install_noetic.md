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

### Check that the robot's Noetic ROS installation is working
1. Shutdown and power off your robot. Then turn your robot back on. Once you're logged in, you can test your robot with the following commands.
2. Go ahead and attempt to home your robot using `stretch_robot_home.py`. It should be calibrated afterward.
3. Run `stretch_robot_system_check.py` to make sure that things are normal. Ideally, you will see all green and no red. 
4. Make sure the game controller dongle is plugged in and run `stretch_xbox_controller_teleop.py`. Use the game controller to test out the motions of the robot.
5. Run `roslaunch stretch_core wheel_odometry_test.launch`. You should see coherent visualizations of the robot's body, the laser range finder output, and the D435i point cloud output. You should be able to move the robot around using keyboard commands and see reasonable visualizations in RViz. 
6. You may delete the `./stretch-re1-xxxx` directory and its contents that you copied over from the Ubuntu 18.04 partition, if you'd like.

### Recalibrate your robot
1. The new Noetic ROS installation starts out by using the calibrated URDF that was created at the Hello Robot factory. 
2. We recommend that you recalibrate your robot by following the [stretch_calibration instructions](https://github.com/hello-robot/stretch_ros/tree/dev/noetic/stretch_calibration). This takes about 1.5 hours of robot time, but will result in a higher-quality model that matches the current state of the robot. For example, shipping can sometimes shift components a little. 

### Troubleshooting

##### Firmware Mismatch Error

If you are seeing the following error:
```
----------------
Firmware protocol mismatch on hello-.
Protocol on board is pX.
Valid protocol is: pX.
Disabling device.
Please upgrade the firmware and/or version of Stretch Body.
----------------
```
Your version of Stretch Body does not align with the firmware installed with your robot. Run the firmware updater tool to automatically update the firmware to the required version for your software.
```
$ python -m pip install hello-robot-stretch-factory
$ RE1_firmware_updater.py
```
