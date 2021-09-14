![](../images/banner.png)

## Overview

*stretch_demos* provides simple demonstrations for the Stretch RE1 mobile manipulator from Hello Robot Inc. For an overview of the demos, we recommend you look at the following forum post: https://forum.hello-robot.com/t/autonomy-video-details

## Getting Started Demos

**Please be aware that these demonstrations typically do not perform careful collision avoidance. Instead, they expect to operate in freespace and detect contact through motor current if an obstacle gets in the way. Please be careful when trying out these demonstrations.**

### Handover Object Demo

First, place the robot near you so that it can freely move back and forth and reach near your body. Then, launch the handover object demo using the following command: 

```
roslaunch stretch_demos handover_object.launch
```

For this demonstration, the robot will pan its head back and forth looking for a face. It will remember the 3D location of the mouth of the nearest face that it has detected. If you press "y" or "Y" on the keyboard in the terminal, the robot will move the grasp region of its gripper toward a handover location below and away from the mouth. 

The robot will restrict itself to Cartesian motion to do this. Specifically, it will move its mobile base backward and forward, its lift up and down, and its arm in and out. If you press "y" or "Y" again, it will retract its arm and then move to the most recent mouth location it has detected. 

At any time, you can also use the keyboard teleoperation commands in the terminal window. With this, you can adjust the gripper, including pointing it straight out and making it grasp an object to be handed over.

### Grasp Object Demo

For this demonstration, the robot will look for the nearest elevated surface, look for an object on it, and then attempt to grasp the largest object using Cartesian motions. Prior to running the demo, you should move the robot so that its workspace will be able to move its gripper over the surface while performing Cartesian motions. 

Once the robot is in position, retract and lower the arm so that the robot can clearly see the surface when looking out in the direction of its arm. 

Now that the robot is ready, launch the demo with the following command:

```
roslaunch stretch_demos grasp_object.launch
```

Then, press the key with ‘ and “ on it while in the terminal to initiate a grasp attempt.

While attempting the grasp the demo will save several images under the ./stretch_user/debug/ directory within various grasping related directories. You can view these images to see some of what the robot did to make its decisions.

### Clean Surface Demo

For this demonstration, the robot will look for the nearest elevated surface, look for clear space on it, and then attempt to wipe the clear space using Cartesian motions. Prior to running the demo, you should move the robot so that its workspace will be able to move its gripper over the surface while performing Cartesian motions. 

**You should also place a soft cloth in the robot's gripper.**

Once the robot is in position with a cloth in its gripper, retract and lower the arm so that the robot can clearly see the surface when looking out in the direction of its arm. 

Now that the robot is ready, launch the demo with the following command:

```
roslaunch stretch_demos clean_surface.launch
```

Then, press the key with the / and ? on it while in the terminal to initiate a surface cleaning attempt.

## License

For license information, please see the LICENSE files. 
