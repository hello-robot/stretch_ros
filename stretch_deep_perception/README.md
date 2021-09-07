![](../images/banner.png)

## Overview

*stretch_deep_perception* provides demonstration code that uses open deep learning models to perceive the world. 

This code depends on the stretch_deep_perception_models repository, which should be installed under ~/stretch_user/ on your Stretch RE1 robot.

Link to the stretch_deep_perception_models repository:
https://github.com/hello-robot/stretch_deep_perception_models

## Getting Started Demos

There are four demonstrations for you to try.

### Face Estimation Demo

First, try running the face detection demonstration via the following command:

```
roslaunch stretch_deep_perception stretch_detect_faces.launch 
```

RViz should show you the robot, the point cloud from the camera, and information about detected faces. If it detects a face, it should show a 3D planar model of the face and 3D facial landmarks. These deep learning models come from OpenCV and the Open Model Zoo (https://github.com/opencv/open_model_zoo).

You can use the keyboard_teleop commands within the terminal that you ran roslaunch in order to move the robot's head around to see your face.


```
             i (tilt up)
	     
j (pan left)               l (pan right)

             , (tilt down)
```

Pan left and pan right are in terms of the robot's left and the robot's right.

Now shut down everything that was launched by pressing q and Ctrl-C in the terminal.

### Object Detection Demo

Second, try running the object detection demo, which uses the tiny YOLO v3 object detection network (https://pjreddie.com/darknet/yolo/). RViz will display planar detection regions. Detection class labels will be printed to the terminal. 

```
roslaunch stretch_deep_perception stretch_detect_objects.launch
```

Once you're ready for the next demo, shut down everything that was launched by pressing q and Ctrl-C in the terminal.

### Body Landmark Detection Demo

Third, try running the body landmark point detection demo. The deep learning model comes from the Open Model Zoo (https://github.com/opencv/open_model_zoo). RViz will display colored 3D points on body landmarks. The network also provides information to connect these landmarks, but this demo code does not currently use it.


```
roslaunch stretch_deep_perception stretch_detect_body_landmarks.launch 
```

Once you're ready for the next demo, shut down everything that was launched by pressing q and Ctrl-C in the terminal.

### Nearest Mouth Detection Demo

Finally, try running the nearest mouth detection demo. RViz will display a 3D frame of reference estimated for the nearest mouth detected by the robot. Sometimes the point cloud will make it difficult to see. Disabling the point cloud view in RViz will make it more visible.

We have used this frame of reference to deliver food near a person's mouth. This has the potential to be useful for assistive feeding. However, use of this detector in this way could be risky. Please be very careful and aware that you are using it at your own risk.

A less risky use of this detection is for object delivery. stretch_demos has a demonstration that delivers an object based on this frame of reference by holding out the object some distance from the mouth location and below the mouth location with respect to the world frame. This works well and is inspired by similar methods used with the robot EL-E at Georgia Tech [1]. 


```
roslaunch stretch_deep_perception stretch_detect_nearest_mouth.launch 
```

## References

[1] Hand It Over or Set It Down: A User Study of Object Delivery with an Assistive Mobile Manipulator, Young Sang Choi, Tiffany L. Chen, Advait Jain, Cressel Anderson, Jonathan D. Glass, and Charles C. Kemp, IEEE International Symposium on Robot and Human Interactive Communication (RO-MAN), 2009. http://pwp.gatech.edu/hrl/wp-content/uploads/sites/231/2016/05/roman2009_delivery.pdf


## License

For license information, please see the LICENSE files. 
