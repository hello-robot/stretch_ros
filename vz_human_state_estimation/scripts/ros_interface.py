#! /usr/bin/python3

# ROS specific imports 
import rospy
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Non ROS imports
import merged_detection_algorithm as mda
import cv2
from cv_bridge import CvBridge, CvBridgeError

UPDATE_RATE_HZ = 10 #hz for publishing bounding box later 

class ROSInterface:
    def __init__(self):
        # Initialize the two images as None
        self.cv_rgbd_img = None
        self.cv_thermal_img = None
        self.cv_bounding_boxed_img = None

        # Images for display and passing to human state estimation node
        # They will be rotated based on parameter /flip_images_human_state_est
        self.cv_rgbd_img_probably_rotated = None
        self.cv_thermal_img_probably_rotated = None

        # To flip or not to flip...
        self.flip_images_human_state_est = rospy.get_param('flip_images_human_state_est')

        self.MDA = mda.ConnectToROS()

        # set up CV Bridge
        self.bridge = CvBridge()

        #Image subscribers
        rgbd_sub    = message_filters.Subscriber("/D435i_raw",Image)
        thermal_sub = message_filters.Subscriber("/flir_3_5_near_realsense_raw",Image)

        # todo: confirm the slop parameter with Paul Ghanem. 2 secs seems high
        ats = message_filters.ApproximateTimeSynchronizer([rgbd_sub,thermal_sub],queue_size=10,slop=2.0)
        ats.registerCallback(self.ats_callback)
    
    # Once we receive rgbd and thermal images, which are also periodically received by this function
    # We call this ats_callback function. This is also where we call the processData function to interface
    # with other algorithms e.g. Human State Estimation and procure their outputs
    def ats_callback(self,rgbd_img_data,thermal_img_data):
        self.cv_rgbd_img = self.bridge.imgmsg_to_cv2(rgbd_img_data)
        self.cv_thermal_img = self.bridge.imgmsg_to_cv2(thermal_img_data)

        # Rotate if needed
        # Show images and pass onto human state estimation flipped if following true
        # This is done because the raw images captured from rgbd and thermal cameras are oriented incorrect

        if(self.flip_images_human_state_est == True):
            ##### NOTE: Change rotation based on what you see ########
            self.cv_rgbd_img_probably_rotated = cv2.rotate(self.cv_rgbd_img,cv2.ROTATE_90_CLOCKWISE)            
            self.cv_thermal_img_probably_rotated = cv2.rotate(self.cv_thermal_img,cv2.ROTATE_180)
        else:
            self.cv_rgbd_img_probably_rotated = self.cv_rgbd_img
            self.cv_thermal_img_probably_rotated = self.cv_thermal_img

        # Call the human state estimation code via processData interface
        # This is where we will iteratively call the algorithms to obtain outputs eg. bounding boxes
        self.cv_bounding_boxed_img = self.MDA.processData(self.cv_rgbd_img_probably_rotated,self.cv_thermal_img_probably_rotated)

    def run(self):
        while not rospy.is_shutdown():
            if(self.cv_rgbd_img_probably_rotated is None or self.cv_thermal_img_probably_rotated is None or self.cv_bounding_boxed_img is None):
                continue
            try:
                # Show images
                cv2.imshow('D435i',cv2.resize(self.cv_rgbd_img_probably_rotated,(600,800), interpolation = cv2.INTER_AREA))
                cv2.imshow('Thermal',cv2.resize(self.cv_thermal_img_probably_rotated,(600,800), interpolation = cv2.INTER_AREA))
                cv2.imshow('BoundingBox',cv2.resize(self.cv_bounding_boxed_img,(600,800), interpolation = cv2.INTER_AREA))
                cv2.waitKey(3)

            except Exception as e:
                rospy.logwarn(e)

if __name__ == '__main__':
    try:
        rospy.init_node('merged_detection_algorithm_ros', anonymous=True)
        interface = ROSInterface()
        interface.run()
    except rospy.ROSInterruptException:
        print('Human Pose Detection Node Failed')
        pass
    cv2.destroyAllWindows()


# The following is just a simple subscriber. I think we need a time synced subscriber
# class ROSInterface:
#     def __init__(self):
#         # Initialize the two images as None
#         self.cv_rgbd_img = None
#         self.cv_thermal_img = None

#         # set up CV Bridge
#         self.bridge = CvBridge()

#         #Image subscribers
#         rospy.Subscriber("/D435i_raw",Image,self.rgbd_callback)
#         rospy.Subscriber("/flir_3_5_near_realsense_raw",Image,self.thermal_callback)
    
#     def rgbd_callback(self,rgbd_img_data):
#         self.cv_rgbd_img = self.bridge.imgmsg_to_cv2(rgbd_img_data)
#         print('rgbd callback')
        
#     def thermal_callback(self,thermal_img_data):
#         self.cv_thermal_img = self.bridge.imgmsg_to_cv2(thermal_img_data)
#         print('thermal callback')
    
#     def run(self):
#         while not rospy.is_shutdown():
#             if(self.cv_rgbd_img is None or self.cv_thermal_img is None):
#                 continue
#             try:
#                 # Show images
#                 cv2.imshow('D435i',cv2.resize(self.cv_rgbd_img,(600,800), interpolation = cv2.INTER_AREA))
#                 cv2.imshow('Thermal',cv2.resize(self.cv_thermal_img,(600,800), interpolation = cv2.INTER_AREA))
#                 cv2.waitKey(3)

#             except Exception as e:
#                 rospy.logwarn(e)
