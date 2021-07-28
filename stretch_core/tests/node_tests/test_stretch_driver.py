import pytest
import rospy 
import time 
import actionlib

from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu, MagneticField
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import SetBool, SetBoolRequest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
######## DEFINE TEST FIXTURES #######

@pytest.fixture
def node():
    rospy.init_node('test_stretch_driver', anonymous=True)

@pytest.fixture
def waiter():
    class Waiter:
        def __init__(self):
            self.received = []
            self.condition = lambda x: False

        @property
        def success(self):
            return True in self.received

        def callback(self, data):
            self.received.append(self.condition(data))

        def wait(self, timeout):
            timeout_t = time.time() + timeout
            while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                time.sleep(0.1)

        def reset(self):
            self.received = []

    return Waiter()


######## TEST PUBLISHERS #######

def test_joint_states_receives_something(node, waiter):
    waiter.condition = lambda data: True  # any message is good
    
    rospy.Subscriber("/stretch/joint_states", JointState, waiter.callback) 
    waiter.wait(10.0)

    assert waiter.success 

def test_odom_receives_something(node, waiter):
    waiter.condition = lambda data: True  # any message is good
    
    rospy.Subscriber("/odom", Odometry, waiter.callback) 
    waiter.wait(10.0)

    assert waiter.success 


def test_imu_mobile_base_receives_something(node, waiter):
    waiter.condition = lambda data: True  # any message is good
    
    rospy.Subscriber("/imu_mobile_base", Imu, waiter.callback) 
    waiter.wait(10.0)

    assert waiter.success 


def test_imu_wrist_receives_something(node, waiter):
    waiter.condition = lambda data: True  # any message is good
    
    rospy.Subscriber("/imu_wrist", Imu, waiter.callback) 
    waiter.wait(10.0)

    assert waiter.success 


def test_magnetometer_mobile_base_receives_something(node, waiter):
    waiter.condition = lambda data: True  # any message is good
    
    rospy.Subscriber("/magnetometer_mobile_base", MagneticField, waiter.callback) 
    waiter.wait(10.0)

    assert waiter.success 



######## TEST SERVICES #######

def test_switch_to_navigation_mode(node): 
    rospy.wait_for_service('switch_to_navigation_mode') 

    s = rospy.ServiceProxy('switch_to_navigation_mode', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)

    assert result.success == True

def test_switch_to_position_mode(node): 
    rospy.wait_for_service('switch_to_position_mode') 

    s = rospy.ServiceProxy('switch_to_position_mode', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)

    assert result.success == True

def test_stop_the_robot(node): 
    rospy.wait_for_service('stop_the_robot') 

    s = rospy.ServiceProxy('stop_the_robot', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)

    assert result.success == True

def test_runstop(node): 
    rospy.wait_for_service('runstop') 

    s = rospy.ServiceProxy('runstop', SetBool)

    s_request = SetBoolRequest()

    result = s(s_request)

    assert result.success == True


''' Test timing out (60s), error could be due to calibration

def test_switch_to_manipulation_mode(node): 
    rospy.wait_for_service('switch_to_manipulation_mode') 

    s = rospy.ServiceProxy('switch_to_manipulation_mode', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)

    assert result.success == True
'''

######## TEST ACTION SERVER #######
