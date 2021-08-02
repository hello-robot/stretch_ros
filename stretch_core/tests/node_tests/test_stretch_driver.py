import pytest
import rospy 
import time 
import actionlib
import math

from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu, MagneticField
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import SetBool, SetBoolRequest
from control_msgs.msg import FollowJointTrajectoryAction ,FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

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

@pytest.fixture 
def action_server_client():
    class ActionServerClient: 

        def __init__(self):
            self.trajectory_client = None
            self.joint_state = None
            
        def joint_state_callback(self,joint_state):
            self.joint_state = joint_state 

        def start_trajectory_client(self):
            self.trajectory_client = actionlib.SimpleActionClient("/stretch_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
            server_reached = self.trajectory_client.wait_for_server(timeout=rospy.Duration(60.0))
            return server_reached

        def send_trajectory(self,command):
            for delta in command['deltas']: 
                point = JointTrajectoryPoint()
                point.time_from_start = rospy.Duration(0.0)
                trajectory_goal = FollowJointTrajectoryGoal()
                trajectory_goal.goal_time_tolerance = rospy.Time(3.0)
                joint_name = command['joint']
                trajectory_goal.trajectory.joint_names = [joint_name]            
                joint_index = self.joint_state.name.index(joint_name)
                joint_value = self.joint_state.position[joint_index]
                delta_val = command['deltas'][delta]
                rospy.loginfo('delta = {0}, joint_index = {1}, joint_value = {2}'.format(delta, joint_index, joint_value))        
                new_value = joint_value + delta_val
                point.positions = [new_value]
                trajectory_goal.trajectory.points = [point]
                trajectory_goal.trajectory.header.stamp = rospy.Time.now()
                rospy.loginfo('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
                rospy.loginfo('Done sending pose.')
                time.sleep(3)
                self.trajectory_client.send_goal(trajectory_goal)

    return ActionServerClient()



     

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

def test_switch_to_position_mode(node): 
    rospy.wait_for_service('switch_to_position_mode') 

    s = rospy.ServiceProxy('switch_to_position_mode', Trigger)

    s_request = TriggerRequest()

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

def test_move_lift(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)  
    translate = 0.05
    deltas = {'down' : -translate , 'up' : translate}
    command = {'joint' : 'joint_lift' , 'deltas' : deltas} 

    if not action_server_client.start_trajectory_client():
        assert False 
    else: 
        action_server_client.send_trajectory(command)
        assert True

def test_move_wrist(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)  
    translate = 0.05
    deltas = {'in' : -translate , 'out' : translate}
    command = {'joint' : 'wrist_extension' , 'deltas' : deltas} 

    if not action_server_client.start_trajectory_client():
        assert False 
    else: 
        action_server_client.send_trajectory(command)
        assert True
        


def test_wrist_yaw(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)  
    deg = 10
    deltas = {'ccw' : -((math.pi/180) * deg) , 'cw' : ((math.pi/180) * deg)}
    command = {'joint' : 'joint_wrist_yaw' , 'deltas' : deltas} 

    if not action_server_client.start_trajectory_client():
        assert False 
    else: 
        action_server_client.send_trajectory(command)
        assert True

def test_head_tilt(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)  
    deg = 10
    deltas = {'ccw' : -((math.pi/180) * deg) , 'cw' : ((math.pi/180) * deg)}
    command = {'joint' : 'joint_head_tilt' , 'deltas' : deltas} 

    if not action_server_client.start_trajectory_client():
        assert False 
    else: 
        action_server_client.send_trajectory(command)
        assert True


def test_head_pan(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)  
    deg = 10
    deltas = {'ccw' : -((math.pi/180) * deg) , 'cw' : ((math.pi/180) * deg)}
    command = {'joint' : 'joint_head_pan' , 'deltas' : deltas} 

    if not action_server_client.start_trajectory_client():
        assert False 
    else: 
        action_server_client.send_trajectory(command)
        assert True