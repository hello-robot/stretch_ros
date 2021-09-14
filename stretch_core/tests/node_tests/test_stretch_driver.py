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
    rospy.init_node("test_stretch_driver", anonymous=True)

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
            self.initial_position = None
            self.final_position = None

            self.joint_initial_rotate = []
            self.joint_final_rotate = []

            self.joint_initial_translate = []
            self.joint_final_translate = []

        def joint_state_callback(self,joint_state):
            self.joint_state = joint_state

        def start_trajectory_client(self):
            self.trajectory_client = actionlib.SimpleActionClient("/stretch_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
            server_reached = self.trajectory_client.wait_for_server(timeout=rospy.Duration(60.0))
            return server_reached

        def move_joint(self,command):

            #Unpack joint name and store inital state in member variable
            joint_name = command['joint']
            joint_index = self.joint_state.name.index(joint_name)
            self.initial_position = self.joint_state.position[joint_index]

            #Loop through desired deltas in command dictionary and send goal
            for delta in command['deltas']:
                point = JointTrajectoryPoint()
                point.time_from_start = rospy.Duration(0.0)
                trajectory_goal = FollowJointTrajectoryGoal()
                trajectory_goal.goal_time_tolerance = rospy.Time(3.0)
                trajectory_goal.trajectory.joint_names = [joint_name]
                joint_value = self.joint_state.position[joint_index]
                delta_val = command['deltas'][delta]
                new_value = joint_value + delta_val
                point.positions = [new_value]
                trajectory_goal.trajectory.points = [point]

                rospy.loginfo('Done sending pose.')
                self.trajectory_client.send_goal(trajectory_goal)
                time.sleep(2)

            #Store final state in memeber variable
            self.final_position = self.joint_state.position[joint_index]


        def move_multiple_joints(self,joint_names,rotate,translate):

            #Unpack joint names and store inital states in member variable
            joint_names_rotate = [joint_name for joint_name in joint_names if joint_name != 'joint_lift' and joint_name != 'wrist_extension']
            joint_names_translate = [joint_name for joint_name in joint_names if joint_name == 'joint_lift' or joint_name == 'wrist_extension']
            joint_index_rotate = [self.joint_state.name.index(joint_name) for joint_name in joint_names_rotate]
            joint_index_translate = [self.joint_state.name.index(joint_name) for joint_name in joint_names_translate]
            self.joint_initial_rotate = [self.joint_state.position[joint_index] for joint_index in joint_index_rotate]
            self.joint_initial_translate = [self.joint_state.position[joint_index] for joint_index in joint_index_translate]

            #Declare points in trajectory
            point_move_forward = JointTrajectoryPoint()
            point_move_back = JointTrajectoryPoint()

            point_move_forward.time_from_start = rospy.Duration(0.0)
            point_move_back.time_from_start = rospy.Duration(5.0)

            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(10.0)

            trajectory_goal.trajectory.joint_names = joint_names_rotate + joint_names_translate

            #Set forward point to inital_values + desired_movements
            new_values_rotate_forward = [joint_values + rotate for joint_values in self.joint_initial_rotate]
            new_values_translate_forward = [joint_values + translate for joint_values in self.joint_initial_translate]
            new_value_forward = new_values_rotate_forward + new_values_translate_forward

            #Set back point to inital values
            new_values_rotate_back = [joint_values for joint_values in self.joint_initial_rotate]
            new_values_translate_back = [joint_values for joint_values in self.joint_initial_translate]
            new_value_back = new_values_rotate_back + new_values_translate_back

            point_move_forward.positions = new_value_forward
            point_move_back.positions = new_value_back

            #Add points to trajectory and send goal
            trajectory_goal.trajectory.points = [point_move_forward,point_move_back]
            self.trajectory_client.send_goal(trajectory_goal)
            time.sleep(3)

            #Store final states in member variable
            self.joint_final_rotate = [self.joint_state.position[joint_index] for joint_index in joint_index_rotate]
            self.joint_final_translate = [self.joint_state.position[joint_index] for joint_index in joint_index_translate]

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

######## TEST SUBSCRIBERS #######

def test_cmd_vel_subbed(node):
    cmd_pub = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size = None)
    time.sleep(0.5)
    assert cmd_pub.get_num_connections() == 1


######## TEST SERVICES #######

def test_switch_to_position_mode(node):
    rospy.wait_for_service('switch_to_position_mode')

    s = rospy.ServiceProxy('switch_to_position_mode', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)

    assert result.success

def test_switch_to_navigation_mode(node):
    rospy.wait_for_service('switch_to_navigation_mode')

    s = rospy.ServiceProxy('switch_to_navigation_mode', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)

    assert result.success


def test_stop_the_robot(node):
    rospy.wait_for_service('stop_the_robot')

    s = rospy.ServiceProxy('stop_the_robot', Trigger)

    s_request = TriggerRequest()

    result = s(s_request)

    assert result.success

def test_runstop(node):
    rospy.wait_for_service('runstop')

    s = rospy.ServiceProxy('runstop', SetBool)

    s_request = SetBoolRequest()

    result = s(s_request)

    assert result.success


######## TEST ACTION SERVER #######

def test_move_lift(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)
    translate = 0.1
    deltas = {'down' : -translate , 'up' : translate}
    command = {'joint' : 'joint_lift' , 'deltas' : deltas}

    if not action_server_client.start_trajectory_client():
        assert False
    else:
        action_server_client.move_joint(command)
        assert (action_server_client.initial_position) == pytest.approx(action_server_client.final_position, abs=1e-2)

def test_move_wrist(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)
    translate = 0.1
    deltas = { 'out' : translate, 'in' : -translate ,}
    command = {'joint' : 'wrist_extension' , 'deltas' : deltas}

    if not action_server_client.start_trajectory_client():
        assert False
    else:
        action_server_client.move_joint(command)

        assert action_server_client.initial_position == pytest.approx(action_server_client.final_position, abs=1e-2)

def test_move_wrist_yaw(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)
    deg = 15
    deltas = {'ccw' : ((math.pi/180) * deg) , 'cw' : -((math.pi/180) * deg)}
    command = {'joint' : 'joint_wrist_yaw' , 'deltas' : deltas}

    if not action_server_client.start_trajectory_client():
        assert False
    else:
        action_server_client.move_joint(command)

        assert action_server_client.initial_position == pytest.approx(action_server_client.final_position, abs=1e-1)

def test_move_gripper_finger(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)
    deg = 5
    deltas = {'ccw' : ((math.pi/180) * deg) , 'cw' : -((math.pi/180) * deg)}
    command = {'joint' : 'joint_gripper_finger_left' , 'deltas' : deltas}

    if not action_server_client.start_trajectory_client():
        assert False
    else:
        action_server_client.move_joint(command)

        assert action_server_client.initial_position == pytest.approx(action_server_client.final_position, abs=1e-1) #0.01

def test_move_head_tilt(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)
    deg = 20
    deltas = {'ccw' : ((math.pi/180) * deg) , 'cw' : -((math.pi/180) * deg)}
    command = {'joint' : 'joint_head_tilt' , 'deltas' : deltas}

    if not action_server_client.start_trajectory_client():
        assert False
    else:
        action_server_client.move_joint(command)

        assert action_server_client.initial_position == pytest.approx(action_server_client.final_position, abs = 1e-1)

def test_move_head_pan(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)
    deg = 20
    deltas = {'ccw' : ((math.pi/180) * deg) , 'cw' : -((math.pi/180) * deg)}
    command = {'joint' : 'joint_head_pan' , 'deltas' : deltas}

    if not action_server_client.start_trajectory_client():
        assert False
    else:
        action_server_client.move_joint(command)

        assert action_server_client.initial_position == pytest.approx(action_server_client.final_position, abs = 1e-1)


def test_move_multiple_joints(node, action_server_client):
    rospy.Subscriber("/stretch/joint_states", JointState, action_server_client.joint_state_callback)
    time.sleep(0.5)
    deg = 30
    rad = ((math.pi/180) * deg)
    translate = 0.09

    joint_names = ['joint_lift','joint_wrist_yaw','joint_head_pan','wrist_extension']

    if not action_server_client.start_trajectory_client():
        assert False
    else:
        action_server_client.move_multiple_joints(joint_names,rad,translate)

        translate_results = all([initial  == pytest.approx(final,abs=1e-2) for initial, final in zip(action_server_client.joint_initial_translate, action_server_client.joint_final_translate)])
        rotate_results = all([initial  == pytest.approx(final,abs=1e-2) for initial, final in zip(action_server_client.joint_initial_rotate, action_server_client.joint_final_rotate)])
        assert translate_results + rotate_results

