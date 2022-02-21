#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# Gazebo messages
from gazebo_msgs.msg import ModelStates

# wrap all stuff below in a class later. global variables for now.sorry
goal_dict = {"x" : 0.0, "y": 0.0 , "orientation" : 0.0}

# A lot can be done here. Later 
def generate_goal_pose(data):
    
    person_of_interest_idx = data.name.index('unit_box')
    # return a goal that is 5,5 in front of the object
    goal_dict["x"] = data.pose[person_of_interest_idx].position.x - 8.0
    goal_dict["y"] = data.pose[person_of_interest_idx].position.y - 8.0
    return goal_dict

def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

   # Generate a goal pose that gets us within 5m x and 5m y of the unit_block
    # goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.position.x = goal_dict["x"]
    goal.target_pose.pose.position.x = goal_dict["y"]
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        rospy.Subscriber("gazebo/model_states",ModelStates,generate_goal_pose)
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")