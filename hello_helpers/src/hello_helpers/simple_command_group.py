import hello_helpers.hello_misc as hm


class SimpleCommandGroup:
    def __init__(self, joint_name, joint_range, acceptable_joint_error=0.015, node=None):
        """Simple command group to extend

        Attributes
        ----------
        name: str
            joint name
        range: tuple(float)
            acceptable joint bounds
        active: bool
            whether joint is active
        index: int
            index of joint's goal in point
        goal: dict
            components of the goal
        error: float
            the error between actual and desired
        acceptable_joint_error: float
            how close to zero the error must reach
        """
        self.name = joint_name
        self.range = joint_range
        if self.range is None:
            self.update_joint_range(None, node=node)
        self.active = False
        self.index = None
        self.goal = {"position": None}
        self.error = None
        self.acceptable_joint_error = acceptable_joint_error

    def get_num_valid_commands(self):
        """Returns number of active joints in the group

        Returns
        -------
        int
            the number of active joints within this group
        """
        if self.active:
            return 1

        return 0

    def update_joint_range(self, joint_range, node=None):
        """Updates the commandable joint range

        Parameters
        ----------
        joint_range: tuple(float, float) or None
            updates range if provided, else calculates it automatically
        node: StretchBodyNode or None
            required to calculate range automatically
        """
        raise NotImplementedError

    def update(self, commanded_joint_names, invalid_joints_callback, **kwargs):
        """Activates joints in the group

        Checks commanded joints to activate the command
        group and validates joints used correctly.

        Parameters
        ----------
        commanded_joint_names: list(str)
            list of commanded joints in the trajectory
        invalid_joints_callback: func
            error callback for misuse of joints in trajectory

        Returns
        -------
        bool
            False if commanded joints invalid, else True
        """
        self.active = False
        self.index = None
        if self.name in commanded_joint_names:
            self.index = commanded_joint_names.index(self.name)
            self.active = True

        return True

    def set_goal(self, point, invalid_goal_callback, fail_out_of_range_goal, **kwargs):
        """Sets goal for the joint group

        Sets and validates the goal point for the joints
        in this command group.

        Parameters
        ----------
        point: trajectory_msgs.JointTrajectoryPoint
            the target point for all joints
        invalid_goal_callback: func
            error callback for invalid goal
        fail_out_of_range_goal: bool
            whether to bound out-of-range goals to range or fail

        Returns
        -------
        bool
            False if commanded goal invalid, else True
        """
        self.goal = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        if self.active:
            goal_pos = point.positions[self.index] if len(point.positions) > self.index else None
            if goal_pos is None:
                err_str = ("Received goal point with positions array length={0}. "
                           "This joint ({1})'s index is {2}. Length of array must cover all joints listed "
                           "in commanded_joint_names.").format(len(point.positions), self.name, self.index)
                invalid_goal_callback(err_str)
                return False

            self.goal['position'] = hm.bound_ros_command(self.range, goal_pos, fail_out_of_range_goal)
            self.goal['velocity'] = point.velocities[self.index] if len(point.velocities) > self.index else None
            self.goal['acceleration'] = point.accelerations[self.index] if len(point.accelerations) > self.index else None
            self.goal['contact_threshold'] = point.effort[self.index] if len(point.effort) > self.index else None
            if self.goal['position'] is None:
                err_str = ("Received {0} goal point that is out of bounds. "
                            "Range = {1}, but goal point = {2}.").format(self.name, self.range, goal_pos)
                invalid_goal_callback(err_str)
                return False

        return True

    def init_execution(self, robot, robot_status, **kwargs):
        """Starts execution of the point

        Uses Stretch's Python API to begin moving to the
        target point.

        Parameters
        ----------
        robot: stretch_body.robot.Robot
            top-level interface to Python API
        robot_status: dict
            robot's current status
        """
        raise NotImplementedError

    def update_execution(self, robot_status, **kwargs):
        """Monitors progress of joint group

        Checks against robot's status to track progress
        towards the target point.

        This method must set self.error.

        Parameters
        ----------
        robot_status: dict
            robot's current status

        Returns
        -------
        float/None
            error value if group active, else None
        """
        raise NotImplementedError

    def goal_reached(self):
        """Returns whether reached target point

        Returns
        -------
        bool
            if active, whether reached target point, else True
        """
        if self.active:
            return (abs(self.error) < self.acceptable_joint_error)

        return True

    def joint_state(self, robot_status, **kwargs):
        """Returns state of the joint group

        Parameters
        ----------
        robot_status: dict
            whole robot's current status

        Returns
        -------
        (float, float, float)
            Current position, velocity, and effort
        """
        raise NotImplementedError
