import math

import rospy

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.menu_dash_widget import MenuDashWidget
from rqt_robot_dashboard.widgets import BatteryDashWidget, ConsoleDashWidget, MonitorDashWidget

from sensor_msgs.msg import BatteryState

from std_msgs.msg import Bool, String

from std_srvs.srv import Trigger


class CalibrateWidget(MenuDashWidget):
    def __init__(self):
        MenuDashWidget.__init__(self, 'Calibration', icons=[
            ['bg-grey.svg', 'ic-motors.svg'],     # State 0: Unknown
            ['bg-green.svg', 'ic-motors.svg'],    # State 1: Calibrated
            ['bg-yellow.svg', 'ic-motors.svg']])  # State 2: Not Calibrated
        self.update_state(0)
        self.setToolTip('Calibration')

        self.client = rospy.ServiceProxy('/calibrate_the_robot', Trigger)
        self.status_sub = rospy.Subscriber('is_calibrated', Bool, self.status_cb, queue_size=1)

        self.add_action('Calibrate!', lambda: self.client.call())

    def status_cb(self, msg):
        if msg.data:
            self.update_state(1)
        else:
            self.update_state(2)


class ModeWidget(MenuDashWidget):
    def __init__(self):
        # TODO: These could probably use custom icons
        MenuDashWidget.__init__(self, 'Robot Mode', icons=[
            ['bg-grey.svg'],                            # State 0: Unknown
            ['bg-green.svg', 'ic-steering-wheel.svg'],  # State 1: Navigation
            ['bg-green.svg', 'ic-wrench.svg'],          # State 2: Manipulation
            ['bg-green.svg', 'ic-runstop-off.svg']])    # State 3: Position
        self.update_state(0)
        self.setToolTip('Unknown Mode')

        self.mode_map = {
            'navigation': 1,
            'manipulation': 2,
            'position': 3
        }

        self.clients = {}
        for mode in self.mode_map:
            self.clients[mode] = rospy.ServiceProxy(f'/switch_to_{mode}_mode', Trigger)
            self.add_action(f'Switch to {mode} mode',
                            lambda mode_arg=mode: self.clients[mode_arg].call())

        self.status_sub = rospy.Subscriber('mode', String, self.status_cb, queue_size=1)

    def status_cb(self, msg):
        if msg.data in self.mode_map:
            self.update_state(self.mode_map[msg.data])
            self.setToolTip(msg.data.title() + ' Mode')
        else:
            self.update_state(0)
            self.setToolTip('Unknown Mode')


class StretchDashboard(Dashboard):
    def __init__(self, context):
        super(StretchDashboard, self).__init__(context)
        self.power_subscriber = rospy.Subscriber('/battery', BatteryState, self.battery_cb, 10)

    def get_widgets(self):
        self.monitor = MonitorDashWidget(self.context)
        self.console = ConsoleDashWidget(self.context)
        self.calibrate = CalibrateWidget()
        self.mode_w = ModeWidget()
        self.battery = BatteryDashWidget(self.context)

        return [[self.monitor, self.console, self.calibrate, self.mode_w], [self.battery]]

    def battery_cb(self, msg):
        # TODO: Incorporate logic if robot is plugged in
        if not math.isnan(msg.percentage):
            self.battery.update_perc(msg.percentage)

        self.battery.setToolTip('{:.2f} V\n{:.2f} A'.format(msg.voltage, msg.current))
