import diagnostic_msgs

import diagnostic_updater

import stretch_body.hello_utils as hello_utils  # For get_fleet_id

# Shortcut constants
ERROR = diagnostic_msgs.msg.DiagnosticStatus.ERROR
WARN = diagnostic_msgs.msg.DiagnosticStatus.WARN
OK = diagnostic_msgs.msg.DiagnosticStatus.OK


def add_keyvalues_to_status(status, d, skip_keys=None):
    """Add all the key value pairs in dictionary d (except those keys in skip_keys) to the diagnostic status."""
    for k, v in d.items():
        if skip_keys and k in skip_keys:
            continue
        status.add(k, str(v))


def report_stats(status, d, skip_keys=None):
    """Diagnostic update function that marks the summary as OK and just reports the stats from d."""
    status.summary(OK, 'OK')
    add_keyvalues_to_status(status, d, skip_keys)
    return status


def analyze_stepper(status, device):
    """Diagnostic update function for device with stepper motor."""
    if not device.motor.hw_valid:
        status.summary(ERROR, 'Hardware not present')
    elif not device.motor.status['pos_calibrated']:
        status.summary(WARN, 'Hardware not calibrated')
    elif device.motor.status['runstop_on']:
        status.summary(WARN, 'Runstop on')
    else:
        status.summary(OK, 'Hardware present and calibrated')

    add_keyvalues_to_status(status, device.status, ['motor'])
    return status


def analyze_dynamixel(status, device, joint_name):
    """Diagnostic update function for device with dynamixel motor."""
    motor = device.motors[joint_name]
    if not motor.hw_valid:
        status.summary(ERROR, 'Hardware not present')
    elif not motor.is_calibrated:
        status.summary(WARN, 'Hardware not calibrated')
    else:
        status.summary(OK, 'Hardware present and calibrated')

    add_keyvalues_to_status(status, motor.status)

    return status


def analyze_base(status, base):
    """Diagnostic update function for the base."""
    for wheel in [base.left_wheel, base.right_wheel]:
        if not wheel.hw_valid:
            status.summary(ERROR, 'Hardware not present')
            break
    else:
        status.summary(OK, 'Hardware present')

    add_keyvalues_to_status(status, base.status, ['left_wheel', 'right_wheel'])
    return status


def check_range(status, device, key, lower_bound, upper_bound, out_of_bounds_status=ERROR):
    """Diagnostic update function that checks if value(s) exceeds the specified bounds."""
    value = device.status[key]
    if isinstance(value, list):
        values = value
        for i, value in enumerate(values):
            status.add(f'{key}[{i}]', str(value))
    else:
        status.add(key, str(value))
        values = [value]

    for value in values:
        if value < lower_bound or value > upper_bound:
            status.summary(out_of_bounds_status, f'{key}={value} | out of range [{lower_bound},{upper_bound}]')
            return status

    status.summary(OK, 'OK')
    return status


class StretchDiagnostics:
    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        self.updater = diagnostic_updater.Updater(node)
        self.updater.setHardwareID(hello_utils.get_fleet_id())

        # Configure the different update functions
        self.updater.add('Pimu/Device',
                         lambda status: report_stats(status, self.robot.pimu.status, ['imu', 'transport']))

        # TODO: Add IMU
        self.updater.add('Pimu/Voltage',
                         lambda status: check_range(status, self.robot.pimu, 'voltage',
                                                    self.robot.pimu.config['low_voltage_alert'], 14.5))
        self.updater.add('Pimu/Current',
                         lambda status: check_range(status, self.robot.pimu, 'current',
                                                    0.5, self.robot.pimu.config['high_current_alert']))
        self.updater.add('Pimu/Temperature', lambda status: check_range(status, self.robot.pimu, 'temp', 10, 40))
        self.updater.add('Pimu/Cliff',
                         lambda status: check_range(status, self.robot.pimu, 'cliff_range',
                                                    self.robot.pimu.config['cliff_thresh'], 20))

        self.updater.add('Lift/Device', lambda status: analyze_stepper(status, self.robot.lift))
        self.updater.add('Lift/Motor', lambda status: report_stats(status, self.robot.lift.motor.status, ['transport']))
        self.updater.add('Arm/Device', lambda status: analyze_stepper(status, self.robot.arm))
        self.updater.add('Arm/Motor', lambda status: report_stats(status, self.robot.arm.motor.status, ['transport']))
        self.updater.add('Base/Device', lambda status: analyze_base(status, self.robot.base))
        self.updater.add('Base/LeftWheel',
                         lambda status: report_stats(status, self.robot.base.left_wheel.status, ['transport']))
        self.updater.add('Base/RightWheel',
                         lambda status: report_stats(status, self.robot.base.right_wheel.status, ['transport']))
        self.updater.add('Wacc/Device', lambda status: check_range(status, self.robot.wacc, 'ax', 8.0, 11.0))

        for name, device in [('Head', self.robot.head), ('EndOfArm', self.robot.end_of_arm)]:
            for jname in device.joints:
                self.updater.add(f'{name}/{jname}', lambda status: analyze_dynamixel(status, device, jname))
