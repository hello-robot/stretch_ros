import argparse
import sys
import time
import typing
import py_trees
import py_trees.console as console

class MoveToPredock(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MoveToPredock, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [MoveToPredock::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [MoveToPredock::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [MoveToPredock::update()]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [MoveToPredock::terminate()]" % self.name)


class MoveToDock(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MoveToDock, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [MoveToDock::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [MoveToDock::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [MoveToDock::update()]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [MoveToDock::terminate()]" % self.name)


class CameraScan(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CameraScan, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [CameraScan::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CameraScan::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [CameraScan::update()]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [CameraScan::terminate()]" % self.name)

class ArucoDetect(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ArucoDetect, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [ArucoDetect::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ArucoDetect::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [ArucoDetect::update()]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [ArucoDetect::terminate()]" % self.name)

class PredockPose(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(PredockPose, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [PredockPose::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [PredockPose::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [PredockPose::update()]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [PredockPose::terminate()]" % self.name)

class Charging(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Charging, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [Charging::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Charging::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Charging::update()]" % self.name)
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Charging::terminate()]" % self.name)

