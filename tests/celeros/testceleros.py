#!/usr/bin/env python
import logging
import unittest
import sys
import os
import time

# Unit test import
try:
    import rostest
    import rospy
    import roslaunch
    from std_msgs.msg import String, Empty
    from std_srvs.srv import Trigger, TriggerResponse
    from pyros_setup import rostest_nose
except ImportError as exc:
    import os
    import sys
    import pyros_setup
    pyros_setup = pyros_setup.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
    import rostest
    import rospy
    import roslaunch
    from std_msgs.msg import String, Empty
    from std_srvs.srv import Trigger, TriggerResponse
    from pyros_setup import rostest_nose

from celeros import celeros_app
from celeros import rostasks

launch = None
worker_process = None

# TODO : maybe we need a config for tests specifically ( instead of using the default )
app = "celeros.app"  # OR celeros.celeros_app OR celeros_app ( to match code )??
config = "celeros.config"


# Should be used only by nose ( or other python test tool )
# CAREFUL with comments, copy paste mistake are real...
# CAREFUL dont use assertFalse -> easy to miss when reviewing code
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # Start roslaunch
        global launch
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # start required worker - needs to match the content of *.test files for rostest to match
        global worker_process

        rospy.set_param('/celeros/topics', "['/celeros_test.*/injected','/celeros_test.*/extracted']")
        rospy.set_param('/celeros/services', "['/celeros_test.*/called']")
        worker_node = roslaunch.core.Node(
                'celeros', 'celeros',
                name='celeros',
                args=" -A " + app +
                     " --config " + config
        )
        worker_process = launch.launch(worker_node)


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        # finishing all process are finished
        if worker_process is not None:
            worker_process.stop()

        rostest_nose.rostest_nose_teardown_module()


class Timeout(object):
    """
    Small useful timeout class
    """
    def __init__(self, seconds):
        self.seconds = seconds

    def __enter__(self):
        self.die_after = time.time() + self.seconds
        return self

    def __exit__(self, type, value, traceback):
        pass

    @property
    def timed_out(self):
        return time.time() > self.die_after


class TestCeleros(unittest.TestCase):

    @classmethod
    def setUpClass(cls):

        # we still need a node to interact with topics
        rospy.init_node('celeros_test', anonymous=True)
        # CAREFUL : this should be done only once per PROCESS
        # Here we enforce TEST class 1<->1 PROCESS. ROStest style

        # set required parameters - needs to match the content of *.test files for rostest to match
        rospy.set_param("~config", config)

    @classmethod
    def tearDownClass(cls):
        rospy.signal_shutdown('test complete')

    def _topic(self, data):
        print data
        self._injected = True
        return String(data='got me a message!')

    def _service(self, req):
        print req
        self._called = True
        return TriggerResponse(success=True, message='test node called !')

    def setUp(self):
        config = rospy.get_param('~config')
        if config:
            rospy.logwarn("GOT TEST CONFIG PARAM {0}".format(config))
            celeros_app.config_from_object(config)

    def tearDown(self):
        # the node will shutdown with the test process.
        pass

    def test_service_call(self):
        # service to be called
        self._called = False
        self._srv = rospy.Service('~called', Trigger, self._service)

        # constant used just to prevent spinning too fast
        overspin_sleep_val = 0.02

        def prevent_overspin_sleep():
            time.sleep(overspin_sleep_val)

        # we wait a bit for the service to be discovered
        time.sleep(2)

        # we send the task to the worker
        result = rostasks.service_call.apply_async([rospy.resolve_name('~called')])

        # we wait until we get called
        with Timeout(60) as t:
            while not t.timed_out and not self._called:
                prevent_overspin_sleep()

        assert not t.timed_out and self._called

    def test_topic_inject(self):
        self._injected = False
        self._sub = rospy.Subscriber('~injected', String, self._topic)

        # constant use just to prevent spinning too fast
        overspin_sleep_val = 0.02

        def prevent_overspin_sleep():
            time.sleep(overspin_sleep_val)

        # we wait a bit for the service to be discovered
        time.sleep(2)

        # we send the task to the worker
        result = rostasks.topic_inject.apply_async([rospy.resolve_name('~injected'), {'data':'here is a string'}])

        # we wait until we get called
        with Timeout(60) as t:
            while not t.timed_out and not self._injected:
                prevent_overspin_sleep()

        assert not t.timed_out and self._injected

    # def test_topic_extract(self):
    #
    #     self._extracted = False
    #     self._pub = rospy.Publisher('~extracted', String, queue_size=1)
    #     pass


if __name__ == '__main__':
    print("ARGV : %r", sys.argv)
    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('celeros', 'testCeleros', TestCeleros, sys.argv)
