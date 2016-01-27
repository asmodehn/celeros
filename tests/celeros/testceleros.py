#!/usr/bin/env python
import unittest
import sys
import os
import time
import rostest
import rospy
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger, TriggerResponse

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from celeros import celeros_app
from celeros import rostasks

import roslaunch

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

    def _topic(self, data):
        print data
        self._injected = True
        return String(data='got me a message!')

    def _service(self, req):
        print req
        self._called = True
        return TriggerResponse(success=True, message='test node called !')

    def setUp(self):
        # Test code is a node
        # => we can detect whats happening to this node, while also launching tasks.
        rospy.init_node('celeros_test')

        config = rospy.get_param('~config')
        if config:
            rospy.logwarn("GOT TEST CONFIG PARAM {0}".format(config))
            celeros_app.config_from_object(config)

        # getting the parameters to configure celery here in the same way as the worker :
        broker_url = rospy.get_param('~broker')
        if broker_url:
            rospy.logwarn("GOT TEST BROKER PARAM {0}".format(broker_url))
            celeros_app.conf.update(
                BROKER_URL=broker_url,
                CELERY_RESULT_BACKEND=broker_url,
            )

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
        result = rostasks.service_call.apply_async(['/celeros_test/called'])

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
        result = rostasks.topic_inject.apply_async(['/celeros_test/injected', {'data':'here is a string'}])

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

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )

    # Ros arguments will tell us if we started from ros, or from straight python
    rosargs = [arg for arg in sys.argv if arg.startswith("__")]

    if len(rosargs) > 0:
        rostest.rosrun('test_celeros', 'test_all', TestCeleros)
    else:
        print("PURE PYTHON TEST NOT IMPLEMENTED YET")
        # Need this solved : http://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/

        # TODO : use this to start all we need to test from here :

        #Start roslaunch
        #launch = roslaunch.scriptapi.ROSLaunch()
        #launch.start()

        # start required nodes
        #empty_srv_node = roslaunch.core.Node('rostful_node', 'emptyService.py', name='empty_service')
        #trigger_srv_node = roslaunch.core.Node('rostful_node', 'triggerService.py', name='trigger_service')
        #empty_srv_process = launch.launch(empty_srv_node)
        #trigger_srv_process = launch.launch(trigger_srv_node)

        #import nose
        #nose.runmodule()
