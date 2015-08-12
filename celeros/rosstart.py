#!/usr/bin/env python

try:
    import rospy
except:
    import logging
    logging.error("Could not find rospy. Exiting.")
    raise
    
from celery import Celery, bootsteps

from collections import namedtuple
from rostful_node import RostfulNode
from rostful_node.rostful_client import RostfulClient

class BootRostfulNode(bootsteps.StartStopStep):

    def ros_shutdown(self):
        node.async_stop()
        rospy.logwarn('rostful node stopped')
    
    def __init__(self, worker, **kwargs):
        ros_args = kwargs['ros_args'].split(',')
        # rospy.sleep(10)
        rospy.init_node("celeros", argv=ros_args, anonymous=False, disable_signals=True)
        rospy.logwarn('rostful node started with args : %r', ros_args)

        rospy.on_shutdown(self.ros_shutdown)

        node = RostfulNode()
        client_conn = node.async_spin()
        client = RostfulClient(client_conn)
        rospy.loginfo("finished boot init")

    def create(self, worker):
        return self

    def start(self, worker):
        # our step is started together with all other Worker/Consumer
        # bootsteps.
        rospy.loginfo('{0!r} is starting'.format(worker))

    def stop(self, worker):
        # the Consumer calls stop every time the consumer is restarted
        # (i.e. connection is lost) and also at shutdown.  The Worker
        # will call stop at shutdown only.
        rospy.loginfo('{0!r} is stopping'.format(worker))

    def terminate(self, worker):
        # shutdown is called by the Consumer at shutdown, it's not
        # called by Worker.
        rospy.loginfo('{0!r} is shutting down'.format(worker))

