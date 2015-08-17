#!/usr/bin/env python

try:
    import rospy
except:
    import logging
    logging.error("Could not find rospy. Exiting.")
    raise

import multiprocessing
    
from celery import Celery, bootsteps

from rostful_node.rostful_node_process import RostfulNodeProcess
from rostful_node.rostful_client import RostfulClient

class BootRostfulNode(bootsteps.StartStopStep):

    def ros_shutdown(self):
        self.node_proc.terminate()
    
    def __init__(self, worker, **kwargs):
        ros_args = []
        if 'ros_args' in kwargs and kwargs['ros_args']:
            ros_args = kwargs['ros_args'].split(',')

        self.node_proc = RostfulNodeProcess()
        client_conn = self.node_proc.launch('celeros', ros_args)
        self.client = RostfulClient(client_conn)
        rospy.logwarn("finished boot init")

    def create(self, worker):
        return self

    def start(self, worker):
        # our step is started together with all other Worker/Consumer
        # bootsteps.
        rospy.logwarn('{0!r} is starting'.format(worker))

    def stop(self, worker):
        # the Consumer calls stop every time the consumer is restarted
        # (i.e. connection is lost) and also at shutdown.  The Worker
        # will call stop at shutdown only.
        rospy.logwarn('{0!r} is stopping'.format(worker))
        self.ros_shutdown()

    def terminate(self, worker):
        # shutdown is called by the Consumer at shutdown, it's not
        # called by Worker.
        rospy.logwarn('{0!r} is shutting down'.format(worker))

