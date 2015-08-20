#!/usr/bin/env python

import sys

try:
    import rospy
except:
    import logging
    logging.error("Could not find rospy. Exiting.")
    raise

import multiprocessing
    
from celery import Celery, bootsteps

try:
    from rostful_node.rostful_node_process import RostfulNodeProcess
    from rostful_node.rostful_client import RostfulClient
except ImportError, e:
    print "Exception caught : ", e
    print "rostful_node module is probably not accessible in sys.path. Please check, it is required to run celeros."
    print "sys.path = %r", sys.path
    raise


class BootRostfulNode(bootsteps.StartStopStep):

    def __init__(self, worker, **kwargs):
        self.node_proc = RostfulNodeProcess()
        ros_argv = kwargs['ros_arg'] if 'ros_arg' in kwargs else []
        client_conn = self.node_proc.launch('celeros', ros_argv)
        worker.app.ros_node_client = RostfulClient(client_conn)
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
        self.node_proc.terminate()


