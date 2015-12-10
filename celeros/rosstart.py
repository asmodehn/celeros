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
    from pyros.rosinterface import PyrosROS
    from pyros.pyros_client import PyrosClient
except ImportError, e:
    print "Exception caught : ", e
    print "rostful_node module is probably not accessible in sys.path. Please check, it is required to run celeros."
    print "sys.path = %r", sys.path
    raise


class BootRostfulNode(bootsteps.StartStopStep):

    def __init__(self, worker, **kwargs):
        self.ros_argv = kwargs['ros_arg'] if 'ros_arg' in kwargs else []
        self.node_proc = PyrosROS('celeros', self.ros_argv)
        client_conn = self.node_proc.start()
        worker.app.ros_node_client = PyrosClient(client_conn)  # we do this in init so all pool processes have acces to it.
        rospy.logwarn('{0!r} is starting'.format(worker))
        rospy.logwarn("finished boot init")

    def create(self, worker):
        return self

    def start(self, worker):
        # our step is started together with all other Worker/Consumer
        # bootsteps.
        pass  # not sure in which process this is run.


    def stop(self, worker):
        # the Consumer calls stop every time the consumer is restarted
        # (i.e. connection is lost) and also at shutdown.  The Worker
        # will call stop at shutdown only.
        rospy.logwarn('{0!r} is stopping'.format(worker))
        self.node_proc.shutdown()


