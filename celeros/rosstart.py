#!/usr/bin/env python

import sys
import os

import logging
import multiprocessing
    
from celery import Celery, bootsteps


class BootRostfulNode(bootsteps.StartStopStep):

    def __init__(self, worker, **kwargs):
        # TODO : super(BootRostfulNode, self).__init__()

        # dynamic setup and import ( in different process now )
        try:
            # this will import rosinterface and if needed simulate ROS setup
            import sys
            import pyros
            import pyros.rosinterface
            # this doesnt work ??
            #sys.modules["pyros.rosinterface"] = pyros.rosinterface.delayed_import_auto(distro='indigo', base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..'))
            pyros.rosinterface = pyros.rosinterface.delayed_import_auto(distro='indigo', base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        except ImportError as e:
            logging.warn("{name} Error: Could not import pyros.rosinterface : {e}".format(name=__name__, e=e))
            raise

        self.ros_argv = kwargs['ros_arg'] if 'ros_arg' in kwargs else []
        self.node_proc = pyros.rosinterface.PyrosROS('celeros', self.ros_argv, base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        client_conn = self.node_proc.start()
        worker.app.ros_node_client = pyros.PyrosClient(client_conn)  # we do this in init so all pool processes have acces to it.
        logging.warn('{0!r} is starting'.format(worker))
        logging.warn("finished boot init")

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
        logging.logwarn('{0!r} is stopping'.format(worker))
        self.node_proc.shutdown()


