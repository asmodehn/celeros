#!/usr/bin/env python

import sys
import os

import logging
import multiprocessing
    
from celery import Celery, bootsteps
from celery.platforms import signals as _signals
from celery.utils.log import get_logger

logger = get_logger(__name__)


# TODO : configuration for tests...
class BootPyrosNode(bootsteps.StartStopStep):

    requires = ('celery.worker.components:Pool',)

    def __init__(self, worker, **kwargs):
        logging.warn('{0!r} is starting from {1}'.format(worker, __file__))

        # dynamic setup and import ( in different process now )
        try:
            # this will import rosinterface and if needed simulate ROS setup
            import sys
            import pyros
            import pyros.rosinterface
            # this doesnt work with celery handling of imports
            # sys.modules["pyros.rosinterface"] = pyros.rosinterface.delayed_import_auto(
            #     distro='indigo',
            #     base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')
            # )
            pyros.rosinterface = pyros.rosinterface.delayed_import_auto(
                distro='indigo',
                base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')
            )
        except ImportError as e:
            logging.warn("{name} Error: Could not import pyros.rosinterface : {e}".format(name=__name__, e=e))
            raise

        self.ros_argv = kwargs['ros_arg'] if 'ros_arg' in kwargs else []
        self.node_proc = pyros.rosinterface.PyrosROS(
            'celeros',
            self.ros_argv,
            base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')
        )
        client_conn = self.node_proc.start()
        # we do this in init so all pool processes have access to it.
        worker.app.ros_node_client = pyros.PyrosClient(client_conn)

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
        logging.warn('{0!r} is stopping. Attempting termination of current tasks...'.format(worker))

        # Following code from worker.control.revoke

        terminated = set()

        # cleaning all reserved tasks since we are shutting down
        signum = _signals.signum('TERM')
        for request in [r for r in worker.state.reserved_requests]:
            if request.id not in terminated:
                terminated.add(request.id)
                logger.info('Terminating %s (%s)', request.id, signum)
                request.terminate(worker.pool, signal=signum)

        # Aborting currently running tasks, and triggering soft timeout exception to allow task to clean up.
        signum = _signals.signum('USR1')
        for request in [r for r in worker.state.active_requests]:
            if request.id not in terminated:
                terminated.add(request.id)
                logger.info('Terminating %s (%s)', request.id, signum)
                request.terminate(worker.pool, signal=signum)  # triggering SoftTimeoutException in Task

        if terminated:
            terminatedstr = ', '.join(terminated)
            logger.info('Tasks flagged as revoked: %s', terminatedstr)

        self.node_proc.shutdown()



