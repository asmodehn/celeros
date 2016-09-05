#!/usr/bin/env python

import sys
import os

import logging
import multiprocessing
from functools import partial

import pyros
from celery import Celery, bootsteps
from celery.platforms import signals as _signals
from celery.utils.log import get_logger

_logger = get_logger(__name__)


# TODO : fix logging : http://docs.celeryproject.org/en/latest/userguide/extending.html#installing-bootsteps
# logging is not reentrant, and methods here are called in different ways...

# TODO : configuration for tests...
class PyrosBoot(bootsteps.StartStopStep):
    """
    This is a worker bootstep. It starts the pyros node and a client.
    That client can then be used in tasks and in other places in celery customization code
    """
    requires = ('celery.worker.components:Pool', )

    def __init__(self, worker, **kwargs):
        logging.warn('{0!r} bootstep {1}'.format(worker, __file__))

        # dynamic setup and import ( in different process now )
        try:
            import pyros
            self.ros_argv = kwargs['ros_arg'] if 'ros_arg' in kwargs else []
            self.node_proc = pyros.PyrosROS(
                'celeros',
                self.ros_argv
            )
            # Attribute error is triggered when using pyros < 0.1.0
        except (ImportError, AttributeError) as e:
            #logging.warn("{name} Error: Could not import pyros : {e}".format(name=__name__, e=e))
            ### TMP ###
            logging.warn("{name} Attempting bwcompat import : {e}".format(name=__name__, e=e))
            # BWcompat with old pyros :
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

                self.ros_argv = kwargs['ros_arg'] if 'ros_arg' in kwargs else []
                self.node_proc = pyros.rosinterface.PyrosROS(
                    'celeros',
                    self.ros_argv,
                    base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')
                )

            except ImportError as e:
                logging.warn("{name} Error: Could not import pyros.rosinterface : {e}".format(name=__name__, e=e))
                raise

        client_conn = self.node_proc.start()
        # we do this in init so all pool processes have access to it.
        worker.app.ros_node_client = pyros.PyrosClient(client_conn)

    def create(self, worker):
        return self

    def start(self, worker):
        # our step is started together with all other Worker/Consumer
        # bootsteps.
        pass

    def stop(self, worker):
        # The Worker will call stop at shutdown only.
        logging.warn('{0!r} is stopping. Attempting termination of current tasks...'.format(worker))

        # Following code from worker.control.revoke
        terminated = set()

        # cleaning all reserved tasks since we are shutting down
        signum = _signals.signum('TERM')
        for request in [r for r in worker.state.reserved_requests]:
            if request.id not in terminated:
                terminated.add(request.id)
                _logger.info('Terminating %s (%s)', request.id, signum)
                request.terminate(worker.pool, signal=signum)

        # Aborting currently running tasks, and triggering soft timeout exception to allow task to clean up.
        signum = _signals.signum('USR1')
        for request in [r for r in worker.state.active_requests]:
            if request.id not in terminated:
                terminated.add(request.id)
                _logger.info('Terminating %s (%s)', request.id, signum)
                request.terminate(worker.pool, signal=signum)  # triggering SoftTimeoutException in Task

        if terminated:
            terminatedstr = ', '.join(terminated)
            _logger.info('Tasks flagged as revoked: %s', terminatedstr)

        self.node_proc.shutdown()



