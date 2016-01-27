from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'celeros')))

from celeros.worker import Worker
from celeros import config


class TestWorker(object):
    def setUp(self):

        broker = "redis://localhost:6379"
        config = "celeros.config"
        tasks = "celeros.rostasks"

        self.workerInstance = Worker(
            hostname=hostname, pool_cls=pool_cls, loglevel=loglevel,
            logfile=logfile,  # node format handled by celery.app.log.setup
            pidfile=self.node_format(pidfile, hostname),
            state_db=self.node_format(state_db, hostname), **kwargs
        ).start()
        cmd_conn = self.workerInstance.launch( broker_url=broker, tasks=tasks, config=config, ros_args='')

    def tearDown(self):
        self.workerInstance.async_stop()
        pass

    def test_configuration(self):
        """
        Test the worker has the specified configuration
        """




# Test the worker can execute a task




# Test the worker can receive and execute only one task at a time



if __name__ == '__main__':

    import nose
    nose.runmodule()
