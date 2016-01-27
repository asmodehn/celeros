from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'celeros')))

from celeros.worker import Worker
from celeros import config

class TestWorker(object):
    def setUp(self):
        self.workerInstance = Worker()
        cmd_conn = self.workerInstance.launch( broker_url='', tasks='', config='', ros_args='')

    def tearDown(self):
        self.workerInstance.async_stop()
        pass

    Worker()



# Test the worker has the specified configuration




# Test the worker can execute a task




# Test the worker can receive and execute only one task at a time