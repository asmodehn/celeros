from __future__ import absolute_import

import time
import datetime
import random

from celery import Celery
from celery.bin import Option
from celery.result import AsyncResult
from .bootsteps import PyrosBoot, BatteryWatcher

# REQUIRED, to have celerybeatredis subpackage loaded by celery beat...
from . import celerybeatredis

import sys

# move that into __init__. It seems celery apps usually follow same package pattern as flask.
celeros_app = Celery()

# BEWARE : https://github.com/celery/celery/issues/3050
# doing this prevent setting config from command line
# from . import config
# celeros_app.config_from_object(config)

# setting up custom bootstep to start ROS node and pass ROS arguments to it
# for worker ( running on robot )
celeros_app.steps['worker'].add(PyrosBoot)
celeros_app.steps['consumer'].add(BatteryWatcher)
celeros_app.user_options['worker'].add(Option('-R', '--ros-arg', action="append", help='Arguments for ros initialisation'))
# and for beat ( running on concert )
celeros_app.user_options['beat'].add(Option('-R', '--ros-arg', action="append", help='Arguments for ros initialisation'))


#############################
# Basic task for simple tests
#############################

from celery.utils.log import get_task_logger
_logger = get_task_logger(__name__)




# These tasks do not require ROS.
# But they are useful to test if basic celery functionality is working.

# How to test in a shell :
# Normal run (to default queue)
# $ celery -b redis://localhost:6379 --config=celeros.config call celeros.app.add_together --args='[4,6]'
# simulated run (to simulated queue)
# $ celery -b redis://localhost:6379 --config=gopher_tasks.config_localhost call celeros.app.add_together --args='[4,6]' --kwargs='{"simulated": true}'
#

@celeros_app.task()
def add_together(a, b, simulated=False):
    _logger.info("Sleeping 7s")
    time.sleep(7)
    _logger.info("Adding %s + %s" % (a, b))
    return a + b


# Basic task with feedback for simple tests
@celeros_app.task(bind=True)
def long_task(self, simulated=False):
    """Background task that runs a long function with progress reports."""
    verb = ['Starting up', 'Booting', 'Repairing', 'Loading', 'Checking']
    adjective = ['master', 'radiant', 'silent', 'harmonic', 'fast']
    noun = ['solar array', 'particle reshaper', 'cosmic ray', 'orbiter', 'bit']
    message = ''
    total = random.randint(10, 50)
    for i in range(total):
        if not message or random.random() < 0.25:
            message = '{0} {1} {2}...'.format(random.choice(verb),
                                              random.choice(adjective),
                                              random.choice(noun))
        self.update_state(state='PROGRESS',
                          meta={'current': i, 'total': total,
                                'status': message})
        time.sleep(1)
    return {'current': 100, 'total': 100, 'status': 'Task completed! from celeros package', 'result': 42}
