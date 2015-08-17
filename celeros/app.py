from __future__ import absolute_import

import time
import random

from . import config
from celery import Celery

celeros_app = Celery()
celeros_app.config_from_object(config.Development)

from celery.utils.log import get_task_logger
_logger = get_task_logger(__name__)


# Basic task for simple tests
@celeros_app.task(bind=True)
def add_together(a, b):
    _logger.info("Sleeping 7s")
    time.sleep(7)
    _logger.info("Adding %s + %s" % (a, b))
    return a + b


# Basic task with feedback for simple tests
@celeros_app.task(bind=True)
def long_task(self):
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
    return {'current': 100, 'total': 100, 'status': 'Task completed! from flask_task_planner package', 'result': 42}
