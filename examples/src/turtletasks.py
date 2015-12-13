from __future__ import absolute_import

import time
import random

from celeros import celeros_app
import rospy
import rostful_node

from celery import states
from celery.contrib.abortable import AbortableTask
from celery.exceptions import Ignore, Reject
from celery.utils.log import get_task_logger

_logger = get_task_logger(__name__)


@celeros_app.task(bind=True)
def turtle_move(self):
    """Task to move the turtle around - TODO """
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
