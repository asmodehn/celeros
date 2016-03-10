from __future__ import absolute_import

import os
import datetime
import logging

import celery
from celery.utils.log import get_logger

import pyros

logger = get_logger(__name__)


# The only way that works to import package content based on file structure
# instead of parent package content, which is not loaded when extending celery.
from .celerybeatredis import RedisScheduleEntry as celerybeatredis_ScheduleEntry
from .celerybeatredis import RedisScheduler as celerybeatredis_Scheduler

from .rosperiodictasks import PeriodicTask


# We use the normal RedisSchedulEntry. This is just here as an example of overloading the behavior for an Entry.
class RedisScheduleEntry(celerybeatredis_ScheduleEntry):

    def is_due(self):
        due = super(RedisScheduleEntry, self).is_due()
        return due


class RedisScheduler(celerybeatredis_Scheduler):
    # Overloading the Entry class for our scheduler
    Entry = RedisScheduleEntry

    def __init__(self, *args, **kwargs):
        logging.warn('{0!r} is starting from {1}'.format(self, __file__))
        super(RedisScheduler, self).__init__(*args, **kwargs)

        # Here an app is setup.
        # And we can get the pyros client :

        print("pyros_client : {}".format(self.app.ros_node_client))

