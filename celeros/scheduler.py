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


class RedisScheduleEntry(celerybeatredis_ScheduleEntry):

    def __init__(self, scheduler_url, task):
        super(RedisScheduleEntry, self).__init__(scheduler_url, task)

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

    def get_from_database(self, entry_class=None):

        entry_class = entry_class or self.Entry

        # First we get all entries from celerybeatredis scheduler
        entries = super(RedisScheduler, self).get_from_database(entry_class)

        # Note : Here we can access self.app and get workers information to modify the schedule if needed

        return entries

    @property
    def schedule(self):
        """
        This is called by celery scheduler. We need to return a dict of entries with a is_due method
        to indicate if this should be run or not.
        :return:
        """
        if self.requires_update():
            self._schedule = self.get_from_database()
            self._last_updated = datetime.datetime.now()
        return self._schedule

