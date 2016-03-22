from __future__ import absolute_import

import os
import datetime
import logging
import pprint

import celery
from celery.utils.log import get_logger

import pyros

logger = get_logger(__name__)


# The only way that works to import package content based on file structure
# instead of parent package content, which is not loaded when extending celery.
from .celerybeatredis import RedisScheduleEntry as celerybeatredis_ScheduleEntry
from .celerybeatredis import RedisScheduler as celerybeatredis_Scheduler

from .rosperiodictasks import PeriodicTask


# We use the normal RedisScheduleEntry. This is just here as an example of overloading the behavior for an Entry.
class RedisScheduleEntry(celerybeatredis_ScheduleEntry):

    def __init__(self, name=None, task=None, enabled=True, fire_and_forget=False, last_run_at=None,
                 total_run_count=None, schedule=None, args=(), kwargs=None,
                 options=None, app=None, **extrakwargs):
        """
        :param name: the name of the task ( = redis key )
        :param task: the task itself (python function)
        :param enabled: whether the task is enabled and should run
        :param fire_and_forget: whether the task should be deleted after triggering once
        :param last_run_at: last time the task was run
        :param total_run_count: the number of time the task was run
        :param schedule: the schedule for the task
        :param args: the args for the task
        :param kwargs: the kwargs for the task
        :param options: the options for the task
        :param app: the current app
        :param extrakwargs: extra kwargs to support extra json fields if needed.
        :return:
        """
        super(RedisScheduleEntry, self).__init__(
            name=name, task=task, enabled=enabled, fire_and_forget=fire_and_forget, last_run_at=last_run_at,
            total_run_count=total_run_count, schedule=schedule, args=args, kwargs=kwargs,
            options=options, app=app, **extrakwargs
        )

    def is_due(self):
        due = super(RedisScheduleEntry, self).is_due()
        return due


class RedisScheduler(celerybeatredis_Scheduler):
    # Overloading the Entry class for our scheduler
    Entry = RedisScheduleEntry

    def __init__(self, *args, **kwargs):
        logging.warn('{0!r} is starting from {1}'.format(self, __file__))
        super(RedisScheduler, self).__init__(*args, **kwargs)

        self._purge = set()  # keeping entries to delete by name for sync later on

        # Here an app is setup.
        # And we can get the pyros client :
        print("pyros_client : {}".format(self.app.ros_node_client))

    def reserve(self, entry):
        # called when the task is about to be run (and data will be modified -> sync() will need to save it)

        # this will add the entry to a dirty list to write change into db during next sync
        new_entry = super(RedisScheduler, self).reserve(entry)

        # If this entry is fire_and_forget, we need to delete it later
        if new_entry.fire_and_forget:
            self._purge.add(new_entry.name)
            self._dirty.remove(new_entry.name)  # no need to save whatever was modified for this entry

        return new_entry

    # Overload this if you need to modify the way the task is run.
    # check parent classes for reference implementation
    def apply_async(self, entry, publisher=None, **kwargs):
        logger.info("triggering schedule entry : {0}".format(pprint.pformat(entry.__dict__)))
        return super(RedisScheduler, self).apply_async(entry, publisher, **kwargs)

    def sync(self):
        logger.info('cleaning up entries to be deleted...')

        _tried_purge = set()
        try:
            while self._purge:
                name = self._purge.pop()
                _tried_purge.add(name)
                # delete entries that need to be purged
                self.rdb.delete(name)
        except Exception as exc:
            # retry later
            self._purge |= _tried_purge
            logger.error('Error while sync: %r', exc, exc_info=1)

        super(RedisScheduler, self).sync()
