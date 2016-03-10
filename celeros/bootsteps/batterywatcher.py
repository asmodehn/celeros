#!/usr/bin/env python

import sys
import os

import logging
import multiprocessing

import pyros
from celery import Celery, bootsteps
from celery.platforms import signals as _signals
from celery.utils.log import get_logger
from celery.utils.functional import maybe_list

_logger = get_logger(__name__)

# TODO : fix logging : http://docs.celeryproject.org/en/latest/userguide/extending.html#installing-bootsteps
# logging is not reentrant, and methods here are called in different ways...

# TODO : configuration for tests...
class BatteryWatcher(bootsteps.StartStopStep):
    """
    This is a worker bootstep. It starts a timer to watch for battery levels.
    Pyros bootstep is required for this to be able to work.
    """

    def __init__(self, worker, **kwargs):
        logging.warn('{0!r} bootstep {1}'.format(worker, __file__))
        self.battery_topic = None

    def create(self, worker):
        return self

    def start(self, worker):
        # our step is started together with all other Worker/Consumer
        # bootsteps.

        self.battery_topic = worker.app.conf.CELEROS_BATTERY_TOPIC

        if self.battery_topic:  # if we care about the battery
            # Setting up a timer looping to watch Battery levels
            worker.timer.call_repeatedly(60, self.battery_watcher, args=(worker,), kwargs={}, priority=0)
        else:
            _logger.info("CELEROS_BATTERY_TOPIC set to None. BatteryWatcher disabled.")

    def battery_watcher(self, worker):
        try:
            topic_list = worker.app.ros_node_client.topics()

            if self.battery_topic not in topic_list:
                _logger.warn("Topic {battery_topic} not detected. giving up.".format(battery_topic=self.battery_topic))
                return

            try:
                battery_msg = worker.app.ros_node_client.topic_extract(
                    self.battery_topic
                )

                battpct = battery_msg.get('percentage', None)  # we assume standard battery message structure here
                if battpct is None:
                    _logger.warn("Battery percentage not found in battery message : {0}".format(battery_msg))
                else:
                    # _logger.info("Watching Battery : {0}% ".format(battpct))
                    # enabling/disabling consumer to queues bound by battery requirements
                    for bpct, q in maybe_list(worker.app.conf.CELEROS_MIN_BATTERY_PCT_QUEUE):
                        if battpct < bpct:
                            _logger.warn("Battery Low {0}%. Ignoring task queue {1} until battery is recharged.".format(battpct, q))
                            worker.cancel_task_queue(q)
                        else:
                            worker.add_task_queue(q)

            except pyros.PyrosServiceTimeout as exc:
                _logger.warn("Failed to get battery levels. giving up.")
                return

        except pyros.PyrosServiceTimeout as exc:
            _logger.warn("Failed to lookup services. giving up.")

    def stop(self, worker):
        # The Worker will call stop at shutdown only.
        logging.warn('{0!r} stopping {1}'.format(worker, __file__))
        worker.timer.clear()



