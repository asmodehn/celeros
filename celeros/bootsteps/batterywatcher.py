#!/usr/bin/env python

import sys
import os

import logging
import multiprocessing

import pyros
import kombu
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

    def __init__(self, consumer, **kwargs):
        logging.warn('{0!r} bootstep {1}'.format(consumer, __file__))
        self.battery_topic = None

    def create(self, consumer):
        return self

    def start(self, consumer):
        # our step is started together with all other Worker/Consumer
        # bootsteps.

        self.battery_topic = consumer.app.conf.CELEROS_BATTERY_TOPIC

        if self.battery_topic:  # if we care about the battery
            # Setting up a timer looping to watch Battery levels
            consumer.timer.call_repeatedly(consumer.app.conf.CELEROS_BATTERY_CHECK_PERIOD, self.battery_watcher, args=(consumer,), kwargs={}, priority=0)
        else:
            _logger.info("CELEROS_BATTERY_TOPIC set to None. BatteryWatcher disabled.")

    def battery_watcher(self, consumer):
        try:
            topic_list = consumer.app.ros_node_client.topics()

            if self.battery_topic not in topic_list:
                _logger.warn("Topic {battery_topic} not detected. giving up.".format(battery_topic=self.battery_topic))
                return

            try:
                battery_msg = consumer.app.ros_node_client.topic_extract(
                    self.battery_topic
                )

                battpct = battery_msg.get('percentage', None)  # we assume standard battery message structure here
                if battpct is None:
                    _logger.warn("Battery percentage not found in battery message : {0}".format(battery_msg))
                else:
                    # _logger.info("Watching Battery : {0}% ".format(battpct))
                    # enabling/disabling consumer to queues bound by battery requirements
                    for bpct, q in maybe_list(consumer.app.conf.CELEROS_MIN_BATTERY_PCT_QUEUE):

                        if isinstance(q, kombu.Queue):
                            qname = q.name
                        else:  # assumed str
                            qname = q
                            for kq in consumer.app.conf.CELERY_QUEUES:
                                if kq.name == qname:
                                    # we find a queue with the same name already configured. we should use it.
                                    q = kq
                                    break

                        # to stop consuming from a queue we only need the queue name
                        if battpct < bpct and consumer.task_consumer.consuming_from(qname):
                            _logger.warn("Battery Low {0}%. Ignoring task queue {1} until battery is recharged.".format(battpct, qname))
                            consumer.cancel_task_queue(qname)
                        elif not battpct < bpct and not consumer.task_consumer.consuming_from(qname):
                            # To listen to a queue we might need to create it.
                            # We should reuse the ones from config if possible
                            if isinstance(q, kombu.Queue):
                                consumer.add_task_queue(q)
                            else:  # if we didnt find the queue among the configured queues, we need to create it.
                                consumer.add_task_queue(
                                    queue=qname,
                                    # it seems consumer is not applying the default from config from here?
                                    exchange=consumer.app.conf.CELERY_DEFAULT_EXCHANGE,
                                    exchange_type=consumer.app.conf.CELERY_DEFAULT_EXCHANGE_TYPE,
                                    rounting_key=consumer.app.conf.CELERY_DEFAULT_ROUTING_KEY,
                                )

            except pyros.PyrosServiceTimeout as exc:
                _logger.warn("Failed to get battery levels. giving up.")
                return

        except pyros.PyrosServiceTimeout as exc:
            _logger.warn("Failed to lookup topics. giving up.")

    def stop(self, consumer):
        # The Consumer will call stop() when the connection is lost
        logging.warn('{0!r} stopping {1}'.format(consumer, __file__))
        consumer.timer.clear()



