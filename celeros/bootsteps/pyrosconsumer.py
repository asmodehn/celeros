#!/usr/bin/env python

import sys
import os

import logging
import multiprocessing

from celery import Celery, bootsteps
from kombu import Consumer, Queue
from celery.platforms import signals as _signals
from celery.utils.log import get_logger

logger = get_logger(__name__)


# If We need more than a CustomerStep
class PyrosConsumer(bootsteps.StartStopStep):

    requires = ('celery.worker.consumer:Tasks',)

    def __init__(self, parent, **kwargs):
        # here we can prepare the Worker/Consumer object
        # in any way we want, set attribute defaults and so on.
        print('{0!r} is in init'.format(parent))

        # The Pyros client should have been started by the worker custom booststep
        # and should be accessible here
        print(parent.app.ros_node_client)

    def start(self, parent):
        # our step is started together with all other Worker/Consumer
        # bootsteps.
        print('{0!r} is starting'.format(parent))

    def stop(self, parent):
        # the Consumer calls stop every time the consumer is restarted
        # (i.e. connection is lost) and also at shutdown.
        print('{0!r} is stopping'.format(parent))

    def shutdown(self, parent):
        # shutdown is called by the Consumer at shutdown.
        print('{0!r} is shutting down'.format(parent))
        self.node_proc.shutdown()


# If not, we can go the easy way
# class PyrosConsumer(bootsteps.ConsumerStep):
#
#     def get_consumers(self, channel):
#         return [Consumer(channel,
#                          queues=[my_queue],
#                          callbacks=[self.handle_message],
#                          accept=['json'])]
#
#     def handle_message(self, body, message):
#         print('Received message: {0!r}'.format(body))
#         message.ack()

