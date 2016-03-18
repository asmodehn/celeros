#
#  sample celeros configuration.
# this file can be overloaded and passed as argument to change the celeros configuration
#
# TODO : move to a cleaner way to do configuration ( probably after mutating into up a pure python package )

from kombu import Queue
from .config import *

# We only override what is different from normal config.
CELERY_QUEUES = (
)

# (List of) tuples of battery levels and queues that can be consumed from,
# only if the battery has a percentage higher than the specified level
CELEROS_MIN_BATTERY_PCT_QUEUE = [
    (10, Queue('simulated.celeros', routing_key='simulated.celeros'))
]
# This will automatically create the required queues. No need to specify CELERY_QUEUES here.

