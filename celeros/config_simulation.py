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

# Note : celery groups tasks by queues in order to send them to different workers.
# But the default case is that all worker should mostly able to do all tasks
# For robot the usual assumption is different : only one robot can do one task.
# => We should have some kind of "per robot" configuration that allows a robot to provide one task but not another...
#    And this should probably be independent of how the broker handles the transmission of these tasks
#    ie. what a robot can do, should not be related to queues, but to simple robot configuration
#    => Maybe task imports ? but sender needs all, and all robots could potentially send...
