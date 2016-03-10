#
#  sample celeros configuration.
# this file can be overloaded and passed as argument to change the celeros configuration
#
# TODO : move to a cleaner way to do configuration ( probably after mutating into up a pure python package )

from kombu import Queue
import celeros

#
# These should probably be changed, depending on your celeros deployment
#
DEBUG = False
TESTING = False

# NOTE : useful only for sending task. worker get URL from cmd line args
CELERY_BROKER_URL = 'redis://localhost:6379/0'
BROKER_URL = CELERY_BROKER_URL
# NOTE : used for getting results, and also aborting tasks
CELERY_RESULT_BACKEND = 'redis://localhost:6379/1'

# config used by beat for scheduler
CELERY_REDIS_SCHEDULER_URL = "redis://localhost:6379/2"

CELERY_IMPORTS = ('celeros.rostasks', )

CELERYBEAT_SCHEDULER = 'celeros.RedisScheduler'

#
# These are assume to always stay the same by celeros and should not be changed
#
CELERY_ACCEPT_CONTENT = ['application/json']
CELERY_TASK_SERIALIZER = 'json'
CELERY_RESULT_SERIALIZER = 'json'

CELERY_CREATE_MISSING_QUEUES = True  # We create queue that do not exist yet.
CELERY_ALWAYS_EAGER = False  # We do NOT want the task to be executed immediately locally.
CELERY_TRACK_STARTED = True  # We want to know when the task are started

# config used by beat for scheduler
CELERY_REDIS_SCHEDULER_KEY_PREFIX = 'schedule:'
CELERYBEAT_SYNC_EVERY = 1
CELERYBEAT_MAX_LOOP_INTERVAL = 30

# each worker will accept only a single task at a time
CELERYD_CONCURRENCY = 1  # only need one worker process.
CELERYD_PREFETCH_MULTIPLIER = 1  # 0 means no limit.
CELERY_ACKS_LATE = True

# Force each worker to have its own queue. So we can send specific task to each.
CELERY_WORKER_DIRECT = True

# Routes are ony important for task sender : we do not care about simulation or not here.
CELERY_DEFAULT_QUEUE = 'celeros'
CELEROS_ROUTES = {
    'celeros.app.add_together': {'queue': 'celeros'},
    'celeros.app.long_task': {'queue': 'celeros'},
}

# Extending celery router
# celeros router automatically prepend "simulated." if the task run is intended to be simulated.
CELERY_ROUTES = (celeros.Router(CELEROS_ROUTES, CELERY_DEFAULT_QUEUE), )

#
# Custom Celeros settings:
#

# TODO : think about how to specify topic OR service...
# Specifying where we should get the battery information from.
# None means we dont want to care about it.
CELEROS_BATTERY_TOPIC = '/robot/battery'
# TODO : regex here (matching the local hostname instead of "robot"?) ?

# (List of) tuples of battery levels and queues that can be consumed from,
# only if the battery has a percentage higher than the specified level
CELEROS_MIN_BATTERY_PCT_QUEUE = [
    (20, 'celeros')
]
# This will automatically create the required queues. No need to specify CELERY_QUEUES here.
