#
#  sample celeros configuration.
# this file can be overloaded and passed as argument to change the celeros configuration
#
# TODO : move to a cleaner way to do configuration ( probably after mutating into up a pure python package )

from kombu import Queue

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
CELERY_SEND_TASK_SENT_EVENT = True  # needed to have the sent time (exposed by flower)

# config used by beat for scheduler
CELERY_REDIS_SCHEDULER_KEY_PREFIX = 'schedule:'
CELERYBEAT_SYNC_EVERY = 1
CELERYBEAT_MAX_LOOP_INTERVAL = 30

# each worker will accept only a single task at a time
CELERYD_CONCURRENCY = 1  # only need one worker process.
CELERYD_PREFETCH_MULTIPLIER = 1  # 0 means no limit.
CELERY_ACKS_LATE = True  # we ack when the task is finished. only then the next task will be fetched.

# Force each worker to have its own queue. So we can send specific task to each.
CELERY_WORKER_DIRECT = True

# Routes are only important for task sender : we do not care about simulation or not here.
CELERY_DEFAULT_QUEUE = 'celeros'
CELERY_DEFAULT_EXCHANGE = 'celeros'
CELERY_DEFAULT_EXCHANGE_TYPE = 'direct'  # topic doesnt seem to work with redis at the moment ( https://github.com/celery/celery/issues/3117 )
CELERY_DEFAULT_ROUTING_KEY = 'celeros'

# Queueing strategy : matching routing_key determine which queue the task will go in
# default routing key is the task name. a matching queue will be determined by celery.
CELERY_QUEUES = (
)
# Note the battery sensitive queue should not be put here
# as it would automatically start consume before we are able to do battery check

CELERY_ROUTES = ({
    'celeros.app.add_together': {
        'queue': 'celeros'
    },
    'celeros.app.long_task': {
        'queue': 'celeros'
    },
})
# Note : some routes will need to be setup here for simulated task to go to simulated queues.
# Hint : Celery 3.1 Router doesnt get options passed to it,
#  so there is no convenient way to route a task run based on "task run intent".
#  The route can depend only on the task name itself
#  -> simulated task should duplicate normal task, with only a different name, that will branch to a different route.

#
# Custom Celeros settings:
#

# TODO : think about how to specify topic OR service...
# Specifying where we should get the battery information from.
# None means we dont want to care about it.
CELEROS_BATTERY_TOPIC = '/robot/battery'
# TODO : regex here (matching the local hostname instead of "robot"?) ?

# how often we check the battery (in secs)
CELEROS_BATTERY_CHECK_PERIOD = 60

# (List of) tuples of battery levels and queues that can be consumed from,
# only if the battery has a percentage higher than the specified level
CELEROS_MIN_BATTERY_PCT_QUEUE = [
    (20, Queue('celeros',    routing_key='celeros'))
]
# This will automatically create the required queues if needed.
# But we need to specify CELERY_QUEUES here in order to set the routing key for it.
