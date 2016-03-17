#
#  sample celeros configuration.
# this file can be overloaded and passed as argument to change the celeros configuration
#
# TODO : move to a cleaner way to do configuration ( probably after mutating into up a pure python package )

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

#
# These are assume to always stay the same by celeros and should not be changed
#
CELERY_ACCEPT_CONTENT = ['application/json']
CELERY_TASK_SERIALIZER = 'json'
CELERY_RESULT_SERIALIZER = 'json'

CELERY_ALWAYS_EAGER = False  # We do NOT want the task to be executed immediately locally.
CELERY_TRACK_STARTED = True  # We want to know when the task are started
CELERY_SEND_TASK_SENT_EVENT = True  # needed to have the sent time (exposed by flower)

# config used by beat for scheduler
CELERY_REDIS_SCHEDULER_KEY_PREFIX = 'schedule:'
CELERYBEAT_SYNC_EVERY = 1
CELERYBEAT_MAX_LOOP_INTERVAL = 30

# each worker will accept only a single task at a time
CELERYD_CONCURRENCY = 1
CELERYD_PREFETCH_MULTIPLIER = 1
CELERY_ACKS_LATE = True

