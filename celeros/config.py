DEBUG = False
TESTING = False

# NOTE : useful only for sending task. worker get URL from cmd line args
BROKER_URL = 'redis://localhost:6379'
CELERY_RESULT_BACKEND = BROKER_URL
CELERY_ACCEPT_CONTENT = ['application/json']
CELERY_TASK_SERIALIZER = 'json'
CELERY_RESULT_SERIALIZER = 'json'

CELERY_TRACK_STARTED = True  # we want to know when the task has started
CELERY_ALWAYS_EAGER = False  # FOR NOW : Always put into the queue
# TODO : ?maybe? use True to match rapp/task behavior and start locally if possible, otherwise push into queue...

CELERY_REDIS_SCHEDULER_URL = "redis://localhost:6379/2"
CELERY_REDIS_SCHEDULER_KEY_PREFIX = 'schedule:'
CELERYBEAT_SYNC_EVERY = 1
CELERYBEAT_MAX_LOOP_INTERVAL = 5



