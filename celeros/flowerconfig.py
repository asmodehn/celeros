import ast
import json

from flower.utils.template import humanize

# Enable debug logging
logging = 'DEBUG'


# This should parse and change to json the args and kwargs
# TODO : doesnt seem to do anything. fix it !
def format_task(task):
    print (task)
    task.args = json.dumps(task.args)
    task.kwargs = json.dumps(task.kwargs)
    return task
