import time
from .app import celeros_app
from celery import Task
from celery.contrib.abortable import AbortableTask

from celery.utils.log import get_task_logger

_logger = get_task_logger(__name__)


# TODO : fancy task decorators and behaviors for ROS-style tasks
# http://stackoverflow.com/questions/6393879/celery-task-and-customize-decorator

class RosTask(AbortableTask):

    def run(self, *args, **kwargs):
        """
        :param args: args of the task
        :param kwargs: kwargs of the tas.k
        :return:
        """
        # TODO : investigate dynamic wake up of pyros ?
        # TODO : investigate the task specifying which service / topics it needs to access (via decorator) ?
        # Would it be better (only using resources when actually needed) ?
        # or worse (dynamic everything makes things more fragile) ?
        _logger.info("Starting RosTask")
        return self.run(*args, **kwargs)

    def after_return(self, status, retval, task_id, args, kwargs, einfo):
        # exit point of the task whatever is the state
        _logger.info("Ending RosTask")
        pass


@celeros_app.task(bind=True)
def topic_inject(self, topic_name, _msg_content={}, **kwargs):
    _logger.info("Injecting {msg} {kwargs} into {topic}".format(msg=_msg_content, kwargs=kwargs, topic=topic_name))
    res = self.app.ros_node_client.topic_inject(topic_name, _msg_content, **kwargs)
    _logger.info("Result : {res}".format(res=res))
    return res


@celeros_app.task(bind=True)
def topic_extract(self, topic_name):
    _logger.info("Extracting from {topic}".format(topic=topic_name))
    res = self.app.ros_node_client.topic_extract(topic_name)
    _logger.info("Result : {res}".format(res=res))
    return res


@celeros_app.task(bind=True)
def service_call(self, service_name, _msg_content={}, **kwargs):
    _logger.info("Calling service {service} with {msg} {kwargs}".format(msg=_msg_content, kwargs=kwargs, service=service_name))
    res = self.app.ros_node_client.service_call(service_name, _msg_content, **kwargs)
    _logger.info("Result : {res}".format(res=res))
    return res

# TODO : implement something similar to actions.
# Like actions with topics
# AND with services only
# But without using action lib.
