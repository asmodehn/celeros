import time
from .app import celeros_app
from celery import Task
from celery.contrib.abortable import AbortableTask

from celery.utils.log import get_task_logger

_logger = get_task_logger(__name__)

#import required ros modules

import inspect

import unicodedata
import json
from rosinterface import message_conversion as msgconv
from rosinterface import ActionBack
import datetime
import random

import rospy
import rostful_node
from importlib import import_module

from celery.utils.log import get_task_logger

_logger = get_task_logger(__name__)


# TODO : we should probably move these to rostful-node as shared tasks...
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

@celeros_app.task(bind=True, base=AbortableTask)
def action(self, action_name, **kwargs):
#TODO : fix this, somehow... we should probably use the actionlib Client somwhere...

    #interfacing celery task with ros action. both are supposed to represent a long running async job.

    #NOTE : kwargs can contain multiple goals (?) or one action task is one goal... more goals mean more action tasks ?

    # because Actions ( interpreted from roslib.js ) and because we don't want to know about the action msg type here
    # use the task ID as a goal ID
    goalID = 'goal_' + str(self.request.id)

    if not self.is_aborted():  # to make sure we didn't abort before it starts...
        res = self.app.ros_node_client.action_goal(action_name, goalID, **kwargs)

        # get full goalID
        if 'goal_id' in res.keys():
            goalID = res['goal_id']

        polling_period = 2.0
        while not self.is_aborted():

            # watch goal and feedback
            #TODO : estimate progression ? what if multiple goals ?
            res = self.app.ros_node_client.action_goal(action_name, goalID)

            if res and res.get('goal_status', {}) and res.get('goal_status', {}).get('status') in [0, 1, 6, 7]:
                self.update_state(
                    state='FEEDBACK',
                    meta={'rostful_data': res,}
                )
            #detect action end and match celery status
            elif res and res.get('goal_status', {}) and res.get('goal_status', {}).get('status', {}) in [4, 5, 8]:
                #TODO : set state and info properly
                self.update_state(
                    state=celeros_app.states.FAILURE,
                    meta={'rostful_data': res,}
                )
                return
            elif res and res.get('goal_status', {}) and res.get('goal_status', {}).get('status', {}) in [2, 3]:
                res = self.app.ros_node_client.action_result(action_name, goalID)
                self.update_state(
                    state=celeros_app.states.SUCCESS,
                    meta={'rostful_data': res,}
                )


            time.sleep(polling_period)
        return res

    return {}  # unhandled status ??


@celeros_app.task(bind=True, base=AbortableTask)
def rocon_app(self, rapp_name, **kwargs):

    rospy.wait_for_service('~start_rapp')
    try:
        call_service = rospy.ServiceProxy('~start_rapp', rostful_node.srv.StartRapp)
        res_data = call_service(rapp_name, **kwargs)
        if res_data.started:


            time.sleep(42)

            call_service = rospy.ServiceProxy('~stop_rapp', rostful_node.srv.StopRapp)
            res_data = call_service()





        return {'data_json': res_data.data_json}
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

