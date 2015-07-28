# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os
import sys
import logging

try:
    import rostful_node
except Exception, e:
    print "rostful_node module is not accessible in sys.path. It is required to run celeros."
    print "Exception caught : ", e
    print "sys.path = %r", sys.path
    raise

from celery.bin import Option

from .rosargs import RosArgs
from .app import celeros_app


class Worker(object):
    # TODO : pass config file from command line here
    def __init__(self):

        # Setup Celery ( NOT the FLask Celery helper )
        # TODO : check SHARK http://sharq.io => how about double backend ? celery+flower or shark
        self.app = celeros_app

        # self.celery.user_options['worker'].add(
        #    Option("--ros_args", action="store", dest="ros_args", default=None, help="Activate support of rapps")
        # )
        # self.celery.steps['worker'].add(RosArgs)

    def _setup(self, ros_node, ros_node_client):
        self.ros_node = ros_node
        self.app.ros_node_client = ros_node_client

    def launch(self, broker_url='', tasks='', ros_args=''):

        print broker_url, tasks

        # changing broker ( needed even without worker running here )
        if broker_url != '':
            self.app.conf.update(
                CELERY_BROKER_URL=broker_url,
                CELERY_RESULT_BACKEND=broker_url,
            )

        # importing extra tasks ( needed even without worker running here )
        if tasks != '':
            self.app.conf.update(CELERY_IMPORTS=tasks)

        # One RostfulNode is needed for Flask.
        # TODO : check if still true with multiple web process
        with rostful_node.RostfulCtx(name='celeros', argv=ros_args) as node_ctx:
            self._setup(node_ctx.node, node_ctx.client)

            # Celery needs rostfulNode running, and uses it via python via an interprocess Pipe interface
            import threading
            # TODO : investigate a simpler way to start the (unique) worker asynchronously ?
            celeros_worker.worker_thread = threading.Thread(
                target=celeros_worker.app.worker_main,
                kwargs={'argv': [
                    'celery',
                    # '--app=celery',
                    # '--config=celery_cfg.Development',
                    '--events',
                    '--loglevel=INFO',
                    '--broker=' + broker_url,
                    '--concurrency=1',
                    '--autoreload',  # not working ??
                    # '--ros_args=' + ros_args
                ]}
            )
            celeros_worker.worker_thread.start()
            # TODO : fix signal handling when running celery in another thread...

            #we wait here to keep rostful node alive
            celeros_worker.worker_thread.join()



# Creating THE only instance of Worker.
celeros_worker = Worker()
