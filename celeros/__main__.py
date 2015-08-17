#!/usr/bin/python
# All ways to run rostful and all Manager commands
# should be defined here for consistency
from __future__ import absolute_import
import os
import sys
import click
import threading

#importing current package if needed ( solving relative package import from __main__ problem )
if __package__ is None:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from celeros import celeros_worker
    from celeros import celeros_app
else:
    from . import celeros_worker
    from . import celeros_app

import logging
from logging.handlers import RotatingFileHandler


@click.command()
@click.option('-b', '--broker-url', default='redis://localhost:6379')  # by default use config module value
@click.option('-t', '--worker-tasks', default='celeros.tasks')  # by default use config module value
@click.option('-c', '--worker-config', default='')  # by default use config module value
@click.option('ros_args', '-r', '--ros-arg', multiple=True, default='')
def work(broker_url='', worker_tasks='', worker_config='', ros_args=[]):

    logging.getLogger().setLevel(logging.DEBUG)

    logging.info('broker %r tasks %r config %r', broker_url, worker_tasks, worker_config)
    logging.info('ros_args %r', ros_args)
    str_rosargs = ','.join([str(arg) for arg in ros_args])  # remove unicode designation in strings

    # changing broker ( needed even without worker running here )
    if broker_url != '':
        celeros_app.conf.update(
            CELERY_BROKER_URL=broker_url,
            CELERY_RESULT_BACKEND=broker_url,
        )

    # importing extra tasks ( needed even without worker running here )
    if worker_tasks != '':
        celeros_app.conf.update(CELERY_IMPORTS=worker_tasks)
    
    worker_thread = threading.Thread(
        target=celeros_app.worker_main,
        kwargs={'argv': [
            'celery',
            # '--app=celery',
            # '--config=celery_cfg.Development',
            '--events',
            '--loglevel=INFO',
            '--broker=' + broker_url,
            '--concurrency=1',
            '--autoreload',  # not working ??
            '--ros-args=' + str_rosargs
        ]})

    worker_thread.start()
    worker_thread.join()
    
    #TODO : when called from python and no master found, do as roslaunch : create a master so it still can work from python
    #Launch the server
    #celeros_worker.launch(broker_url, worker_tasks, worker_config, list(ros_args))

if __name__ == '__main__':
    # change the ros arguments into a format readable by click
    sys.argv = map(lambda x : '--ros-arg=' + x if x[0:2] == '__' else x, sys.argv)
    work()
