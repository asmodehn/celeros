#!/usr/bin/python
# All ways to run celeros
# should be defined here for consistency
from __future__ import absolute_import
import os
import sys
import click
import errno
import celery
import celery.bin.worker

#importing current package if needed ( solving relative package import from __main__ problem )
if __package__ is None:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from celeros import celeros_app
else:
    from . import celeros_app

import logging
from logging.handlers import RotatingFileHandler

# Note to keep things simple and somewhat consistent, we try to follow rostful's __main__ structure here...

# TODO : handle ros arguments here
# http://click.pocoo.org/5/commands/#group-invocation-without-command
@click.group()
def cli():
    pass

#
# Arguments' default value is None here
# to use default values from config file if one is provided.
# If no config file is provided, internal defaults are used.
#
@cli.command()
@click.option('hostname', '-n', '--hostname', default=None)
@click.option('broker', '-b', '--broker', default=None)
@click.option('beat', '-B', '--beat', is_flag=True, default=False)
@click.option('loglevel', '-l', '--loglevel', default=None)
@click.option('logfile', '-f', '--logfile', default=None)
@click.option('scheduler', '-S', '--schedulerclass', default=None)
@click.option('queues', '-Q', '--queues', default=None)
@click.option('config', '-c', '--config', default=None)  # this is the last possible config override, and has to be explicit.
@click.option('ros_args', '-r', '--ros-arg', multiple=True, default='')
def worker(hostname, broker, beat, loglevel, logfile, queues, scheduler, config, ros_args):
    """
    Starts a celeros worker
    :param hostname: hostname for this worker
    :param broker: broker to connect to
    :param loglevel: loglevel
    :param logfile: logfile to dump logs
    :param queues: queues to connect to
    :param config: config file to use (last overload)
    :param ros_args: extra ros args
    :return:
    """

    # Massaging argv to make celery happy
    argv = []
    if beat:
        argv += ['--beat']
    if hostname:
        argv += ['--hostname={0}'.format(hostname)]
    if broker:
        argv += ['--broker={0}'.format(broker)]
    if queues:
        argv += ['--queues={0}'.format(queues)]
    if scheduler:
        argv += ['--scheduler={0}'.format(scheduler)]
    if loglevel:
        argv += ['--loglevel={0}'.format(loglevel)]
    if logfile:
        argv += ['--logfile={0}'.format(logfile)]
    if config:
        argv += ['--config={0}'.format(config)]

    for r in ros_args:
        argv += ['--ros-arg={0}'.format(r)]

    celeros_app.worker_main(argv=argv)


# TODO : inspect command wrapper here to simplify usage to our usecase.


if __name__ == '__main__':
    cli()
