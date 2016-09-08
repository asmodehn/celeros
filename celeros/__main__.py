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


# logging configuration should be here to not be imported by users of celeros.
# only used from command line

import logging.config
# Setting up logging for this test
logging.config.dictConfig(
    {
        'version': 1,
        'formatters': {
            'verbose': {
                'format': '%(levelname)s %(asctime)s %(module)s %(process)d %(thread)d %(message)s'
            },
            'simple': {
                'format': '%(levelname)s %(name)s:%(message)s'
            },
        },
        'handlers': {  # Handlers not filtering level -> we filter from logger.
            'null': {
                'level': 'DEBUG',
                'class': 'logging.NullHandler',
            },
            'console': {
                'level': 'DEBUG',
                'class': 'logging.StreamHandler',
                'formatter': 'simple'
            },
        },
        'loggers': {
            'pyros_config': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup': {
                'handlers': ['console'],
                'level': 'INFO',
            },
            'pyros': {
                'handlers': ['console'],
                'level': 'INFO',
            },
            # Needed to see celery processes log (especially beat)
            'celery': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'celeros': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            '': {  # root logger
                'handlers': ['console'],
                'level': 'INFO',
            }

        }
    }
)



#importing current package if needed ( solving relative package import from __main__ problem )
if __package__ is None:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from celeros import celeros_app
else:
    from . import celeros_app


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
    # First arg needs to be the prog_name (following schema "celery prog_name --options")
    argv = ['worker']
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

    logging.info("Starting celery with arguments {argv}".format(**locals()))
    celeros_app.worker_main(argv=argv)

# TODO : inspect command wrapper here to simplify usage to our usecase.


if __name__ == '__main__':
    cli()
