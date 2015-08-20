from __future__ import absolute_import

from . import config
from celery import Celery
from celery.bin import Option
from .rosstart import BootRostfulNode
import sys

print("sys argv is ", sys.argv)
celeros_app = Celery()

# setting up default config
celeros_app.config_from_object(config)

# setting up custom bootstep to start ROS node and pass ROS arguments to it
celeros_app.steps['worker'].add(BootRostfulNode)
celeros_app.user_options['worker'].add(Option('-R', '--ros-arg', action="append", help='Arguments for ros initialisation'))

