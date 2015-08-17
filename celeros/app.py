from __future__ import absolute_import

from celery import Celery
from celery.bin import Option
from .rosstart import BootRostfulNode
import sys

print("sys argv is ", sys.argv)
celeros_app = Celery()
celeros_app.steps['worker'].add(BootRostfulNode)
celeros_app.user_options['worker'].add(Option('-R', '--ros-args', default=None, help='Arguments for ros initialisation'))
