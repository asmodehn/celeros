from __future__ import absolute_import

from . import config
from celery import Celery


app = Celery()
app.config_from_object(config.Development)
