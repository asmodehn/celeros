from __future__ import absolute_import

from . import config
from celery import Celery


celeros_app = Celery()
celeros_app.config_from_object(config.Development)
