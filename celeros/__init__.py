### This package contains a Worker ( instanciated once here to allow easy setup procedure, but not started )
from __future__ import absolute_import

from .worker import celeros_worker
from . import app as celeros_tasks

__all__ = [
    'celeros_worker',
    'celeros_tasks'
]
