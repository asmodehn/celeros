### This package contains a Worker ( instanciated once here to allow easy setup procedure, but not started )
from __future__ import absolute_import

from .worker import celeros_worker
from .app import celeros_app

__all__ = [
    'celeros_worker',
    'celeros_app'
]
