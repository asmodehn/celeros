from __future__ import absolute_import

# The only way that works to import package content based on file structure
# instead of parent package content, which is not loaded when extending celery.
from .celerybeatredis import PeriodicTask as celerybeatredis_PeriodicTask


class PeriodicTask(celerybeatredis_PeriodicTask):
    pass

