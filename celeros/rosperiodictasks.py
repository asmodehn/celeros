from __future__ import absolute_import

# The only way that works to import package content based on file structure
# instead of parent package content, which is not loaded when extending celery.
from .celerybeatredis import PeriodicTask as celerybeatredis_PeriodicTask


class PeriodicTask(celerybeatredis_PeriodicTask):
    def __init__(self, name, task, schedule, enabled=True, fire_and_forget=False, args=(), kwargs=None, options=None,
                 last_run_at=None, total_run_count=None, **extrakwargs):

        super(PeriodicTask, self).__init__(name=name, task=task, enabled=enabled, fire_and_forget=fire_and_forget, last_run_at=last_run_at,
             total_run_count=total_run_count, schedule=schedule, args=args, kwargs=kwargs,
             options=options, **extrakwargs)

