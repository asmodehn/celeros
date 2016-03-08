from __future__ import absolute_import

# Abstract Task / Queue / Worker vs Actual Job / Job type / Robot
# How to match these :
# - Task is code, and should not be changed when installing.
# - Worker is configuration and can be changed when installing.
# - Queue is dependent on application and environment and should be changed when installing.
#
# * robot <=> worker, ie : 1 and only one worker per robot ( no point to run concurrent tasks on robot )
# * job type <=> task, ie : 1 and only one task per job type (and one job type per task)
#   -> this is how we will configure the job type ( for now at least )
# * Queue <-> Robot ?  We know when sending which robot will get it
#   -> predictible behavior from beat -> scheduler can make final decision with all information.
# * Queue <-> Job Type ? We can dynamically reassign robot from one job type to another...

from celery.app.routes import MapRoute


# This is celeros default router.
# Note that setting the queue manually when starting the task will override its decision here.
# cf. http://docs.celeryproject.org/en/latest/userguide/routing.html#specifying-task-destination
class Router(MapRoute):
    # WARNING : we dont have access to app here, since any client (even not worker)
    #  can send task and would need to route them...
    # This router allows to send a request to simulate any task to another queue, where your simulators can grab it.

    # No simple way to get the map from celery,
    # Done in configuration for now...
    def __init__(self, map):
        print("Setting up default routes from {map}".format(map=map))
        super(Router, self).__init__(map)

    # NOTE : celery logger doesnt seem to work(no output) here
    def route_for_task(self, task, args=None, kwargs=None):
        """
        Router function. Basic example that can easily be extended.
        We use an extra task argument to determine which queue ( based on configuration )
        the task is going to be routed to. This means that the caller package doesn't need to know the queue layout.
        The router here will import the configuration and figure it out.
        BEWARE : we do not have access to the app here... any client could instantiate the router (command line, flower, etc.)
        """
        try:  # To log all exceptions and avoid passing them silently in caller

            # Getting the usual route
            res = super(Router, self).route_for_task(task, args=args, kwargs=kwargs)

            # if simulated run, we prepend "simulated." string to reroute the task
            # WARNING : passing this as a part of args will not be detected here !
            # TODO : use options https://github.com/celery/celery/pull/2217
            if not kwargs.get("simulated", False):
                print("Task -> Routing to {defq}".format(defq=res))
                return res
            else:
                res['queue'] = 'simulated.' + res.get('queue', '')
                print("Simulated Task -> Routing to {simq}".format(simq=res))
                return res

        except Exception as exc:
            print("Exception in Router : {0}".format(exc))
