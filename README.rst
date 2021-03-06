.. image:: https://travis-ci.org/asmodehn/celeros.svg?branch=indigo-devel
    :target: https://travis-ci.org/asmodehn/celeros

# celeros
Celery ROS python interface

WARNING : This project is DEPRECATED. Use at your own peril.
The point here was to send tasks to robots. However Celery :

- likes stable network connection (which we don't have since robot can leave network anytime, for long periods)
- idempotent tasks (which we cannot have since the point of a robot is to interact with the real world)
- is too complex for custom modifications that are needed in a unstable robotic environment.

Next Candidate for multi robot cooperation : Custom REST based Passive Scheduler.
Robot should get free of the server and is just a web client like any human.


.. image:: https://badges.gitter.im/Join%20Chat.svg
   :alt: Join the chat at https://gitter.im/asmodehn/celeros
   :target: https://gitter.im/asmodehn/celeros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge

This repository has a few main branches:

- master : main branch, python dev workflow, releasing version tags into a pip package.
- indigo-devel : current indigo-based ongoing development. catkin dev workflow.
- indigo : current indigo-based release (ROS pkg - tags attempting to match)
- <ros_distro> : current <ros_distro>-based release (ROS pkg)

Apart from these we follow a `feature branching workflow <https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow>`_

WARNING: This repository structure is currently being implemented...

Celeros is a repurposing of the celery distributed task manager with ROS systems.
With celeros you can send tasks to multiple robots, in the same way as you would send task to a cluster of servers.
Tasks can be scheduled in time, routed to different priority queues, from which robots get the next tasks to execute.

We aim to retain the full customizable structure of celery.
However, given that robots are servers, but also quite different at the same time, the default behavior that users and developers expect for a scheduler is quite different.
Celeros already integrate these "default" behavior specially designed for robots.