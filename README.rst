.. image:: https://travis-ci.org/asmodehn/celeros.svg?branch=indigo-devel
    :target: https://travis-ci.org/asmodehn/celeros

# celeros
Celery ROS python interface


.. image:: https://badges.gitter.im/Join%20Chat.svg
   :alt: Join the chat at https://gitter.im/asmodehn/celeros
   :target: https://gitter.im/asmodehn/celeros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge


Celeros is a repurposing of the celery distributed task manager with ROS systems.
With celeros you can send tasks to multiple robots, in the same way as you would send task to a cluster of servers.
Tasks can be scheduled in time, routed to different priority queues, from which robots get the next tasks to execute.

We aim to retain the full customizable structure of celery.
However, given that robots are servers, but also quite different at the same time, the default behavior that users and developers expect for a scheduler is quite different.
Celeros already integrate these "default" behavior specially designed for robots.