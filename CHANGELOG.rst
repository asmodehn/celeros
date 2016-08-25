^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package celeros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2016-08-25)
------------------
* setting celeros.log location to gopher folder
  It seems this setting cannot be overridden by another configuration from a child project...
* Merge pull request `#34 <https://github.com/asmodehn/celeros/issues/34>`_ from asmodehn/indigo-devel
  Applying log patch for better gopher logging
* cleanup debug logs.
* monkey patching logging system to be able to log to file AND to console.
* quick fix : forcing global celery log in a specific file.
* now assuming battery message contain the standard sensor_state msg
* fixing celeros rostest
* informative comments for tornado dependency version issue...
* bumping to get last_task_id. cosmetics.
* exposing datetime encoder and decoder. sometime useful to have.
* now flower launch logging into roslog assigned file. cosmetics
* cosmetics and comments
* fix setup.py for installing bootsteps subpackage.
* fixing test run after renaming launcher
* Merge pull request `#28 <https://github.com/asmodehn/celeros/issues/28>`_ from asmodehn/no_router
  simplified routing. cosmetics.
* bumping celerbeatredis to get editable custom field for fire_and_forget.
  cosmetics
* simplified routing. cosmetics.
* Merge pull request `#27 <https://github.com/asmodehn/celeros/issues/27>`_ from asmodehn/ghosts
  Ghosts
* bumping celerybeatredis
* Merge branch 'indigo-devel' of https://github.com/asmodehn/celeros into ghosts
  Conflicts:
  celeros/config.py
* bumping celerybeatredis to get log cleanup
* Cleaning up time tracking related code : already done in flower.
  Now sending sent event by default.
  bumping celerybeatredis.
* now tracking sent event, needed to get the sent time from the task.
* improving battery check and fixing routing to simulated queue
* fixing battery consumer bootstep to reuse queues if possible and avoid conflicts in settings.
* Merge pull request `#26 <https://github.com/asmodehn/celeros/issues/26>`_ from asmodehn/queue_inspection
  bumping up celerybeatreadis to include refactor of scheduler
* fire_and_forget first implementation.
* Merge branch 'queue_inspection' of https://github.com/asmodehn/celeros into ghosts
* bumping up celerybeatreadis to include refactor of scheduler
* Merge pull request `#25 <https://github.com/asmodehn/celeros/issues/25>`_ from asmodehn/refactor_cmaketest
  getting rid of extra cmakelists.txt in tests
* many improvements for configuration and battery check in worker.
  Also especially working with new refactored celerybeatredis. still WIP.
* not using solo pool : cannot revoke/terminate jobs (not implemented).
* Added batterywatcher to require a minimum battery level before listening on some queues.
  refactored bootsteps, now in subpackage.
  merged launchers.
  added customization files for eventual modification later.
* Merge pull request `#24 <https://github.com/asmodehn/celeros/issues/24>`_ from asmodehn/revoke_on_shutdown
  Revoke on shutdown
* cleanup
* revoking and terminating task on shutdown.
* getting rid of extra cmakelists.txt in tests
* cosmetics
* Merge branch 'indigo' of https://github.com/asmodehn/celeros into indigo-devel
* Merge pull request `#19 <https://github.com/asmodehn/celeros/issues/19>`_ from asmodehn/fix_configs
  reviewing config setup. '--config' command arg is now mandatory.
* comments
* adding cmake install rule for config files.
* adding redis client dependency, updating beat redis scheduler
* reviewing config setup. '--config' command arg is now mandatory.
  updating celery and dependencies.
* bumping flower to latest celeros forked version to fix `#18 <https://github.com/asmodehn/celeros/issues/18>`_
* comments and log message fix.
* bumping flower dep to get latest abort task capability
* adding python-babel as ros dependency since flower needs it
  Conflicts:
  package.xml
* adding python-babel as ros dependency since flower needs it
* Merge pull request `#17 <https://github.com/asmodehn/celeros/issues/17>`_ from asmodehn/pyros_fixes
  Pyros fixes
* fixing celeros test
* Merge branch 'indigo-devel' into pyros_fixes
* WIP. cosmetics and small test improvements.
* Merge pull request `#15 <https://github.com/asmodehn/celeros/issues/15>`_ from asmodehn/pyros_fixes
  Pyros fixes
* adding missing dependency of celery.
* adding cache params for worker.
* updated tests.
* adding missing test dependency on std_srvs
* Merge branch 'pyros_fixes' of https://github.com/asmodehn/celeros into pyros_fixes
  Conflicts:
  celeros/rosstart.py
* celeros now passing basepath to pyros context dynamically.
* celeros now passing basepath to pyros context dynamically.
* Contributors: AlexV, Daniel Stonier, alexv

0.0.3 (2016-01-28)
------------------
* Merge tag '0.0.3' into indigo
* Merge pull request `#14 <https://github.com/asmodehn/celeros/issues/14>`_ from asmodehn/examples
  Examples
* migrating to new pyros 0.1
* Merge branch 'indigo-devel' of https://github.com/asmodehn/celeros into examples
* Renaming rostful-node repo
* Merge pull request `#13 <https://github.com/asmodehn/celeros/issues/13>`_ from asmodehn/subdeps
  Subdeps
* Merge pull request `#12 <https://github.com/asmodehn/celeros/issues/12>`_ from asmodehn/indigo-devel
  updating indigo before dependency refactoring
* switching flower branch and grabbing tornado-cors
* adding flask celery helper as dependency
* Merge pull request `#9 <https://github.com/asmodehn/celeros/issues/9>`_ from asmodehn/travis
  Travis
* adding travis badge
* integrating with travis
* adding flower as dependency for celeros
* small tab index change
* splitting main window and turtle widget.
* first version of QT UI for turtle command center
* adding basic structure for examples. not working yet
* adding rosinstall files for wstool
* adding billiard as submodule
* adding kombu as submodule
* adding celery as a submodule
* Contributors: AlexV, alexv

0.0.2 (2015-10-12)
------------------
* bump submodule with scheduler fixes
* worker only gets a single task at a time, can specify the loglevel
* bumping submodule to get more logs
* allowing to pass queue names as roslaunch argument
* adding max loop interval to config.
* bumping submodule to not dies on json spelling mistake
* Merge branch 'redis-scheduler' of https://github.com/asmodehn/celeros into redis-scheduler
  Conflicts:
  deps/celerybeatredis
* bumping submodul to get simpler task scheduling
* add prefetch multiplier to config to limit task prefetching to one
* updated config file to prevent tasks being added to queue repeatedly
  if this value is not set, the synchronisation with the redis server happens only
  every three minutes, and results in any task after the first which should have
  run being repeatedly sent to the queue.
* udpated submodule
* making scheduler loop interval msall (5 secs) instead of 5 mins.
* Merge remote-tracking branch 'origin/redis-scheduler' into redis-scheduler
* adding debug output for beat scheduler.
* Merge remote-tracking branch 'origin/redis-scheduler' into redis-scheduler
* removing now obsolete celeros python script.
* Merge remote-tracking branch 'origin/redis-scheduler' into redis-scheduler
* improving celeros worker parameters and listening to its private queue.
* fixing typo
* Merge branch 'envhook' of https://github.com/asmodehn/celeros into redis-scheduler
* Merge branch 'indigo-devel' into redis-scheduler
  Conflicts:
  .gitmodules
* adding missing .gitmodules files
* env var as default hostname
* fixing scheduler to be able to use as client to write in DB as well
* fixing tasks. removed duplicates.
* fixing indent after merge
* Merge branch 'indigo-devel' of https://github.com/asmodehn/celeros into redis-scheduler
  Conflicts:
  celeros/app.py
  celeros/config.py
  celeros/rostasks.py
  celeros/worker.py
  launch/worker.launch
* Merge branch 'indigo-devel' of https://github.com/asmodehn/celeros into envhook
* adding env-hook to grab hostname as default parameter. untested.
* Merge remote-tracking branch 'origin/subproc' into subproc
* fixing tests and cleanups.
* fixed worker bash script hostname switch
* Merge branch 'indigo-devel' of https://github.com/asmodehn/celeros into subproc
  Conflicts:
  celeros/__main_\_.py
  celeros/app.py
  celeros/config.py
  celeros/worker.py
  launch/worker.launch
  scripts/worker
* Fixed task calls to client node, tests (mostly) working
* Merge branch 'indigo-devel' of https://github.com/asmodehn/celeros into indigo-devel
* WIP : implementing test for celeros with subprocess rostful_node
* now using celerybeatredis scheduler from celery beat.
* fixing config and arguments handling problem introduced by the hostname parameter.
* can now specify hostname for robot in worker launch
* Refactored after merge. Now using rostful_node_process
  Merge branch 'launch-reconf' of https://github.com/asmodehn/celeros into indigo-devel
  Conflicts:
  celeros/__main_\_.py
  launch/worker.launch
* Merge branch 'indigo' of https://github.com/asmodehn/celeros into indigo-devel
* adding tests. work in progress. on hold because the worker python code is kind of blocking us here.
* adding celerybeat-redis as submodule
* config cleanup
* Merge pull request `#4 <https://github.com/asmodehn/celeros/issues/4>`_ from asmodehn/celeryconfig-launch
  added launch file parameter for setting celery configuration
* Merge pull request `#1 <https://github.com/asmodehn/celeros/issues/1>`_ from gitter-badger/gitter-badge
  Add a Gitter chat badge to README.rst
* added launch file parameter for setting celery configuration
* starting on reconfiguring launch for fewer steps and faster shutdown
* exposing port and address ffor flower through roslaunch
* fixing install rule
* fixing ros arguments handling.
* keep rostful_node alive until celery worker finishes.
  more fixes to make celeros work for gopher_rocon.
  cosmetics
* Merge branch 'indigo-devel' of https://github.com/asmodehn/celeros into indigo-devel
* passing node name to rostful_node to avoid conflict with other instances
* Added Gitter badge
* fixing flower launch file and script
* first version of a working celeros. extracted from rostful.
* Initial commit
* Contributors: AlexV, Michal Staniaszek, The Gitter Badger, alexv
