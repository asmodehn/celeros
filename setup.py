# This setup is usable by catkin, or on its own as usual python setup.py

_CATKIN = False
try:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup
    _CATKIN = True
except Exception, e:
    from setuptools import setup

# CAREFUL distutils and setuptools take different argument sand have different behaviors
if _CATKIN:  # using distutils : https://docs.python.org/2/distutils
    # fetch values from package.xml
    setup_args = generate_distutils_setup(
        packages=[
            'celeros',
            'celeros.celerybeatredis',
            'celery', 'celery.app', 'celery.apps', 'celery.backends', 'celery.backends.database', 'celery.bin', 'celery.concurrency', 'celery.contrib', 'celery.events', 'celery.fixups', 'celery.loaders', 'celery.security', 'celery.task', 'celery.utils', 'celery.utils.dispatch', 'celery.worker',

        ],
        package_dir={
            'celeros': 'celeros',
            'celery': 'deps/celery/celery',
        },
    )
    setup(**setup_args)

else:  # using setuptools : http://pythonhosted.org/setuptools/

    setup(name='celeros',
        version='0.0.1',
        description='Celery as a scheduler for ROS systems',
        url='http://github.com/asmodehn/celeros',
        author='AlexV',
        author_email='asmodehn@gmail.com',
        license='BSD',
        packages=[
            'celeros',
            'celeros.celerybeatredis',
            'celery', 'celery.app', 'celery.apps', 'celery.backends', 'celery.backends.database', 'celery.bin', 'celery.concurrency', 'celery.contrib', 'celery.events', 'celery.fixups', 'celery.loaders', 'celery.security', 'celery.task', 'celery.utils', 'celery.utils.dispatch', 'celery.worker',
        ],
        package_dir={
            'celeros': 'celeros',
            'celery': 'deps/celery/celery',
        },
        # this is better than using package data ( since behavior is a bit different from distutils... )
        include_package_data=True,  # use MANIFEST.in during install.
        install_requires=[  # External dependencies only. the ones that match package.xml. others are included in here already.
        ],
        zip_safe=False,  # TODO testing...
    )
