from setuptools import setup

setup(name='celeros',
    version='0.1.0',
    description='Celery as a scheduler for ROS systems',
    url='http://github.com/asmodehn/celeros',
    author='AlexV',
    author_email='asmodehn@gmail.com',
    license='BSD',
    packages=[
        'celeros',
        'celeros.bootsteps',
        'celeros.celerybeatredis',
        'celery', 'celery.app', 'celery.apps', 'celery.backends', 'celery.backends.database', 'celery.bin',
        'celery.concurrency', 'celery.contrib', 'celery.events', 'celery.fixups', 'celery.loaders', 'celery.security',
        'celery.task', 'celery.utils', 'celery.utils.dispatch', 'celery.worker',
        'kombu', 'kombu.async', 'kombu.transport', 'kombu.transport.sqlalchemy', 'kombu.transport.virtual',
        'kombu.utils',
        'billiard', 'billiard.dummy', 'billiard.py2', 'billiard.py3',
        'flower', 'flower.api', 'flower.utils', 'flower.utils.backports', 'flower.views',
        'tornado_cors',
        'redis',
    ],
    package_dir={
        'celeros': 'celeros',
        'celery': 'deps/celery/celery',
        'kombu': 'deps/kombu/kombu',
        'billiard': 'deps/billiard/billiard',
        'flower': 'deps/flower/flower',
        'tornado_cors': 'deps/tornado-cors/tornado_cors',
        'redis': 'deps/redis/redis',
    },
    package_data={
        'flower': ['templates/*', 'static/**/*', 'static/*.*']
    },
    # TODO : config files install via data_files. careful : https://bitbucket.org/pypa/setuptools/issues/130
    # or maybe move to wheel ?
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        'pyros>=0.2.0',
        'click',
    ],
    zip_safe=False,  # TODO testing...
)
