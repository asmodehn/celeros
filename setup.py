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
    ],
    package_dir={
        'celeros': 'celeros',
    },
    # TODO : config files install via data_files. careful : https://bitbucket.org/pypa/setuptools/issues/130
    # or maybe move to wheel ?
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        'pyros>=0.1.0',
        'celery==3.1.20',
        'kombu==3.0.33',
        'billiard==3.3.0.22',
        'redis==2.10.5',
        'flower',
        'tornado-cors',
    ],
    zip_safe=False,  # TODO testing...
)
