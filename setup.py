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
        'flower', 'flower.api', 'flower.utils', 'flower.utils.backports', 'flower.views',
    ],
    package_dir={
        'celeros': 'celeros',
        'flower': 'deps/flower/flower',
    },
    package_data={
        'flower': ['templates/*', 'static/**/*', 'static/*.*']
    },
    # TODO : config files install via data_files. careful : https://bitbucket.org/pypa/setuptools/issues/130
    # or maybe move to wheel ?
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        'celery==3.1.20',
        'pyros>=0.2.0',
        'click',
    ],
    zip_safe=False,  # TODO testing...
)
