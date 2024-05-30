from setuptools import setup

package_name = 'ackermann_vehicle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'ackermann_controller',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Twoje Imię',
    maintainer_email='twoj_email@example.com',
    description='Opis pakietu ackermann_vehicle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_controller = ackermann_controller:main',
        ],
    },
)
