from setuptools import setup

package_name = 'tb3_rnftsmc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='tom',
    author_email='tom@example.com',
    description='RNFTSMC controller for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rnftsmc_controller = tb3_rnftsmc.controller:main',
        ],
    },
)