# Copyright 2026 Juan S. Cely G.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import find_packages
from setuptools import setup

package_name = 'roboligo_cli'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Juan S. Cely G.',
    author_email='juanscelyg@gmail.com',
    maintainer='Juan S. Cely G.',
    maintainer_email='juanscelyg@gmail.com',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Programming Language :: Python',
    ],
    description='roboligo cli: roboligo cli tools to manage robots',
    long_description="""\
roboligo cli: roboligo cli tools to manage robots.""",
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'roboligo = roboligo_cli.command.roboligo:RoboligoCommand',
        ],
        'roboligo_cli.verb': [
            'arming = roboligo_cli.verb.arming:ArmingVerb',
            'disarming = roboligo_cli.verb.disarming:DisarmingVerb',
            'landing = roboligo_cli.verb.landing:LandingVerb',
            'offboard = roboligo_cli.verb.offboard:OffboardVerb',
            'takeoff = roboligo_cli.verb.takeoff:TakeoffVerb',
            'standby = roboligo_cli.verb.standby:StandbyVerb',
        ],
    }
)
