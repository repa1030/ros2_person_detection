# ===============================================================================
# Copyright 2020 Hochschule Karlsruhe - Technik und Wirtschaft
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ===============================================================================
# Authors:  Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch, Patrick Rebling
# ===============================================================================

import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


parameters_file = 'test.yaml'


def generate_launch_description():

    parameters_file_path = str(pathlib.Path(__file__).parents[0])
    parameters_file_path += '/' + parameters_file
    return LaunchDescription([
        DeclareLaunchArgument(
            'venv',
            default_value='',
            description='Absolute path to virtualenv python interpreter.'
        ),
        DeclareLaunchArgument(
            'param_file',
            default_value=parameters_file_path,
            description='Absolute path to yaml parameter file.'
        ),
        Node(
            package='utilities',
            executable='image_publisher',
            output='screen',
            prefix=[LaunchConfiguration('venv')],
            parameters=[
                LaunchConfiguration('param_file')
            ]
        ),
        Node(
            package='detection_driver',
            executable='detection_driver',
            output='screen',
            prefix=[LaunchConfiguration('venv')],
            parameters=[
                LaunchConfiguration('param_file')
            ]
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            output='screen',
        )
    ])
