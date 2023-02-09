
# Copyright (c) 2023, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory

from robotnik_common.launch import RewrittenYaml

# Environment variables
#   NAMESPACE: Namespace for the nodes
#   NODE_NAME: Name of the node
#   FRAME_ID: Frame id of the imu
#   PORT: Port of the serial device

def read_params(ld : launch.LaunchDescription):
    environment = launch.substitutions.LaunchConfiguration('environment')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    node_name = launch.substitutions.LaunchConfiguration('node_name')
    frame_id = launch.substitutions.LaunchConfiguration('frame_id')
    port = launch.substitutions.LaunchConfiguration('port')
    
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='environment',
        description='Read params from environment variables.',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace for the nodes.',
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='node_name',
        description='Name of the node.',
        default_value='myahrs_imu')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='frame_id',
        description='Frame id of the imu.',
        default_value='imu_link')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='port',
        description='Port of the serial device.',
        default_value='/dev/ttyACM0')
    )

    ret = {}

    if environment == 'false':
        ret = {
        'namespace': namespace,
        'node_name': node_name,
        'frame_id': frame_id,
        'port': port,
        }
    else:
        if 'NAMESPACE' in os.environ:
            ret['namespace'] = os.environ['NAMESPACE']
        else: ret['namespace'] = namespace

        if 'NODE_NAME' in os.environ:
            ret['node_name'] = os.environ['NODE_NAME']
        else: ret['node_name'] = node_name

        if 'FRAME_ID' in os.environ:
            ret['frame_id'] = os.environ['FRAME_ID']
        else: ret['frame_id'] = frame_id

        if 'PORT' in os.environ:
            ret['port'] = os.environ['PORT']
        else: ret['port'] = port

    return ret


def generate_launch_description():
    ld = launch.LaunchDescription()
    
    config_dir = get_package_share_directory('myahrs_ros2_driver')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')

    params = read_params(ld)

    config_file_rewritten = RewrittenYaml(
        source_file=config_file,
        param_rewrites={
            'frame_id': params['frame_id'],
        },
        root_key=[params['namespace'], '/', params['node_name'],],
        convert_types=True
    )

    # Launch the myahrs driver
    ld.add_action(launch_ros.actions.PushRosNamespace(namespace=params['namespace']))
    ld.add_action(launch_ros.actions.Node(
        package='myahrs_ros2_driver',
        executable='myahrs_ros2_driver',
        name=params['node_name'],
        output='screen',
        arguments=[params['port'], '115200'],
        parameters=[config_file_rewritten]
    ))

    return ld
