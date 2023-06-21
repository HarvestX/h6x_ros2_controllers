# Copyright 2023 HarvestX Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_path
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    robot_description_content = """
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <ros2_control name="Battery" type="sensor">
        <hardware>
            <plugin>h6x_ros2_controller_example/FakeBatteryInterface</plugin>
        </hardware>
        <sensor name="battery_sensor">
            <state_interface name="voltage"/>
            <state_interface name="temperature"/>
            <state_interface name="current"/>
            <state_interface name="charge"/>
            <state_interface name="capacity"/>
            <state_interface name="percentage"/>
        </sensor>
    </ros2_control>

    <link name="base_link" />
    <link name="link1" />
    <joint name="base_joint">
        <parent link="base_link"/>
        <child link="link1"/>
    </joint>
</robot>
    """

    param = get_package_share_path('h6x_ros2_controller_example') / \
        'config' / 'fake_battery_controller.yaml'

    bsbc_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['battery_state_broadcaster', '-c', '/controller_manager'])

    ctrl_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            str(param),
            {'robot_description': robot_description_content}
        ])

    ld = LaunchDescription()

    ld.add_action(bsbc_spawner_node)
    ld.add_action(ctrl_manager_node)

    return ld
