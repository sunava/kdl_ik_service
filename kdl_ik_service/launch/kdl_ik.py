from doctest import debug

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, LocalSubstitution, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # debug = LaunchConfiguration('debug')
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='200'
    )
    ik_node = Node(
            package='kdl_ik_service',
            executable='start_ros_server',
            name='kdl_ik_service',
            output='screen',
            arguments=[],
            # parameters=[{'ik_debug': debug_arg}],
        )
    return LaunchDescription([
        debug_arg,
        ik_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ik_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                                 ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                         LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])