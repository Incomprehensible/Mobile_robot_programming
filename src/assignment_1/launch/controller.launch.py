from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
def generate_launch_description():
    ld = LaunchDescription([
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("turtlebot3_gazebo"), '/launch', '/empty_world.launch.py'])
            )])
    control_node = Node(
        package="assignment_1",
        executable="control_node",
        name='control_node',
        respawn=True,
    )
 
    # included_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #             FindPackageShare("turtlebot3_gazebo"), '/launch', '/empty_world.launch.py']))
    # ld.add_action(included_launch)
    ld.add_action(control_node)
    return ld

# import launch
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
#     # Include launch description for the turtlesimbot gazebo simulation
#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([turtlebot3_gazebo', 'launch', 'empty_world.launch.py']),
#     )

#     # Launch description for your control node
#     control_node = Node(
#         package='assignment_1',
#         executable='control_node',
#         output='screen'
#     )

#     return LaunchDescription([
#         gazebo_launch,
#         control_node,
#     ])

