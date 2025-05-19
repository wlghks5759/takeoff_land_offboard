from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'connection_url',
            default_value='udpin://0.0.0.0:14540',
            description='MAVSDK connection URL'
        ),
        DeclareLaunchArgument(
            'square_side_length_m',
            default_value='10.0',
            description='Side length of the square path in meters'
        ),
        DeclareLaunchArgument(
            'altitude_layer1_m',
            default_value='2.0',
            description='Relative altitude for the first layer of the square path'
        ),
        DeclareLaunchArgument(
            'altitude_layer2_m',
            default_value='4.0',
            description='Relative altitude for the second layer of the square path'
        ),
        DeclareLaunchArgument(
            'waypoint_threshold_m',
            default_value='1.0',  # New threshold parameter for waypoint completion
            description='Distance threshold in meters to consider waypoint reached'
        ),
        
        Node(
            package='takeoff_land_offboard',  # Use your actual package name here
            executable='takeoff_land_offboard',  # Use your actual executable name here
            name='autonomous_flight_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'connection_url': LaunchConfiguration('connection_url'),
                'square_side_length_m': LaunchConfiguration('square_side_length_m'),
                'altitude_layer1_m': LaunchConfiguration('altitude_layer1_m'),
                'altitude_layer2_m': LaunchConfiguration('altitude_layer2_m'),
                'waypoint_threshold_m': LaunchConfiguration('waypoint_threshold_m'),  # Pass the new parameter
            }]
        )
    ])