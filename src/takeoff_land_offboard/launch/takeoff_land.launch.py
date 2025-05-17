from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'connection_url',
            default_value='udpin://0.0.0.0:14540', # Default for SITL
            description='MAVSDK connection URL'
        ),
        DeclareLaunchArgument(
            'takeoff_altitude',
            default_value='5.0', # Default takeoff altitude in meters
            description='Target takeoff altitude in meters'
        ),
        DeclareLaunchArgument(
            'mission_hold_duration_s',
            default_value='10', # Default hold duration in seconds
            description='Duration to hold position in air in seconds'
        ),

        Node(
            package='takeoff_land_offboard', # 패키지 이름은 실제 패키지 이름으로 변경
            executable='takeoff_land_offboard', # 실행 파일 이름은 실제 빌드된 이름으로 변경
            name='autonomous_flight_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'connection_url': LaunchConfiguration('connection_url'),
                'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                'mission_hold_duration_s': LaunchConfiguration('mission_hold_duration_s')
            }]
        )
    ])