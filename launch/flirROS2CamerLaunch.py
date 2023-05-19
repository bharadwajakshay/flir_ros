from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

camParameters ={
    'frameRate': 10.0,
    'cameraSerialNo':"23015063",
    'frameID': 'testCamera',
    'triggerMode': 'continous', # possible values 'continous', 'software', 'hardware'
    'exposureMode': 'auto', # possible values 'auto' and 'manual'
    'exposureTime': 15, # in milliseconds
    'maxExposureTime': 20, # this is teh maximum exposure time the camera is allowed to compensate to. 
}

def generate_launch_description():
    packageDir = get_package_share_directory('flir_ros2')
    node = Node(package='flir_ros2', 
                executable='flir_ros2',
                output='screen',
                parameters=[camParameters])
    return LaunchDescription([node])