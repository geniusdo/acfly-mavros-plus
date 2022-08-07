from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('mavros'),
      'launch',
      'px4_pluginlists.yaml'
      )
   # config = os.path.join(
   #    get_package_share_directory('mavros'),
   #    'config',
   #    'do_config.yaml'
   #    )
      
   return LaunchDescription([
         Node(
           package='mavros',
           executable='mavros_node',
           #name='TEST',
           #output='screen',
         #   parameters=[{
         #       "fcu_url": "/dev/ttyACM0:115200",
         #       "gcs_url": "",
         #       "target_system_id": 1,
         #       "target_component_id": 1,
         #       "fcu_protocol": "v2.0"
         #   }]
           #parameters=[config1],
           parameters=[config],
        ),
   ])
