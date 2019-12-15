# This Python file uses the following encoding: utf-8

import launch
import launch_ros.actions
from launch import LaunchDescription
#from launch import ExecuteNodeProcess

def generate_launch_description():
   return LaunchDescription([
      launch_ros.actions.Node(
         package='sub_pub_template_test',
         node_executable='subscriber_template',
         node_name="subscriber",
         output='screen'),

      launch_ros.actions.Node(
         package='sub_pub_template_test',
         node_executable='publisher_template',
         node_name="publisher"
         )
   ])

