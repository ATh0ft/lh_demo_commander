from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	
	# Pass keypoints as pairs of x and y coordinates
	coordinates_param = DeclareLaunchArgument("keypoints", default_value="[1.1, 2.2, 3.3, 4.4]")

	return LaunchDescription([
		coordinates_param,
		Node(
			package="lh_demo_commander",
			executable="keypoint_estimator",
			output="screen",
			parameters=[{"keypoints": LaunchConfiguration("keypoints")}]
		),
	])
