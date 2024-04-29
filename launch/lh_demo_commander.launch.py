from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Pass keypoints as pairs of x and y coordinates
    coordinates_param = DeclareLaunchArgument("keypoints", default_value="[-1.764949,  -0.258354891]")
    initial_pose_param = DeclareLaunchArgument("initial_pose", default_value="[-3.627925635,  -2.3089, 0.31076, 0.95048564]") # x, y, rz, rw
    goal_pose_param = DeclareLaunchArgument("goal_pose", default_value="[ -1.30540, 0.16398567, 0.31076, 0.95048564]")
    # initial_pose_param = DeclareLaunchArgument("initial_pose", default_value="[10.0,  10.0, 0.0, 1.0]") # x, y, rz, rw
       
    # goal_pose_param = DeclareLaunchArgument("goal_pose", default_value="[ 14.0, 10.0, 0.0, 1.0]")

    speed_limit_param = DeclareLaunchArgument("speed_limit", default_value="0.1")
    return LaunchDescription([
		coordinates_param,
        initial_pose_param,
        goal_pose_param,
        speed_limit_param,
		Node(
			package="lh_demo_commander",
			executable="keypoint_estimator",
			output="screen",
			parameters=[{"keypoints": LaunchConfiguration("keypoints")}]
		),
        Node(
			package="lh_demo_commander",
			executable="commander",
			output="screen",
            parameters=[{"initial_pose": LaunchConfiguration("initial_pose"), 
                         "goal_pose": LaunchConfiguration("goal_pose"), 
                         "speed_limit": LaunchConfiguration("speed_limit")}]
        ),
	])
