from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Pass keypoints as pairs of x and y coordinates
    coordinates_param = DeclareLaunchArgument("keypoints", default_value="[-1.7671095660857,-0.329028381597]")
    # coordinates_param = DeclareLaunchArgument("keypoints", default_value="[0.0, 0.0]")
    initial_pose_param = DeclareLaunchArgument("initial_pose", default_value="[-3.505,  -2.206, 0.31073, 0.95048]") # x, y, rz, rw
    goal_pose_param = DeclareLaunchArgument("goal_pose", default_value="[ -0.625, 0.89,  0.31073, 0.95048]")
    # initial_pose_param = DeclareLaunchArgument("initial_pose", default_value="[3.0,  0.0, 0.0, 1.0]") # x, y, rz, rw
       
    # goal_pose_param = DeclareLaunchArgument("goal_pose", default_value="[-3.0, 0.0, 0.0, 1.0]")

    speed_limit_param = DeclareLaunchArgument("speed_limit", default_value="0.15")
    return LaunchDescription([
		coordinates_param,
        initial_pose_param,
        goal_pose_param,
        speed_limit_param,
		Node(
			package="lh_demo_commander",
			executable="keypoint_estimator",
			output="screen",
			parameters=[{"keypoints": LaunchConfiguration("keypoints"),
                         "goal_pose": LaunchConfiguration("goal_pose"), 
                         "initial_pose": LaunchConfiguration("initial_pose")}]

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
