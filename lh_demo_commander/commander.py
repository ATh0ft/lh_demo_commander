import rclpy
from rclpy.logging import initialize
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigateToPose
from nav2_msgs.msg import SpeedLimit

class Commander(BasicNavigator):
    def __init__(self, node_name='basic_navigator', namespace=''):
        super().__init__(node_name, namespace)
        self.declare_parameter("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY)    
        self.declare_parameter("goal_pose", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("speed_limit", rclpy.Parameter.Type.DOUBLE)
        
        self.speed_limit_pub_ = self.create_publisher(msg_type=SpeedLimit, topic="speed_limit", qos_profile=10)

        initial_pose = self.get_parameter("initial_pose").value
        goal_pose = self.get_parameter("goal_pose").value
        self.speed_limit = self.get_parameter("speed_limit").value

        self.initial_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose.header.frame_id = "map"
        self.initial_pose.pose.position.x = initial_pose[0]
        self.initial_pose.pose.position.y = initial_pose[1]
        self.initial_pose.pose.orientation.z = initial_pose[2]
        self.initial_pose.pose.orientation.w = initial_pose[3]
        
        self.goal_pose.pose.position.x = goal_pose[0]
        self.goal_pose.pose.position.y = goal_pose[1]
        self.goal_pose.pose.orientation.z = goal_pose[2]
        self.goal_pose.pose.orientation.w = goal_pose[3]

        self.setInitialPose(self.initial_pose)

        self.waitUntilNav2Active()
        self.speed_limit_pub()
        
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goToPose(self.goal_pose)
    
    def speed_limit_pub(self):
        sl = SpeedLimit()
        sl.header.frame_id = "map" #????
        sl.header.stamp = self.get_clock().now().to_msg()
        sl.percentage = False
        sl.speed_limit = self.speed_limit
        self.speed_limit_pub_.publish(sl)

        

def main(args: list = None) -> None:
    rclpy.init(args=args)
    commander = Commander()
    try:
        rclpy.spin(commander)
    finally:
        commander.destroyNode()
        # rclpy.shutdown()

if __name__ == "__main__":
    main()


