# Imports for ROS2
import rclpy
from rclpy.node import Node
from lh_interfaces.msg import Statekey

class KeypointSubscriber(Node):

    def __init__(self):
        super().__init__('keypoint_subscriber')

    ######## This is relevant to copy (Plus the import statements of course) #############

        self.subscription = self.create_subscription(Statekey, '/statekey', self.sub_callback, 1)

    def sub_callback(self, msg):
        active = msg.active # This is a boolean
        upcoming_keypoints = msg.upcoming_keypoints # This is an uint8 array
        time_to_keypoints = msg.time_to_keypoints # This is a float64 array

        self.get_logger().info(f"Active: {active}, Upcoming keypoints: {upcoming_keypoints}, Time to keypoints: {time_to_keypoints}")

    ####### End of what is relevant to copy ######

def main(args: list = None) -> None:
    rclpy.init(args=args)
    KeypointSubNode = KeypointSubscriber()
    try:
        rclpy.spin(KeypointSubNode)
    finally:
        # Cleanup code, if any
        KeypointSubNode.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()