import numpy as np

# Imports for ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lh_interfaces.msg import Statekey

class KeypointEstimator(Node):

    def __init__(self):
        super().__init__('keypoint_estimator')

        self.keypoints = range(3)
        self.keypoints_coordinates = np.zeros([len(self.keypoints), 2])
        # TEST - DELETE THIS
        self.keypoints_coordinates[0, :] = np.array([10, 10])
        # END TEST
        self.keypoints_distances = [0 for _ in range(len(self.keypoints))]
        self.keypoints_time_estimates = [0 for _ in range(len(self.keypoints))]

        self.keypoint_margin = 0.1
        self.keypoints_passed = 0

        self.last_coordinate = [0, 0]
        self.last_time = self.get_clock().now().nanoseconds/1000000000

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.updateDistanceToKeypoints, 1)
        self.statekeyPublisher = self.create_publisher(Statekey, "/statekey", 1)

    def updateDistanceToKeypoints(self, msg):
        time = self.get_clock().now().nanoseconds/1000000000
        coordinate = [0, 0]
        coordinate[0] = msg.pose.pose.position.x
        coordinate[1] = msg.pose.pose.position.y

        # Calculate distance to all keypoints
        for i in range(len(self.keypoints)):
            self.keypoints_distances[i] = np.sqrt((coordinate[0] - self.keypoints_coordinates[i, 0])**2 + (coordinate[1] - self.keypoints_coordinates[i, 1])**2)

        # Calculate current speed
        speed = np.sqrt((coordinate[0] - self.last_coordinate[0])**2 + (coordinate[1] - self.last_coordinate[1])**2)/(time - self.last_time)
        self.last_time = time

        # Estimate time to keypoints
        for i in range(len(self.keypoints)):
            self.keypoints_time_estimates[i] = self.keypoints_distances[i]/speed

        # Check if a keypoint is passed
        if self.keypoints_distances[self.keypoints_passed] < self.keypoint_margin:
            self.keypoints_passed += 1

        # Publish the correct amount of keypoints are their time estimates
        new_msg = Statekey()
        new_msg.active = True
        new_msg.upcoming_keypoints = self.keypoints[self.keypoints_passed:len(self.keypoints)]
        self.get_logger().info(f"Speed {speed} - Type: {type(speed)} - Time: {time}")
        new_msg.time_to_keypoints = self.keypoints_time_estimates[self.keypoints_passed:len(self.keypoints)]
        self.statekeyPublisher.publish(new_msg)

def main(args: list = None) -> None:
    rclpy.init(args=args)
    KeypointEstimatorNode = KeypointEstimator()
    try:
        rclpy.spin(KeypointEstimatorNode)
    finally:
        # Cleanup code, if any
        KeypointEstimatorNode.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()