import rclpy
from rclpy.node import Node 
import numpy as np 

import time

from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped

from tqdm import tqdm 


HZ = 10

START_DISTANCE = 3 

END_DISTANCE = -3

SPEED = 0.2

JITTER = 0.1




class AmclPoseEmulator(Node):
    def __init__(self):
        super().__init__('amcl_pose_emulator')
        self.amcl_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose',1)
    
    def send_amcl_pose(self):
        # assuming linear motion 
        list_of_points = np.linspace(START_DISTANCE, END_DISTANCE, np.uint64((np.abs(START_DISTANCE - END_DISTANCE)/SPEED)*HZ) )

        jitter = np.random.normal(0, JITTER, list_of_points.shape)

        list_of_points += jitter

        prev_point = list_of_points[0]
        
        time_prev_point = self.get_clock().now().nanoseconds/1000000000

        for point in tqdm(list_of_points):
            time_current = self.get_clock().now().nanoseconds/1000000000
            time_delta = np.abs(time_prev_point - time_current)
            
            
            distance = np.abs(prev_point-point)

            speed_est = distance/time_delta 
            # print(speed_est)

            time_to_goal = (START_DISTANCE-distance)/speed_est

            # adding tha data to the msg 

            msg = PoseWithCovarianceStamped()
            
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.header.frame_id = 'map'

            msg.pose.pose.position.x = point

            self.amcl_pose_publisher.publish(msg)
            
            random_time = np.random.normal(0, 0.1)

            time.sleep(np.abs(1/HZ + random_time)) 
 
def main(args:list = None) -> None:
    rclpy.init(args=args)
    emulator = AmclPoseEmulator()
    emulator.send_amcl_pose()

if __name__ == "__main__":
    main()
