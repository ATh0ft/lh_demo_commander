import rclpy
from rclpy.node import Node 
import numpy as np 

import time

from lh_interfaces.msg import Statekey

from tqdm import tqdm 


HZ = 10

START_DISTANCE = 3 

END_DISTANCE = -3

SPEED = 0.2

JITTER = 0




class StatekeyEmulator(Node):
    def __init__(self):
        super().__init__('statekey_emulator')
        self.statekey_publisher = self.create_publisher(Statekey, '/statekey',1)
    
    def send_statekeys(self):
        # assuming linear motion 
        list_of_points = np.linspace(START_DISTANCE, END_DISTANCE, np.uint64((np.abs(START_DISTANCE - END_DISTANCE)/SPEED)*HZ) )
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

            msg = Statekey()
            if point > 0:
                msg.active = True 
                msg.upcoming_keypoints = [1]
                msg.time_to_keypoints = [time_to_goal]

                self.statekey_publisher.publish(msg)
            else:
                msg.active = False 
                msg.upcoming_keypoints = [0]
                msg.time_to_keypoints = [0.0]

                self.statekey_publisher.publish(msg)


            time.sleep(1/HZ) 
 
def main(args:list = None) -> None:
    rclpy.init(args=args)
    emulator = StatekeyEmulator()
    emulator.send_statekeys()

if __name__ == "__main__":
    main()
