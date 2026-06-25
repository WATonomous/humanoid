

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
import numpy as np

class EKFPredictionNode(Node):
    def __init__(self):
        super().__init__('ekf_prediction_node')

        self.dt = 0.01  # Time step
        self.latest_tracket_data = None  # Store the latest tracket data
        self.best_current_estimate = None  # Store the best current estimate
        self.g = np.array([0, 0, -9.81])
         #This is the L-value
        self.aerodynamic_characteristic_length = 3.4
        self.sigma_meas = 0.04 #4 cm standard deviation
        self.R = (self.sigma_meas ** 2) * np.eye(3)
        


        self.pub = self.create_publisher(
            Pose,
            '/ekf_predicted_states',
            10
        )
        self.sub = self.create_subscription(
            Pose,
            '/shuttle_states',
            self.shuttle_callback,
            10
        )

        self.timer = self.create_timer(self.dt, self.step)

    def shuttle_callback(self, msg):
        self.latest_tracket_data = msg

    def step(self):
        if self.latest_tracket_data is not None:


            self.pub.publish(self.latest_tracket_data)

  

def main(args=None):
    rclpy.init(args=args)
    ekf_prediction_node = EKFPredictionNode()
    rclpy.spin(ekf_prediction_node)
    ekf_prediction_node.destroy_node()
    rclpy.shutdown()