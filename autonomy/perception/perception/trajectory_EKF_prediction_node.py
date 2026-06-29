import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
import numpy as np
from std_msgs.msg import Bool

class EKFPredictionNode(Node):
    def __init__(self):
        super().__init__('ekf_prediction_node')
        

        self.dt = 0.01

        # ---------------- STATE ----------------
        # [px, py, pz, vx, vy, vz]
        self.x = np.zeros(6)
        self.true_pos = np.zeros(3)

        # ---------------- COVARIANCE ----------------
        self.P = np.eye(6) * 0.1 #process noise covariance
        self.R = (0.04 ** 2) * np.eye(3)  # measurement noise
        self.process_noise = 1e-6
        self.look_ahead_time = 2 #seconds
        self.bounding_sphere_radius = 5.0 #meters
    

        # ---------------- CONSTANTS ----------------
        self.g = np.array([0.0, 0.0, -9.81])
        self.L = 3.4
     

       

        # ---------------- ROS ----------------
        self.pub = self.create_publisher(Pose, '/ekf_predicted_states', 10)
        self.sub = self.create_subscription(Pose, '/shuttle_states', self.shuttle_callback, 10)
        self.true_sub = self.create_subscription(Pose, '/shuttle_states_true', self.shuttle_true_callback, 10)
        self.new_spawn = self.create_subscription(Bool, '/shuttle_spawned', self.shuttle_spawned_callback, 10)
        self.impact_estimate_pub = self.create_publisher(Bool, '/ekf_impact', 10)

        self.latest_meas = None

        self.timer = self.create_timer(self.dt, self.step)

    # ---------------- CALLBACK ----------------
    def shuttle_callback(self, msg):
        self.latest_meas = msg

    def shuttle_true_callback(self, msg):
        self.true_pos = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])
    def shuttle_spawned_callback(self, msg):
        if msg.data:
            self.get_logger().info("Shuttle spawned → resetting EKF state")
            self.x = np.zeros(6)
            self.P = np.eye(6) * 0.1
            self.latest_meas = None

    # ---------------- MODEL ----------------
    def f(self, x):
        p = x[0:3]
        v = x[3:6]

        speed = np.linalg.norm(v) + 1e-12

        dp = v
        dv = self.g - (speed / self.L) * v

        p_new = p + dp * self.dt
        v_new = v + dv * self.dt

        return np.hstack((p_new, v_new))

    # ---------------- JACOBIAN ----------------
    def F_jacobian(self, x):
        v = x[3:6]
        speed = np.linalg.norm(v) + 1e-12

        F = np.eye(6)

        # dp/dv = I * dt
        F[0:3, 3:6] = np.eye(3) * self.dt

        # dv/dv nonlinear drag term
        I = np.eye(3)



        # derivative of (|v| v)
        outer = np.outer(v, v) / speed

        drag_jac = (1 / self.L) * (speed * I + outer)

        F[3:6, 3:6] = I - drag_jac * self.dt

        return F

    # ---------------- STEP ----------------
    def step(self):
        # ===== PREDICT =====
        F = self.F_jacobian(self.x)   # old state
        self.x = self.f(self.x)
  

        Q = np.eye(6) * self.process_noise
        self.P = F @ self.P @ F.T + Q

        # ===== UPDATE =====
        if self.latest_meas is not None:
            z = np.array([
                self.latest_meas.position.x,
                self.latest_meas.position.y,
                self.latest_meas.position.z
            ])

            H = np.zeros((3, 6))
            H[:, 0:3] = np.eye(3)

            y = z - H @ self.x

            S = H @ self.P @ H.T + self.R
            K = self.P @ H.T @ np.linalg.inv(S)

            self.x = self.x + K @ y

            I = np.eye(6)
            self.P = (I - K @ H) @ self.P

        # ===== PUBLISH =====
        msg = Pose()
        msg.position.x = float(self.x[0])
        msg.position.y = float(self.x[1])
        msg.position.z = float(self.x[2])

        self.get_logger().info(#difference between true and predicted position
            f"Total Error: {np.linalg.norm(self.true_pos - self.x[0:3])}")
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFPredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
