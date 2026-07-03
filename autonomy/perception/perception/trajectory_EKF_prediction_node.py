

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from common_msgs.msg import PerceptionImpactEstimation
import numpy as np


class EKFPredictionNode(Node):
    def __init__(self):
        super().__init__('ekf_prediction_node')
        

        self.dt = 0.01

        # ---------------- STATE ----------------
        # [px, py, pz, vx, vy, vz]
        self.x = np.zeros(6)
        self.true_pos = np.zeros(3)
        self.increments = 0

        # ---------------- COVARIANCE ----------------
        self.P = np.eye(6) * 0.1
        self.P[3:6, 3:6] = np.eye(3) * 50.0   # high uncertainty on vx, vy, vz
        self.R = (0.04 ** 2) * np.eye(3)  # measurement noise
        self.process_noise_pos = 1e-2
        self.process_noise_vel = 1e-2
       

        # ---------------- CONSTANTS ----------------
        self.g = np.array([0.0, 0.0, -9.81])
        self.L = 3.4 #aerodynamic characteristic length (L-value) 
        self.look_ahead_time = 2 #seconds
        self.bounding_sphere_radius = 2.0 #meters
        

       

        # ---------------- ROS ----------------
        self.pub = self.create_publisher(Pose, '/ekf_predicted_states', 10)
        self.sub = self.create_subscription(Pose, '/shuttle_states', self.shuttle_callback, 10)
        self.true_sub = self.create_subscription(Pose, '/shuttle_states_true', self.shuttle_true_callback, 10)
        self.new_spawn = self.create_subscription(Bool, '/shuttle_spawned', self.shuttle_spawned_callback, 10)
        self.impact_estimate_pub = self.create_publisher(PerceptionImpactEstimation, '/ekf_impact', 10)

        self.latest_meas = None
        self.latest_meas_time = None
        self.new_meas_available = False
        self.low_uncertainty = False

        # ---------------- VELOCITY SEEDING ----------------
        self.prev_meas = None
        self.prev_meas_increment = None
        self.velocity_initialized = False

        self.timer = self.create_timer(self.dt, self.step)

    # ---------------- CALLBACK ----------------
    def shuttle_callback(self, msg):
        self.latest_meas = msg
        self.latest_meas_time = self.increments
        self.new_meas_available = True

        z = np.array([msg.position.x, msg.position.y, msg.position.z])

        if not self.velocity_initialized:
            if self.prev_meas is not None:
                dt_elapsed = (self.increments - self.prev_meas_increment) * self.dt
                # guard: only seed if elapsed time is not like zero or too small
                if dt_elapsed > self.dt:
                    v_init = (z - self.prev_meas) / dt_elapsed
                    self.x[0:3] = z
                    self.x[3:6] = v_init
                    self.velocity_initialized = True
                    self.get_logger().info(
                        f"Velocity seeded: {v_init}")
            self.prev_meas = z
            self.prev_meas_increment = self.increments

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
            self.P[3:6, 3:6] = np.eye(3) * 50.0 
            self.increments = 0
            self.latest_meas = None
            self.new_meas_available = False
            self.latest_meas_time = None
            self.true_pos = np.zeros(3)
            self.low_uncertainty = False
            self.prev_meas = None
            self.prev_meas_increment = None
            self.velocity_initialized = False

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

    # ---------------- IMPACT ESTIMATION ----------------
    def estimate_impact(self):
        future_x = self.x.copy()
        impact = False
        time_to_impact = -1
        position_of_impact = np.zeros(3)
        hitback_direction = np.zeros(3)
        
        if(future_x[0]**2 + future_x[1]**2 + future_x[2]**2 <= self.bounding_sphere_radius**2):
            self.get_logger().debug("Already impacted")
            return
        for i in range(int(self.look_ahead_time / self.dt)):
            future_x = self.f(future_x)
            if future_x[0]**2 + future_x[1]**2 + future_x[2]**2 <= self.bounding_sphere_radius**2:
                impact = True
                #using the negative of the normalized velocity at impact to estimate the ideal racket facing orientation for a hit back
                hitback_direction = -(future_x[3:6]/np.linalg.norm(future_x[3:6]))
                time_to_impact = i * self.dt
                position_of_impact = future_x[0:3]
                break
        if (impact):
            self.get_logger().debug(f"Impact estimated in {time_to_impact:.2f} seconds at position {position_of_impact} with hit back racket direction {hitback_direction})")
        else:
            self.get_logger().debug("No impact estimated in the next 2 seconds")
        # Publish the impact estimate
        impact_msg = PerceptionImpactEstimation()
        impact_msg.time_to_impact = float(time_to_impact)

        impact_msg.position_of_impact.x = float(position_of_impact[0])
        impact_msg.position_of_impact.y = float(position_of_impact[1])
        impact_msg.position_of_impact.z = float(position_of_impact[2])

        impact_msg.hitback_direction.x = float(hitback_direction[0])
        impact_msg.hitback_direction.y = float(hitback_direction[1])
        impact_msg.hitback_direction.z = float(hitback_direction[2])

        impact_msg.uncertainty = float(np.trace(self.P))

        self.impact_estimate_pub.publish(impact_msg)

            

    # ---------------- STEP ----------------
    def step(self):
        self.increments += 1
        if (self.true_pos[0]**2 + self.true_pos[1]**2 + self.true_pos[2]**2 <= self.bounding_sphere_radius**2):
            self.get_logger().debug(f"Shuttle has impacted at true position {self.true_pos}")
        if (np.trace(self.P) < 2):
            if(not self.low_uncertainty):
                self.get_logger().info("Low uncertainty → reducing process noise")
            self.process_noise_pos = 1e-5
            self.process_noise_vel = 1e-5
            self.low_uncertainty = True
        else:
            self.process_noise_pos = 1e-2
            self.process_noise_vel = 1e-2
        
        
        # ====IMPACT ESTIMATION =====
        self.estimate_impact()
        # ===== PREDICT =====
        F = self.F_jacobian(self.x)   # old state
        self.x = self.f(self.x)  
   
        
     
        Q = np.eye(6)
        Q[0:3, 0:3] *= self.process_noise_pos
        Q[3:6, 3:6] *= self.process_noise_vel
        self.P = F @ self.P @ F.T + Q

        # ===== UPDATE =====
        if self.latest_meas is not None and self.new_meas_available:
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
        if (self.latest_meas is not None and self.new_meas_available):
                self.get_logger().info(
                    f"Total Error: {np.linalg.norm(self.true_pos - self.x[0:3])}")
                
        self.get_logger().debug(f'Predicted Position: {self.x[0:3]}')
        self.new_meas_available = False
        
           
        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = EKFPredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()