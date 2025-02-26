import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import numpy as np
import math

from er_cobot.msgpacking import init_matrix_array_ros_msg, pack_multiarray_ros_msg

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('traj_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'platform_client/tgt_angles', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # Initial time
        self.msg_angles = init_matrix_array_ros_msg()

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        f_t = self.calculate_sine_wave(elapsed_time)
        f1_t = self.calculate_step(elapsed_time)
        print(f1_t)
        angles = np.array([f_t*60, 0.,90.,0.,0.,0., 80])
        angles = np.expand_dims(angles, axis=0)
        self.msg_angles = pack_multiarray_ros_msg(self.msg_angles, angles)
        self.publisher_.publish(self.msg_angles)
        
    @staticmethod
    def calculate_sine_wave(t):
        amplitude = 1
        frequency = 0.3
        return amplitude * math.sin(2 * math.pi * frequency * t)

    @staticmethod
    def calculate_trapezoidal(t):
        total_time = 10.0  # Total movement time (s)
        max_velocity = 1.0  # Max velocity (m/s)
        acc_time = 2.0  # Acceleration time (s)
        dec_time = 2.0  # Deceleration time (s)

        t = t%total_time

        if t < acc_time:
            # Acceleration phase: v = a * t
            return (max_velocity / acc_time) * t
        elif t < (total_time - dec_time):
            # Constant velocity phase
            return max_velocity
        elif t < total_time:
            # Deceleration phase: v = v_max - a * (t - t_dec_start)
            return max_velocity - (max_velocity / dec_time) * (t - (total_time - dec_time))
        else:
            return 0.0  # Stop at the end

    @staticmethod
    def calculate_step(t):
        total_time = 10.0  # Total movement time (s)
        max_velocity = 1.0  # Max velocity (m/s)
        acc_time = 2.0  # Acceleration time (s)
        dec_time = 2.0  # Deceleration time (s)

        t = t%total_time

        if t < acc_time:
            # Acceleration phase: v = a * t
            return max_velocity
        elif t < (total_time - dec_time):
            # Constant velocity phase
            return max_velocity
        elif t < total_time:
            # Deceleration phase: v = v_max - a * (t - t_dec_start)
            return 0
        else:
            return 0.0  # Stop at the end
   





def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()