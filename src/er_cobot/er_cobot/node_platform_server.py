import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float32MultiArray

import numpy as np

from pymycobot.mycobot import MyCobot

from er_cobot.msgpacking import init_matrix_array_ros_msg, pack_multiarray_ros_msg, pack_np_matrix_from_multiarray_msg

#from rclpy.qos import QoSProfile, QoSReliabilityPolicy
#https://github.com/elephantrobotics/pymycobot/blob/v3.5.3/pymycobot/generate.py

class NodePlatform(Node):

    def __init__(self):
        super().__init__('node_platform_server')

        baud = 115200
        port = '/dev/ttyACM0'
        self.mc = MyCobot(port, str(baud))

        self.mc.set_fresh_mode(1)

        ### Publishers ###
        self.pub_angles = self.create_publisher(Float32MultiArray, 'platform_server/angles', 10)
        self.msg_angles = init_matrix_array_ros_msg()
        self.cur_angles = None

        ### Subscribers ###
        self.create_subscription(Float32MultiArray, 'platform_client/tgt_angles', self.callback_tgt, 10)
        self.msg_tgt_angles = None
        self.timer = self.create_timer(0.1, self.timer_callback)

        ### Initial poistion ###
        initial_position = [0., 0., 0., 0., 0., 0.]
        sp =  10
        self.mc.send_angles(initial_position, sp)

    def callback_tgt(self, msg):
        self.msg_tgt_angles = msg

        angles_n_speed = pack_np_matrix_from_multiarray_msg(self.msg_tgt_angles)        
        tgt_angles = angles_n_speed[0][:-1]
        speed = angles_n_speed[0][-1]

        # if self.cur_angles is not None:
        #     tgt_angles = self.cur_angles + np.clip((tgt_angles - self.cur_angles)/np.linalg.norm(tgt_angles - self.cur_angles), -speed*5,speed*0.7)

        angles = tgt_angles.tolist()
        speed = int(speed)
        self.mc.send_angles(angles, speed)

    def timer_callback(self):

        try:
            angles = self.mc.get_angles()
            angles = np.array(angles)
            self.cur_angles = angles.squeeze()
            angles = np.expand_dims(angles, axis=0)
            self.msg_angles = pack_multiarray_ros_msg(self.msg_angles, angles)
            self.pub_angles.publish(self.msg_angles)

        except:
            print("error in publishing servo angles")



        # if self.msg_tgt_angles is not None:
        
        #     angles_n_speed = pack_np_matrix_from_multiarray_msg(self.msg_tgt_angles)
            
        #     tgt_angles = angles_n_speed[0][:-1]
        #     speed = angles_n_speed[0][-1]

        #     cur_angles = cur_angles.squeeze()
        #     tgt_angles = tgt_angles.squeeze()

        #     print(cur_angles, tgt_angles)

        #     tgt_angles = cur_angles + 5*(tgt_angles - cur_angles)/np.linalg.norm(tgt_angles - cur_angles)

            

        #     angles = tgt_angles.tolist()
        #     speed = int(speed)

        #     self.mc.send_angles(angles, speed)

        # try:
        #     angles = self.mc.get_angles()
        #     angles = np.array(angles)
        #     cur_angles = np.expand_dims(angles, axis=0)
        #     self.msg_angles = pack_multiarray_ros_msg(self.msg_angles, cur_angles)
        #     self.pub_angles.publish(self.msg_angles)

        #     if self.msg_tgt_angles is not None:
            
        #         angles_n_speed = pack_np_matrix_from_multiarray_msg(self.msg_tgt_angles)
                
        #         tgt_angles = angles_n_speed[0][:-1]
        #         speed = angles_n_speed[0][-1]

        #         cur_angles = cur_angles.squeeze()
        #         tgt_angles = tgt_angles.squeeze()
        #         tgt_angles = cur_angles + 5*(tgt_angles - cur_angles)/np.linalg.norm(tgt_angles - cur_angles)

        #         print(tgt_angles, speed)

        #         angles = tgt_angles.tolist()
        #         speed = int(speed)

        #         self.mc.send_angles(angles, speed)


        # except:
        #     print("error in getting servo angles")

        

def main():
    rclpy.init()
    node_platform = NodePlatform()
    rclpy.spin(node_platform)

    # Destroy the node
    node_platform.destroy_node()
    node_platform.mc.set_color(255,255,255)
    rclpy.shutdown()

if __name__ == '__main__':
    main()