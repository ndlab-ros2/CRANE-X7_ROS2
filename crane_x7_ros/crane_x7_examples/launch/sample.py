#練習用コード

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from time import sleep

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # トピックを使って関節角度を制御
        self.joint1_pub = self.create_publisher(Float64, '/crane_x7/joint1_position_controller/command', 10)
        self.joint2_pub = self.create_publisher(Float64, '/crane_x7/joint2_position_controller/command', 10)
        self.joint3_pub = self.create_publisher(Float64, '/crane_x7/joint3_position_controller/command', 10)
        
        # 目標角度を設定
        self.joint1_angle = Float64()
        self.joint2_angle = Float64()
        self.joint3_angle = Float64()

    def move_arm(self):
        # 動作例：関節を動かす
        self.joint1_angle.data = 1.0  # 1 radian
        self.joint2_angle.data = 0.5  # 0.5 radian
        self.joint3_angle.data = -0.5  # -0.5 radian

        # 各関節に目標角度を送信
        self.joint1_pub.publish(self.joint1_angle)
        self.joint2_pub.publish(self.joint2_angle)
        self.joint3_pub.publish(self.joint3_angle)
        
        self.get_logger().info(f'Moving arm to: {self.joint1_angle.data}, {self.joint2_angle.data}, {self.joint3_angle.data}')
        
        sleep(2)

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    
    # アームを動かす
    arm_controller.move_arm()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
