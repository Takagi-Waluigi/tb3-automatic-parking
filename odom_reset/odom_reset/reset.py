import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np
import math

class OdomResetClass(Node):

    def __init__(self):
        super().__init__('node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription

        self.save_origin = True

        self.i = 0

        self.origin = Pose()

    def listener_callback(self, msg):
        if self.save_origin: 
            #1.ノード起動時から最初に受け取ったOdomを原点とする
            print("save origin odometry")
            self.origin = msg.pose.pose
            self.save_origin = False

        else: 
            #2.ノード起動時の原点に対しての相対位置を以下で算出する
            self.currentPose = msg.pose.pose
            self.translatedPoint = Point()

            #2-1.平行移動してOdom原点に移動
            self.translatedPoint.x = self.currentPose.position.x - self.origin.position.x
            self.translatedPoint.y = self.currentPose.position.y - self.origin.position.y

            #2-2.回転移動
            self.originTheta = self.euler_from_quat(self.origin.orientation)

            self._translatedPoint = Point() #計算結果を格納するためのPoint
            self._translatedPoint.x =  self.translatedPoint.x * math.cos(self.originTheta) + self.translatedPoint.y * math.sin(self.originTheta)
            self._translatedPoint.y = -self.translatedPoint.x * math.sin(self.originTheta) + self.translatedPoint.y * math.cos(self.originTheta)
            
            #For Debug
            if self.i % 10 == 0:
               print("translated Positon:", self._translatedPoint.x, "," , self._translatedPoint.y)

            self.i += 1

    def euler_from_quat(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)                                                              
                                                                                                                  
        sinp  = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
                                                                                                                  
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw  
        
def main(args=None):
    rclpy.init(args=args)

    node = OdomResetClass()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()