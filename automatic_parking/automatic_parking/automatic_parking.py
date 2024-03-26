import rclpy
import math
import sys
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
import numpy as np
from math import sin, cos, pi ,atan2
import tf2_geometry_msgs
import tf2_ros 
import tf2_tools

import time

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class Automatic_parking_Publisher(Node):

    def __init__(self):
        super().__init__('automatic_parking_publisher')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.reset_pub = self.create_publisher(Empty, '/reset', 10)

        self.scan_spot_pub = self.create_publisher(LaserScan, "/scan_spot", qos_profile)

        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserScanCallback,
            qos_profile
        )

        self.laser_subscription

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odomCallback,
            10
        )

        self.odom_subscription

        self.odom = Odometry()
        self.scan = LaserScan()
        self.reset = Empty()
        self.save_origin = True
        self.origin = Pose()
        self.i = 0
        self.translatedyaw = 0
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.step = 0
        self.theta = 0

    def timer_callback(self):

        print("step: ", self.step)

        self.twist = Twist()
        yaw = self.translatedyaw

        print("yaw:",yaw)
        
        scan_done, center_angle, start_angle, end_angle = self.scan_parking_spot()
        
        
        if self.step == 0:
            if scan_done == True:
                self.fining_spot, self.start_point, self.center_point, self.end_point = self.finding_spot_position(center_angle, start_angle, end_angle)
                if self.fining_spot == True:
                    self.theta = np.arctan2(self.start_point[1] - self.end_point[1], self.start_point[0] - self.end_point[0])
                    print("=================================")
                    print("|        |     x     |     y     |")
                    print('| start  | {0:>10.3f}| {1:>10.3f}|'.format(self.start_point[0], self.start_point[1]))
                    print('| center | {0:>10.3f}| {1:>10.3f}|'.format(self.center_point[0], self.center_point[1]))
                    print('| end    | {0:>10.3f}| {1:>10.3f}|'.format(self.end_point[0], self.end_point[1]))
                    print("=================================")
                    print('| self.theta  | {0:.2f} deg'.format(np.rad2deg(self.theta)))
                    print('| yaw    | {0:.2f} deg'.format(np.rad2deg(yaw)))
                    print("=================================")
                    print("===== Go to parking spot!!! =====")
                    self.step = 1
            else:
                print("Fail finding parking spot.")

        elif self.step == 1:
            init_yaw = yaw
            yaw = self.theta + yaw
            print("theta: ", self.theta)
            print("theta-yaw", self.theta - init_yaw)
            if self.theta > 0:
                if self.theta - init_yaw > 0.2:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.1
                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_pub.publish(self.twist)
                    time.sleep(1)
                    self.reset_pub.publish(self.reset)
                    self.save_origin = True
                    time.sleep(3)
                    self.rotation_point = self.rotate_origin_only(self.center_point[0], self.center_point[1], -(pi / 2 - init_yaw))
                    self.step = 2
            else:
                if self.theta - init_yaw < -0.2:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -0.1
                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_pub.publish(self.twist)
                    time.sleep(1)
                    self.reset_pub.publish(self.reset)
                    self.save_origin = True
                    time.sleep(3)
                    self.rotation_point = self.rotate_origin_only(self.center_point[0], self.center_point[1], -(pi / 2 - init_yaw))
                    self.step = 2
        
        elif self.step == 2:
            print("abs: ", abs(self._translatedPoint.x - (self.rotation_point[1])))
            if abs(self._translatedPoint.x - (self.rotation_point[1])) > 0.05:
                if self._translatedPoint.x > (self.rotation_point[1]):
                    self.twist.linear.x = -0.02
                    self.twist.angular.z = 0.0
                else:
                    self.twist.linear.x = 0.02
                    self.twist.angular.z = 0.0
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.step = 3

        elif self.step == 3:
            print("yaw: ",yaw)
            if yaw > -pi / 2:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.2
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.step = 4

        elif self.step == 4:
            ranges = []
            for i in range(150, 210):
                if self.scan.ranges[i] != 0:
                    ranges.append(self.scan.ranges[i])
            if min(ranges) > 0.2:
                self.twist.linear.x = -0.04
                self.twist.angular.z = 0.0
            else:
                self.twist.linear.x = 0.0
                self.cmd_pub.publish(self.twist)
                time.sleep(2)
                sys.exit()

        self.cmd_pub.publish(self.twist)
        #self.scan_spot_filter(self.scan, center_angle, start_angle, end_angle)

    def laserScanCallback(self, msg):
        self.scan = msg
        #self.get_logger().info('I heard: laser scan')

    def odomCallback(self, msg):
        #self.odom = msg
        #self.get_logger().info('I heard: odom')
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
            self.originTheta = self.quaternion(self.origin.orientation)
            self.currentTheta = self.quaternion(self.currentPose.orientation)
            self.translatedyaw = self.currentTheta - self.originTheta
            self._translatedPoint = Point() #計算結果を格納するためのPoint
            self._translatedPoint.x =  self.translatedPoint.x * math.cos(self.originTheta) + self.translatedPoint.y * math.sin(self.originTheta)
            self._translatedPoint.y = -self.translatedPoint.x * math.sin(self.originTheta) + self.translatedPoint.y * math.cos(self.originTheta)
            
            # #For Debug
            # if self.i % 10 == 0:
            #    print("translated Positon:", self._translatedPoint.x, "," , self._translatedPoint.y)

            # self.i += 1


    def scan_parking_spot(self):
        scan_done = False
        intensity_index = []
        index_count = []
        spot_angle_index = []
        minimun_scan_angle = 30
        maximun_scan_angle = 330
        intensity_threshold = 1.3
        #intensity_threshold = 100
        center_angle = 0
        start_angle = 0
        end_angle = 0
        if len(self.scan.intensities) > 0:
            print("len",len(self.scan.intensities))
            for i in range(len(self.scan.intensities)):
                if i >= minimun_scan_angle and i < maximun_scan_angle:
                    # print("spot_intensity_cal:",self.scan.intensities[i] ** 2 * self.scan.ranges[i] / 100000)
                    # print("spot_intensity:",self.scan.intensities[i])
                    # print("spot_ranges:", self.scan.ranges[i])
                    #print("len",len(self.scan.intensities))
                    spot_intensity = self.scan.intensities[i] ** 2 * self.scan.ranges[i] / 100000
                    if spot_intensity >= intensity_threshold:
                        intensity_index.append(i)
                        index_count.append(i)
                    else:
                        intensity_index.append(0)
                else:
                    intensity_index.append(0)

            for i in index_count:
                if abs(i - index_count[int(len(index_count) / 2)]) < 20:
                    spot_angle_index.append(i)
                    if len(spot_angle_index) > 10:
                        scan_done = True
                        center_angle = spot_angle_index[int(len(spot_angle_index) / 2)]
                        start_angle = spot_angle_index[2]
                        end_angle = spot_angle_index[-3]

                    else:
                        scan_done = False
                        
        return scan_done, center_angle, start_angle, end_angle

    def quaternion(self, quat): 
        x = quat.x                                                                                          
        y = quat.y                                                                                         
        z = quat.z
        w = quat.w                                                                                         
                                                                                                                  
        sinr_cosp = 2 * (w * x + y * z)                                                                           
        cosr_cosp = 1 - 2 * (x * x + y * y)                                                                       
        roll      = np.arctan2(sinr_cosp, cosr_cosp)                                                              
                                                                                                                  
        sinp  = 2 * (w * y - z * x)                                                                               
        pitch = np.arcsin(sinp)                                                                                   
                                                                                                                  
        siny_cosp = 2 * (w * z + x * y)                                                                           
        cosy_cosp = 1 - 2 * (y * y + z * z)                                                                       
        yaw = np.arctan2(siny_cosp, cosy_cosp)                                                                    
                                                                                                                  
        return yaw 
    
    def get_angle_distance(self,angle):
        distance = self.scan.ranges[int(angle)]
        if (self.scan.ranges[int(angle)] != None) and (distance != 0):
            angle = int(angle)
            distance = distance
        return angle, distance

    def get_point(self,start_angle_distance):
        angle = start_angle_distance[0]
        angle = np.deg2rad(angle - 180)
        distance = start_angle_distance[1]

        if angle >= 0 and angle < pi / 2:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        elif angle >= pi / 2 and angle < pi:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        elif angle >= -pi / 2 and angle < 0:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1
        else:
            x = distance * cos(angle) * -1
            y = distance * sin(angle) * -1

        return [x, y]

    def finding_spot_position(self,center_angle, start_angle, end_angle):
        print("scan parking spot done!")
        fining_spot = False
        start_angle_distance = self.get_angle_distance(start_angle)
        center_angle_distance = self.get_angle_distance(center_angle)
        end_angle_distance = self.get_angle_distance(end_angle)

        if start_angle_distance[1] != 0 and center_angle_distance[1] != 0 and end_angle_distance[1] != 0:
            print("calibration......")
            start_point = self.get_point(start_angle_distance)
            center_point = self.get_point(center_angle_distance)
            end_point = self.get_point(end_angle_distance)
            fining_spot = True
        else:
            fining_spot = False
            print("wrong scan!!")

        return fining_spot, start_point, center_point, end_point

    def rotate_origin_only(self,x, y, radians):
        xx = x * cos(radians) + y * sin(radians)
        yy = -x * sin(radians) + y * cos(radians)
        return xx, yy
    
    def scan_spot_filter(self, msg, center_angle, start_angle, end_angle):
        scan_spot = msg
        scan_spot_list = list(scan_spot.intensities)

        # for i in range(360):
        #     scan_spot_list[i] = 0
        
        for i in len(scan_spot_list):
            scan_spot_list[i] = 0

        scan_spot_list[start_angle] = msg.ranges[start_angle] + 10000
        scan_spot_list[center_angle] = msg.ranges[center_angle] + 10000
        scan_spot_list[end_angle] = msg.ranges[end_angle] + 10000
        scan_spot.intensities = tuple(scan_spot_list)

        self.scan_spot_pub.publish(scan_spot)



def main(args=None):
    rclpy.init(args=args)

    automatic_parking_publisher = Automatic_parking_Publisher()

    rclpy.spin(automatic_parking_publisher)

    automatic_parking_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()