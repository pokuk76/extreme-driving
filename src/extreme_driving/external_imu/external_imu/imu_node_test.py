from __future__ import print_function
import sys
import qwiic_icm20948

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float64
from sensor_msgs.msg import MagneticField,Imu

class ExternalIMU(Node):

    def __init__(self):
        super().__init__('imu_node_test')

        self.timer = self.create_timer(0.001, self.timer_callback)
        self.raw_pub = self.create_publisher(Imu, 'imu_node/raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu_node/mag', 10)

    def timer_callback(self):
        print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
        IMU = qwiic_icm20948.QwiicIcm20948()

        if IMU.connected == False:
            print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
                file=sys.stderr)
        else:
            IMU.begin()

            if IMU.dataReady():
                IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
                raw_msg = Imu()
                raw_msg.header.stamp = self.get_clock().now().to_msg()
                    
                raw_msg.orientation.w = 0.0
                raw_msg.orientation.x = 0.0
                raw_msg.orientation.y = 0.0
                raw_msg.orientation.z = 0.0
                    
                raw_msg.linear_acceleration.x = float(IMU.axRaw)
                raw_msg.linear_acceleration.y = float(IMU.ayRaw)
                raw_msg.linear_acceleration.z = float(IMU.azRaw)
                    
                raw_msg.angular_velocity.x = float(IMU.gxRaw)
                raw_msg.angular_velocity.y = float(IMU.gyRaw)
                raw_msg.angular_velocity.z = float(IMU.gzRaw)
                    
                raw_msg.orientation_covariance[0] = -1
                raw_msg.linear_acceleration_covariance[0] = -1
                raw_msg.angular_velocity_covariance[0] = -1
                    
                self.raw_pub.publish(raw_msg)
                    
                mag_msg = MagneticField()
                mag_msg.header.stamp = self.get_clock().now().to_msg()
                mag_msg.magnetic_field.x = float(IMU.mxRaw)
                mag_msg.magnetic_field.y = float(IMU.myRaw)
                mag_msg.magnetic_field.z = float(IMU.mzRaw)
                mag_msg.magnetic_field_covariance[0] = -1
                self.mag_pub.publish(mag_msg)
            else:
                print("Waiting for data")


def main():
    rclpy.init()
    node = ExternalIMU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()