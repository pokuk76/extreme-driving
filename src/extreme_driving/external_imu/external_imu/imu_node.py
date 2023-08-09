from __future__ import print_function
import sys
import qwiic_icm20948

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExternalIMU(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.timer = self.create_timer(0.001, self.timer_callback)

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
                print(\
                '{: 06d}'.format(IMU.axRaw)\
                , '\t', '{: 06d}'.format(IMU.ayRaw)\
                , '\t', '{: 06d}'.format(IMU.azRaw)\
                , '\t', '{: 06d}'.format(IMU.gxRaw)\
                , '\t', '{: 06d}'.format(IMU.gyRaw)\
                , '\t', '{: 06d}'.format(IMU.gzRaw)\
                , '\t', '{: 06d}'.format(IMU.mxRaw)\
                , '\t', '{: 06d}'.format(IMU.myRaw)\
                , '\t', '{: 06d}'.format(IMU.mzRaw)\
                )
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