#!/usr/bin/env python3

import time
import board
import adafruit_ina260
import rospy
from micro_uuv.msgs import Power

class PowerSensor:
    def __init__(self):
        rospy.init_node("ina260_ros")
        self.publisher = rospy.Publisher("power", Power, queue_size=10)

        i2c = board.I2C()

        self.device = adafruit_ina260.INA260(i2c)

        self.msg = Power()

    def read_sensor(self):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.voltage = self.device.voltage
        self.msg.current = self.device.current
        self.msg.power = self.device.power

    def publish(self):
        self.publisher.publish(self.msg)


if __name__ == "__main__":
    sensor = PowerSensor()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        sensor.read_sensor()
        sensor.publish()
        r.sleep()

