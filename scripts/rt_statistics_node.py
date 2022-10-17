#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Point
from liquid_height_estimation.msg import Statistics

import numpy as np


class RealtimeStatistics:
    def __init__(self):
        self.window_size = 100
        
        rospy.init_node("realtime_statistics", anonymous=False)
        rospy.Subscriber("/height_detector/cylinder_coefficients", Float64MultiArray, self.coeff_cb)
        self.stat_pub = rospy.Publisher("/height_detector/statistics", Statistics, queue_size=1)
        self.stats = Statistics()

        self.radius = None
        self.x = None
        self.y = None
        self.z = None

        self.data = np.array([])
        self.means = np.zeros((1,4))
        self.medians = np.zeros((1,4))
        self.stds = np.zeros((1,4))

    def coeff_cb(self, msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.z = msg.data[2]
        self.radius = msg.data[6]
        arr = np.array([self.x, self.y, self.z, self.radius]).reshape(1,4)
        if self.data.size == 0:
            self.data = arr
        else:
            self.data = np.vstack([self.data, arr])
        if self.data.shape[0] >= self.window_size:
            self.data = self.data[-self.window_size:, :]
            self.means = np.mean(self.data, axis=0)
            self.medians = np.median(self.data, axis=0)
            self.stds = np.std(self.data, axis=0)
        else:
            self.means = np.mean(self.data, axis=0)
            self.medians = np.median(self.data, axis=0)
            self.stds = np.std(self.data, axis=0)
        
        self.stats.means.data = self.means
        self.stats.medians.data = self.medians
        self.stats.stds.data = self.stds
        self.stat_pub.publish(self.stats)

if __name__ == "__main__":
    try:
        RealtimeStatistics()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt!")