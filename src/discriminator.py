#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray

import math
import pandas as pd
import csv

import itertools
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import matplotlib as mpl
from sklearn import mixture

color_iter = itertools.cycle(["navy", "c", "cornflowerblue", "gold", "darkorange"])
gaus_threshold = ;

def callback(data):
    print("start discriminator")
    print(data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/apu_transparent_detection_node/feature", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    print("start dis")
    listener()
