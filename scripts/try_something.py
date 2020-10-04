#!/usr/bin/env python2

import rospy
import STRUCT_interval_cust as cust_int

rospy.set_param('try01', 1)
rospy.set_param('dict', {'a': cust_int.Interval([0, 1]), 'b': cust_int.Interval([2, 3])})
