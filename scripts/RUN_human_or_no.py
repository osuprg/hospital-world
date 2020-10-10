#!/usr/bin/env python3

import rospy

# Brings in the .action file and messages used by the move base action
from geometry_msgs.msg import PoseWithCovarianceStamped
import dynamic_reconfigure.client as dynam
from std_msgs.msg import Bool, Float64, String
import atexit
from random import random

# CHANGE THESE TO POINT TO YOUR PARAMETERS FILE INFO
import STRUCT_hospital_graph_class as HospGraph