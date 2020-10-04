#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client as dynam
from std_msgs.msg import Bool, Float64, String
import atexit
from random import random

# CHANGE THESE TO POINT TO YOUR PARAMETERS FILE INFO
import STRUCT_hospital_v1_parameters as Parameters
from STRUCT_hospital_v1_parameters import HospitalParameters


class HumansPresent:
    def __init__(self, nodes_dict):
        self.nodes_dict = nodes_dict

        # Keeps track of current and prior node to check when it changes
        self.current_node = None
        self.prior_node = None

        self.condition = None
        self.human_status = None

        self.turtlebot_default_vel = 0.22

        self.human_conditions = [[0.0, 1.0],   # Humans present 0% of the time; speed at 100% normal
                                 [0.7, 0.9],   # Humans present 70% of the time, speed at 90% normal
                                 [0.2, 0.75]]  # Humans present 20% of the time, speed at 75% normal

        self.human_pub = rospy.Publisher('humans_or_no', Bool, queue_size=10)
        self.vel_pub = rospy.Publisher('velocity', Float64, queue_size=10)
        self.dynam_client = dynam.Client('move_base/DWAPlannerROS')

    def determine_state(self, msg):
        # Update current and prior nodes
        self.prior_node = self.current_node
        self.current_node = msg.data

        # Checks to see if the node changed and that this is not the first loop
        if self.prior_node != self.current_node:
            self.condition = int(self.nodes_dict[self.current_node][-1])
            self.human_or_no()
            self.set_robot_vel()


    def human_or_no(self):
        dice_roll = random()
        print(dice_roll)
        print(self.human_conditions[self.condition][0])
        if dice_roll < self.human_conditions[self.condition][0]:
            self.human_status = True
        else:
            self.human_status = False

        rospy.loginfo(self.human_status)
        self.human_pub.publish(self.human_status)

    def set_robot_vel(self):
        default_vel = 0.22  # For Turtlebot3
        if self.human_status:
            velocity = self.human_conditions[self.condition][1] * default_vel
        else:
            velocity = default_vel

        new_values = {'max_vel_x': velocity, 'min_vel_x': -velocity}
        self.dynam_client.update_configuration(new_values)

        rospy.loginfo(velocity)
        self.vel_pub.publish(velocity)



if __name__ == "__main__":
    rospy.init_node('is_anybody_out_there_py')

    path_to_param_file = '/home/toothless/workspaces/research_ws/src/hospital-world/scripts/STRUCT_hospital_v1_param_pickle'
    p = Parameters.unpickle_it(path_to_param_file).nodes_dict

    h = HumansPresent(p)

    # Subscribe to the topic that publishes which node the robot is in
    node_sub = rospy.Subscriber('node_in_graph', String, h.determine_state)

    rospy.spin()
