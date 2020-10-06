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


class RobotNodeInfo:
    def __init__(self, hosp_graph):
        self.current_pose = None

        # Keeps track of current and prior node to check when it changes
        self.current_node = None
        self.prior_node = None

        # Parameters file
        self.hosp_graph = hosp_graph

        # Publishes the robot's current node to this topic
        self.node_pub = rospy.Publisher('node_in_graph', String, queue_size=10)

        # Which condition is the node / hall under
        self.condition = None
        self.human_conditions = [[0.0, 1.0],   # Humans present 0% of the time; speed at 100% normal
                                 [0.7, 0.9],   # Humans present 70% of the time, speed at 90% normal
                                 [0.2, 0.75]]  # Humans present 20% of the time, speed at 75% normal

        self.initial_pose = (1.0, 2.0)
        # Boolean for whether or not humans are present in the current node
        self.human_status = None

        # Default velocity of the turtlebot
        self.turtlebot_default_vel = 0.22

        # Publishers for velocity and human state
        # self.human_pub = rospy.Publisher('humans_or_no', Bool, queue_size=10)
        self.vel_pub = rospy.Publisher('velocity', Float64, queue_size=10)

        # This is the magic sauce that allows us to change the max velocity on the fly
        self.dynam_client = dynam.Client('move_base/DWAPlannerROS')

    def set_current_pose(self, msg):
        # Set the robot's current location and determine what node it is in
        self.current_pose = (msg.pose.pose.position.x + self.initial_pose[0],
                             msg.pose.pose.position.y + self.initial_pose[1])
        self.what_node()

        # Publish node info
        rospy.loginfo(self.current_node)
        self.node_pub.publish(self.current_node)

        # If the robot has moved to a new node, check whether humans are present then set new velocity
        if self.prior_node != self.current_node:
            # TODO: Change this to be based on edge, not node
            self.condition = int(self.hosp_graph.nodes[self.current_node]['hum_cond'])
            self.human_or_no()
            self.set_robot_vel()

    def what_node(self):
        """
       Door nodes are checked first - they are within the bounds of rooms / halls so doors must supersede other areas

        Args:
            p: parameters
            loc: current x, y location to check

        Updates self.prior_node and self.current_node

        """
        self.prior_node = self.current_node

        # Update current node from position
        for node in self.p.nodes_dict:
            if self.current_pose[0] in self.p.nodes_dict[node][0] and self.current_pose[1] in self.p.nodes_dict[node][1]:
                self.current_node = node

    def human_or_no(self):
        dice_roll = random()

        # print(dice_roll)
        # print(self.human_conditions[self.condition][0])

        if dice_roll < self.human_conditions[self.condition][0]:
            self.human_status = "_hum"
        else:
            self.human_status = "_no"

        rospy.loginfo(self.human_status)
        self.node_pub.publish(self.human_status)

    def set_robot_vel(self):
        if self.human_status == "_hum":
            velocity = self.human_conditions[self.condition][1] * self.turtlebot_default_vel
        else:
            velocity = self.turtlebot_default_vel

        new_values = {'max_vel_x': velocity, 'min_vel_x': -velocity}
        self.dynam_client.update_configuration(new_values)

        rospy.loginfo(velocity)
        self.vel_pub.publish(velocity)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node('node_in_graph_py')

        path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle'

        hosp_graph = HospGraph.unpickle_it(path_to_pickle)

        r = RobotNodeInfo(hosp_graph)

        amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, r.set_current_pose)

        rospy.spin()

