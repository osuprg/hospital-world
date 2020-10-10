#!/usr/bin/env python3

import rospy

# Brings in the .action file and messages used by the move base action
from geometry_msgs.msg import PoseWithCovarianceStamped
import dynamic_reconfigure.client as dynam
from std_msgs.msg import Bool, Float64, String
import atexit
from random import random
from nav_msgs.msg import Path
from queue import Queue

# CHANGE THESE TO POINT TO YOUR PARAMETERS FILE INFO
import STRUCT_hospital_graph_class as HospGraph


class RobotNodeInfo:
    def __init__(self, hosp_graph):
        self.current_pose = None

        # Keeps track of current and prior node to check when it changes
        self.current_node_msg = None
        self.prior_node_msg = None
        self.current_node_q = None
        self.next_node_q = None

        # Parameters file
        self.hosp_graph = hosp_graph
        self.initial_pose = hosp_graph.graph['initial_pose']

        # Publishes the robot's current node to this topic
        self.node_pub = rospy.Publisher('node_in_graph', String, queue_size=10)

        # Which condition is the node / hall under
        self.condition = None
        self.human_conditions = [[0.0, 1.0],   # Humans present 0% of the time; speed at 100% normal
                                 [0.7, 0.9],   # Humans present 70% of the time, speed at 90% normal
                                 [0.2, 0.75]]  # Humans present 20% of the time, speed at 75% normal

        # Boolean for whether or not humans are present in the current node
        self.human_status = None

        # Default velocity of the turtlebot
        self.turtlebot_default_vel = 0.22

        # Publishers for velocity and human state
        # self.human_pub = rospy.Publisher('humans_or_no', Bool, queue_size=10)
        self.vel_pub = rospy.Publisher('velocity', Float64, queue_size=10)

        # This is the magic sauce that allows us to change the max velocity on the fly
        self.dynam_client = dynam.Client('move_base/DWAPlannerROS')

        self.poses = None
        self.plan = None
        self.plan_q = Queue()
        self.prior_plan = None

    def get_plan(self, msg):
        if self.plan:
            self.prior_plan = self.plan.copy()
        self.plan = []
        prior_node = None

        for ind_pose in msg.poses:
            point = (ind_pose.pose.position.x + self.initial_pose[0], ind_pose.pose.position.y + self.initial_pose[1])
            node = self.what_node(point)
            if node and node != prior_node:
                self.plan.append(node)
                self.plan_q.put(node)
            prior_node = node

        for i in range(len(self.plan) - 1):
            # print(self.plan[i])
            # print([x for x in self.hosp_graph.neighbors(self.plan[i+1])])
            if not self.plan[i] in self.hosp_graph.neighbors(self.plan[i+1]):
                print("{} and {} are not connected".format(self.plan[i], self.plan[i+1]))

        try:
            if self.plan != [x for x in self.plan if x in self.prior_plan]:
                self.current_node_q = self.plan_q.get()
                self.next_node_q = self.plan_q.get()
                print(self.plan)
                print('###########################')
        except TypeError:
            self.current_node_q = self.plan_q.get()
            self.next_node_q = self.plan_q.get()
            print(self.plan)
            print('###########################')

    def what_node(self, point):

        # Update current node from position
        for node in self.hosp_graph.nodes():
            # Check the current pose against the location of all nodes to see which one it is in currently
            if point[0] in self.hosp_graph.nodes[node]['node_loc'][0] \
                    and point[1] in self.hosp_graph.nodes[node]['node_loc'][1]:
                return node
        return None

    def set_current_pose(self, msg):
        # Set the robot's current location and determine what node it is in
        # Offsetting with initial pose translates from the world's coordinates to the robot's coordinates
        self.current_pose = (msg.pose.pose.position.x + self.initial_pose[0],
                             msg.pose.pose.position.y + self.initial_pose[1])

        self.prior_node_msg = self.current_node_msg

        current_node = self.what_node(self.current_pose)

        if current_node:
            self.current_node_msg = current_node

        # # Publish node info
        # rospy.loginfo(self.current_node_msg)
        # self.node_pub.publish(self.current_node_msg)

        # If the robot has moved to a new node, check whether humans are present then set new velocity
        if self.prior_node_msg != self.current_node_msg:

            self.current_node_q = self.next_node_q
            self.next_node_q = self.plan_q.get()

            print(self.current_node_msg, self.current_node_q)
            curr_node_info = str(self.current_node_msg + ", " + self.current_node_q)
            # Publish node info
            rospy.loginfo(curr_node_info)
            self.node_pub.publish(curr_node_info)

            if self.current_node_q != self.current_node_msg:
                print("something has gone terribly wrong")

            # self.condition = int(self.hosp_graph[self.current_node_q][self.next_node_q]['hum_cond'])
            # self.human_or_no()
            # self.set_robot_vel()

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

    path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle'
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    while not rospy.is_shutdown():
        rospy.init_node('node_in_graph_py')
        r = RobotNodeInfo(hosp_graph)
        # TODO: The logic here is fucked. It's not updating the plan queue once it gets the initial plan.
        #  Also the timing between getting the initial plan and figuring out where I am is off.
        plan_sub = rospy.Subscriber('move_base/NavfnROS/plan', Path, r.get_plan)
        amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, r.set_current_pose)
        rospy.spin()
