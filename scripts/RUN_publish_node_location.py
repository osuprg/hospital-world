#!/usr/bin/env python3

import rospy

# Brings in the .action file and messages used by the move base action
from geometry_msgs.msg import PoseWithCovarianceStamped
import dynamic_reconfigure.client as dynam
from std_msgs.msg import Bool, Float64, String
import atexit
from random import random
from nav_msgs.msg import Path
from queue import Queue, Empty

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
        self.current_node_index = 0

        # Parameters file
        self.hosp_graph = hosp_graph
        self.initial_pose = hosp_graph.graph['initial_pose']

        # Publishes the robot's current node to this topic
        self.node_pub = rospy.Publisher('node_in_graph', String, queue_size=1)
        # self.human_status_pub = rospy.Publisher('human_status', String, queue_size=1)

        # Which condition is the node / hall under
        self.condition = None
        # self.human_conditions = [[0.0, 1.0],  # Humans present 0% of the time; speed at 100% normal
        #                          [0.7, 0.9],  # Humans present 70% of the time, speed at 90% normal
        #                          [0.2, 0.75]]  # Humans present 20% of the time, speed at 75% normal

        self.human_conditions = [[0.0, 1.0],  # Humans present 0% of the time; speed at 100% normal
                                 [0.4, 0.75],  # Humans present 40% of the time, speed at 75% normal
                                 [0.8, 0.50]]  # Humans present 80% of the time, speed at 50% normal

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
        self.prior_plan = None

    def get_plan(self, msg):
        # Keep the prior plan for comparison
        if self.plan:
            self.prior_plan = self.plan.copy()

        # Re-initialize plan and prior node
        plan = []
        prior_node = None

        for ind_pose in msg.poses:
            point = (ind_pose.pose.position.x + self.initial_pose[0], ind_pose.pose.position.y + self.initial_pose[1])
            node = self.what_node(point)
            if node and node != prior_node:
                plan.append(node)
            prior_node = node

        # Check to make sure the full path is connected
        # for i in range(len(plan) - 1):
        #     # print(self.plan[i])
        #     # print([x for x in self.hosp_graph.neighbors(self.plan[i+1])])
        #     if not plan[i] in self.hosp_graph.neighbors(plan[i+1]):
        #         print("{} and {} are not connected".format(plan[i], plan[i+1]))

        try:
            # Turn plans into strings to compare
            # This checks to see if it's just the plan updating as it moves along and eliminates earlier actions
            # or if it is actually a brand new plan
            plan_str = str(plan).strip('[]')
            prior_plan_str = str(self.prior_plan).strip('[]')

            if plan_str not in prior_plan_str:
                self.plan = plan.copy()
                self.current_node_index = 0
                self.current_node_q = self.plan[self.current_node_index]
                self.next_node_q = self.plan[self.current_node_index + 1]
                print('###########################')
                print(self.plan)
                print('###########################')

        # Catches the error from the first iteration where there is no prior plan
        # Yes, this is lazy. But I tried it a different way and it was way messier.
        except TypeError:
            self.current_node_q = self.plan[self.current_node_index]
            self.next_node_q = self.plan[self.current_node_index + 1]
            print('###########################')
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

    def find_in_list(self):
        for i in range(len(self.plan)):
            if self.plan[i] == self.current_node_msg:
                print(self.plan[i], self.current_node_msg)
                return i

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

        if not self.plan:
            return

        # If we have reached a new node
        if self.prior_node_msg != self.current_node_msg:

            # Update current node
            self.current_node_index += 1
            self.current_node_q = self.plan[self.current_node_index]

            # Update next node, unless it's the end of the list
            try:
                self.next_node_q = self.plan[self.current_node_index + 1]
            except IndexError:
                # End of the list
                self.next_node_q = None

            # If the message and current node in the list are not the same, correct it
            if self.current_node_msg != self.current_node_q:
                print("Current node message and queue are not the same")
                index = self.find_in_list()
                # print('index of current node in list', index)
                if index is not None:
                    self.current_node_q = self.plan[index]
                    try:
                        self.next_node_q = self.plan[index + 1]
                        self.current_node_index = index
                    except IndexError:
                        self.next_node_q = None
                    print('fixed q to match msg', self.current_node_msg, self.current_node_q)
                else:
                    rospy.loginfo("Current node is not in the plan")
                    self.current_node_q = None

            # If there is something to record
            if self.current_node_q and self.next_node_q:
                # Check to make sure there is an edge between them
                if not self.current_node_q in self.hosp_graph.neighbors(self.next_node_q):
                    print("{} and {} are not connected".format(self.current_node_q, self.next_node_q))
                    rospy.loginfo("{} and {} are not connected".format(str(self.current_node_q), str(self.next_node_q)))
                    self.node_pub.publish("Do not record")
                # If there is, then find the human condition, set the velocity, and publish the node info
                else:
                    self.human_or_no()
                    self.set_robot_vel()
                    rospy.loginfo(str(self.current_node_q) + ", " + str(self.next_node_q) + ", " + self.human_status)
                    self.node_pub.publish(
                        str(self.current_node_q) + ", " + str(self.next_node_q) + ", " + self.human_status)
            else:
                rospy.loginfo('No current or next node')
                self.node_pub.publish('Do not record')

    def human_or_no(self):
        self.condition = int(self.hosp_graph[self.current_node_q][self.next_node_q]['hum_cond'])
        print(self.condition)
        dice_roll = random()

        # print(dice_roll)
        # print(self.human_conditions[self.condition][0])

        if dice_roll < self.human_conditions[self.condition][0]:
            self.human_status = "_hum"
        else:
            self.human_status = "_no"

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
    rospy.init_node('node_in_graph_py')

    path_to_pickle = rospy.get_param('graph_file_path')
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    while not rospy.is_shutdown():
        r = RobotNodeInfo(hosp_graph)

        plan_sub = rospy.Subscriber('move_base/NavfnROS/plan', Path, r.get_plan)
        amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, r.set_current_pose)
        rospy.spin()
