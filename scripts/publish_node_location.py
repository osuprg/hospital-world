#!/usr/bin/env python

import rospy

# Brings in the .action file and messages used by the move base action
from geometry_msgs.msg import PoseWithCovarianceStamped
from hospital_graph_class import HospitalGraph
from std_msgs.msg import String

# CHANGE THESE TO POINT TO YOUR PARAMETERS FILE INFO
from hospital_parameters import HospitalParameters as Parameters

class RobotNodeInfo:
    def __init__(self, param):
        self.current_pose = None
        self.current_node = None

        # Parameters file
        self.p = param

        # Prior node is not necessary based on current logic.
        # Leaving here in case I want to change this to only publish when the robot changes nodes.
        self.prior_node = None

        # Publishes the robot's current node to this topic
        self.pub = rospy.Publisher('node_in_graph', String, queue_size=10)

        # Don't currently need a hospital graph for this file, but leaving it in case it's necessary later
        # self.building = HospitalGraph(p.num_rooms, p.num_halls, p.extra_doors,
        #                               p.hall_door_links, p.extra_door_hall_links, p.connected_halls)

    def set_current_pose(self, msg):
        # Set the robot's current location and determine what node it is in
        self.current_pose = (msg.pose.pose.position.x + self.p.initial_pose[0],
                                 msg.pose.pose.position.y + self.p.initial_pose[1])
        self.what_node()

        # If the node changed, publish it to a topic
        # if self.prior_node != self.current_node:
        rospy.loginfo(self.current_node)
        self.pub.publish(self.current_node)

    def what_node(self):
        """
       Door nodes are checked first - they are within the bounds of rooms / halls so doors must supersede other areas

        Args:
            p: parameters
            loc: current x, y location to check

        Returns:
            node the robot is currently in

        """
        self.prior_node = self.current_node

        # Update if it has moved to a new node
        for node in self.p.nodes_dict:
            if self.current_pose[0] in self.p.nodes_dict[node][0] and self.current_pose[1] in self.p.nodes_dict[node][1]:
                self.current_node = node


if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node('node_in_graph_py')
        p = Parameters()
        r = RobotNodeInfo(p)

        amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, r.set_current_pose)

        rospy.spin()

