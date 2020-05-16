#!/usr/bin/env python

import rospy

# Brings in the .action file and messages used by the move base action
from geometry_msgs.msg import PoseWithCovarianceStamped
from hospital_graph_class import HospitalGraph
from std_msgs.msg import String

from two_rooms_v3_parameters import TwoRoomsParameters


class RobotNodeInfo:
    def __init__(self, p):
        self.current_pose = None
        self.current_node = None

        # Parameters file
        self.p = p

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
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_node = self.what_node()

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

        # Loop through once for doors
        for node in self.p.nodes_dict:
            if 'd' in node:
                if self.current_pose[0] in self.p.nodes_dict[node][0] and self.current_pose[1] in self.p.nodes_dict[node][1]:
                    return node

        # Loop through a second time for all other areas
        for node in self.p.nodes_dict:
            if 'd' in node:
                continue
            else:
                if self.current_pose[0] in self.p.nodes_dict[node][0] and self.current_pose[1] in self.p.nodes_dict[node][1]:
                    return node

        raise ValueError('Robot is out of this worlds')


if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node('node_in_graph_py')

        r = RobotNodeInfo(p=TwoRoomsParameters)

        amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, r.set_current_pose)

        rospy.spin()

