#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import STRUCT_hospital_graph_class as HospGraph


class CurrentPath:
    def __init__(self, hosp_graph):
        self.hosp_graph = hosp_graph
        self.initial_pose = hosp_graph.graph['initial_pose']
        self.poses = None
        self.plan = None
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
            prior_node = node

        for i in range(len(self.plan) - 1):
            # print(self.plan[i])
            # print([x for x in self.hosp_graph.neighbors(self.plan[i+1])])
            if not self.plan[i] in self.hosp_graph.neighbors(self.plan[i+1]):
                print("{} and {} are not connected".format(self.plan[i], self.plan[i+1]))

        if self.prior_plan and self.plan != [x for x in self.plan if x in self.prior_plan]:
            print(self.plan)
            print('###########################')

    def what_node(self, point):
        """
       Door nodes are checked first - they are within the bounds of rooms / halls so doors must supersede other areas

        Args:
            p: parameters
            loc: current x, y location to check

        Updates self.prior_node and self.current_node

        """

        # Update current node from position
        for node in self.hosp_graph.nodes():
            # Check the current pose against the location of all nodes to see which one it is in currently
            if point[0] in self.hosp_graph.nodes[node]['node_loc'][0] \
                    and point[1] in self.hosp_graph.nodes[node]['node_loc'][1]:
                return node

        return None


if __name__ == "__main__":
    rospy.init_node('try_it_py')

    path_to_pickle = rospy.get_param('graph_file_path')

    hosp_graph = HospGraph.unpickle_it(path_to_pickle)
    path = CurrentPath(hosp_graph)

    # Subscribe to the topic that publishes which node the robot is in
    node_sub = rospy.Subscriber('move_base/NavfnROS/plan', Path, path.get_plan)
    #
    # new_plan_sub = rospy.Subscriber('new_plan_pub', String, path.get_plan)
    rospy.spin()

