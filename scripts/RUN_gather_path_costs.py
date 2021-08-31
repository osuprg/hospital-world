#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

# python things
import atexit
from numpy import arctan2
from squaternion import Quaternion
from time import time
import pickle

# custom things
import STRUCT_hospital_graph_class as HospGraph


paths_01 = [['Not slow', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'r02_d00', 'r03_d00', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00','r09_d00', 'r10_d00', 'r10']],
            ['Delicate', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'r02_d00', 'r03_d00', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00', 'r09_d00', 'r10_d00', 'r10']],
            # ['See people', ['r00', 'r00_d00', 'r01_d00', 'h00', 'h03', 'r13_d00', 'r12_d00', 'r11_d00', 'r10_d00', 'r10']],
            ['After hours', ['r00', 'r00_d00', 'r01_d00', 'r01_d01', 'h01', 'h02', 'r11_d00', 'r10_d00', 'r10']]]

paths_02 = [['Not slow', ['r01', 'r01_d01', 'r02_d00', 'r03_d00', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00', 'r08']],
            ['Delicate', ['r01', 'r01_d01', 'h01', 'h02', 'r10_d00', 'r09_d00', 'r08_d00', 'r08']],
            # ['See people', ['r01', 'r01_d01', 'h00', 'h03', 'r13_d00', 'r12_d00', 'r11_d00', 'r10_d00', 'r09_d00', 'r08_d00', 'r08']],
            ['After hours', ['r01', 'r01_d01', 'r02_d00', 'r03_d00', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00', 'r08']]]

paths_03 = [['Not slow', ['r01', 'r01_d01', 'r02_d00', 'r03_d00', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00', 'r09_d00', 'r10_d00','r10']],
            ['Delicate', ['r01', 'r01_d01', 'h01', 'h02', 'r10_d00', 'r10']],
            # ['See people', ['r01', 'r01_d01', 'h00', 'h03', 'r13_d00', 'r12_d00', 'r11_d00', 'r10_d00', 'r10']],
            ['After hours', ['r01', 'r01_d01', 'h01', 'h02', 'r10_d00', 'r10']]]

paths_04 = [['Not slow', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'h01', 'h02', 'r11_d00', 'r12_d00', 'r13_d00', 'r13']],
            ['Delicate', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'h01', 'h02', 'r11_d00', 'r12_d00', 'r13_d00', 'r13']],
            # ['See people', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'r01_d01', 'h00', 'h03', 'r13_d00', 'r13']],
            ['After hours', ['r05', 'r05_d00', 'r06_d00', 'r07_d00', 'r08_d00', 'r09_d00', 'r10_d00', 'r11_d00', 'r12_d00', 'r13_d00', 'r13']]]

paths_05 = [['Not slow', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r18_d00', 'r17_d00', 'r16_d00', 'r15_d00', 'r15']],
            ['Delicate', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r18_d00', 'r17_d00', 'r16_d00', 'r15_d00', 'r15']],
            # ['See people', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'r01_d01', 'h00', 'h03', 'r14_d00', 'r15_d00', 'r15']],
            ['After hours', ['r05', 'r05_d00', 'r03_d00', 'r02_d00', 'r01_d01', 'h00', 'h03', 'r14_d00', 'r15_d00', 'r15']]]

paths_06 = [['Not slow',['r08', 'r08_d00', 'r07_d00', 'r06_d00', 'r05_d00', 'r03_d00', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r19']],
            ['Delicate', ['r08', 'r08_d00', 'r07_d00', 'r06_d00', 'r05_d00', 'r03_d00', 'r02_d00', 'r01_d01', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r19']],
            ['See people', ['r08', 'r08_d00', 'r09_d00', 'r10_d00', 'r11_d00', 'r12_d00', 'r13_d00', 'h03', 'h00', 'r01_d00', 'r00_d00', 'r20_d00', 'r19_d00', 'r19']],
            ['After hours', ['r08', 'r08_d00', 'r09_d00', 'r10_d00', 'r11_d00', 'r12_d00', 'r13_d00', 'r14_d00', 'r15_d00', 'r16_d00', 'r17_d00', 'r18_d00', 'r19_d00', 'r19']]]

paths_to_plot = [['r00', 'r10', paths_01],
                 ['r01', 'r08', paths_02],
                 ['r01', 'r10', paths_03],
                 ['r05', 'r13', paths_04],
                 ['r05', 'r15', paths_05],
                 ['r08', 'r19', paths_06]]


class MoveRobotAround:
    def __init__(self, hosp_graph):
        self.current_position = None  # Keeps track of robot's current x,y position
        self.current_node = None  # Keeps track of what node the robot is in
        self.next_node = None

        # CHANGE THIS TOO
        self.initial_pose = hosp_graph.graph['initial_pose']

        self.next_goal = (0.0, 0.0)  # Initialize robot's initial location
        self.goal_orientation = [0, 0, 0, 1]
        self.prior_goal = (0.0, 0.0)  # Initialize robot's prior goal with its starting position
        self.result_from_path = 3  # Result of a successful path
        self.path_times = []

        self.hosp_graph = hosp_graph  # Parameters of given world
        self.path_data_pickle = "/home/toothless/workspaces/research_ws/src/hospital-world/pickles/path_data_from_move_base-02-24-2021"

        self.new_plan_pub = rospy.Publisher('new_plan_pub', String, queue_size=10)
        self.global_planner = rospy.get_param('global_planner_choice')
        rospy.loginfo('Global planner chosen: {}'.format(self.global_planner))
        self.plan_pub = rospy.Publisher('custom_global_plan', String, queue_size=1)

    def set_current_pose(self, msg):

        # Get the current position of the robot and store it
        self.current_position = (msg.pose.pose.position.x + self.initial_pose[0],
                                 msg.pose.pose.position.y + self.initial_pose[1])
        self.current_node = self.what_node(self.current_position)

    def what_node(self, point):

        # Update current node from position
        for node in self.hosp_graph.nodes():
            # Check the current pose against the location of all nodes to see which one it is in currently
            if point[0] in self.hosp_graph.nodes[node]['node_loc'][0] \
                    and point[1] in self.hosp_graph.nodes[node]['node_loc'][1]:
                return node
        return None

    def select_point_in_room(self, new_room):
        valid = False

        # Select point in the center of the node
        x_mid = (self.hosp_graph.nodes[new_room]['node_loc'][0].low +
                 self.hosp_graph.nodes[new_room]['node_loc'][0].high) / 2
        y_mid = (self.hosp_graph.nodes[new_room]['node_loc'][1].low +
                 self.hosp_graph.nodes[new_room]['node_loc'][1].high) / 2
        self.next_goal = (x_mid, y_mid)

    def set_goal_orientation(self, orientation):

        if orientation == 'up':
            del_y = 1
            del_x = 0
        elif orientation == "down":
            del_y = -1
            del_x = 0
        elif orientation == "left":
            del_y = 0
            del_x = -1
        elif orientation == "right":
            del_y = 0
            del_x = 1
        else:
            raise ValueError("you're dumb.")
        new_yaw = arctan2(del_y, del_x)
        self.goal_orientation = Quaternion.from_euler(0, 0, new_yaw)
        rospy.loginfo('New orientation: {}'.format(self.goal_orientation))

    def movebase_client(self):
        # Code originally copied from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
        rospy.loginfo('New nav point chosen: {}'.format(self.next_goal))

        # rospy.loginfo("Move_base called")
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Move 0.5 meters forward along the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = self.next_goal[0] - self.initial_pose[0]
        goal.target_pose.pose.position.y = self.next_goal[1] - self.initial_pose[1]
        goal.target_pose.pose.position.z = 0.0

        # print("New goal (amcl): {0} {1}".format(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))

        # Squaternion defines Quaternion as (w, x, y, z) NOT (x, y, z, w)
        goal.target_pose.pose.orientation.x = self.goal_orientation[1]
        goal.target_pose.pose.orientation.y = self.goal_orientation[2]
        goal.target_pose.pose.orientation.z = self.goal_orientation[3]
        goal.target_pose.pose.orientation.w = self.goal_orientation[0]

        # Sends the goal to the action server.
        client.send_goal(goal)

        for i in range(10):
            self.new_plan_pub.publish('yes')

        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()

        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            # rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_state()

    def use_custom_planner(self, path):
        case = 0

        if self.current_node == path[0][0][0]:
            case = 0
        elif self.current_node == path[1][0][0]:
            case = 1

        for i in range(len(path[case])):
            node = path[case][i][0]
            direction = path[case][i][1]
            if self.current_node == node:
                continue

            self.select_point_in_room(node)
            self.set_goal_orientation(direction)
            result = self.movebase_client()

    def collect_path_data(self, path):
        iter = 0
        tot_iter = 1000
        try:
            while iter < tot_iter and not rospy.is_shutdown():
                print("##############")
                print("Iteration {}".format(iter))
                rospy.loginfo('Iteration {}'.format(iter))
                start_time = time()
                self.use_custom_planner(path)
                path_time = time() - start_time
                rospy.loginfo('Path took {:0.2f} sec'.format(path_time))
                self.path_times.append(path_time)

                if not iter % 20:
                    pickle_it(self.path_times, self.path_data_pickle)

                iter += 1

        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
            pickle_it(self.path_times, self.path_data_pickle)
            exit()

def pickle_it(obj_to_pickle, file_path_and_name):
    with open(file_path_and_name, 'wb') as f:
        pickle.dump(obj_to_pickle, f)
    print("Saving information to file named {}".format(file_path_and_name))

if __name__ == '__main__':
    rospy.init_node('gather_path_data')
    rospy.loginfo('gather_path_data node started')

    path_to_pickle = rospy.get_param('graph_file_path')
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    # Make a rowing robot
    rowboat_robot = MoveRobotAround(hosp_graph)

    # Subscribe to a bunch of stuff to get a bunch of info.
    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, rowboat_robot.set_current_pose)
    # graph_node_sub = rospy.Subscriber('node_in_graph', String, rowboat_robot.set_current_node)
    atexit.register(pickle_it, obj_to_pickle=rowboat_robot.path_times, file_path_and_name=rowboat_robot.path_data_pickle)

    # This is here so we don't try to run trials before getting a message from AMCL
    rospy.sleep(1)

    path_to_use = [[['r00', 'down'], ['h01', 'down'], ['r10_d00', 'down'], ['r10', 'down']],
                   [['r10', 'up'], ['h02', 'up'], ['h01', 'up'], ['r00', 'up']]]

    # path_to_use = [[['r05', 'left'], ['h01', 'down'], ['r11_d00', 'left'], ['r13', 'down']],
    #                [['r13', 'up'], ['r11_d00', 'up'], ['h01', 'up'], ['r05', 'right']]]

    rowboat_robot.collect_path_data(path_to_use)

