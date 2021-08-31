#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from random import uniform, choice

# python things
from numpy import arctan2
from squaternion import Quaternion

# custom things
import STRUCT_hospital_graph_class as HospGraph
import RUN_custom_global_plan_sampling as GlobalSample


class MoveRobotAround:
    def __init__(self, hosp_graph):
        self.current_position = None        # Keeps track of robot's current x,y position
        self.current_node = None            # Keeps track of what node the robot is in
        self.next_node = None

        # CHANGE THIS TOO
        self.initial_pose = hosp_graph.graph['initial_pose']

        self.next_goal = (0.0, 0.0)         # Initialize robot's initial location
        self.goal_orientation = [0, 0, 0, 1]
        self.prior_goal = (0.0, 0.0)        # Initialize robot's prior goal with its starting position
        self.break_goal = (-3.16, 8.22)     # Goal to test the move_base- should be outside the bounds
        self.result_from_path = 3           # Result of a successful path

        self.hosp_graph = hosp_graph        # Parameters of given world

        self.rooms = []                     # List of rooms - used to determine next goal
        self._init_rooms_list()             # Initialize list of rooms
        self.new_plan_pub = rospy.Publisher('new_plan_pub', String, queue_size=10)
        self.global_planner = rospy.get_param('global_planner_choice')
        rospy.loginfo('Global planner chosen: {}'.format(self.global_planner))

        self.planner = GlobalSample.SamplingPlannerClass(self.hosp_graph)
        self.plan_pub = rospy.Publisher('custom_global_plan', String, queue_size=1)

    def _init_rooms_list(self):
        # Initilaize list of rooms
        self.rooms = ['r' + '%02d' % i for i in range(self.hosp_graph.graph['num_rooms'])]

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

    def select_new_goal(self):
        # Choose a different room to navigate to
        new_room = self.current_node
        trigger = False
        while new_room == self.current_node or not trigger:
            new_room = choice(self.rooms)
            rospy.loginfo('current room: {}, new room selected: {}'.format(self.current_node, new_room))
            if self.current_node:
                new_rm_num = int(new_room[1:3])
                curr_rm_num = int(self.current_node[1:3])
                if new_rm_num != (curr_rm_num + 1) % 21 and curr_rm_num != (new_rm_num + 1) % 21:
                    trigger = True
            else:
                trigger = True

        self.next_node = new_room
        # rospy.loginfo('New node chosen: {}'.format(new_room))
        self.next_goal = self.select_point_in_room(new_room)

    def select_point_in_room(self, new_room):
        valid = False

        while not valid:
            # Select point in the center of the node
            x_mid = (self.hosp_graph.nodes[new_room]['node_loc'][0].low +
                     self.hosp_graph.nodes[new_room]['node_loc'][0].high) /2
            y_mid = (self.hosp_graph.nodes[new_room]['node_loc'][1].low +
                     self.hosp_graph.nodes[new_room]['node_loc'][1].high) /2
            next_goal = (x_mid, y_mid)

            # Check that it is not in a convex portion of the room
            # TODO: Change this to check ALL 'excl' portions of the map / find a better way to ignore those portions
            exclusions = [key for key in self.hosp_graph.nodes() if 'ex' in key]
            trigger = False
            for key in exclusions:
                if next_goal[0] in self.hosp_graph.nodes[key]['node_loc'][0] and \
                        next_goal[1] in self.hosp_graph.nodes[key]['node_loc'][1]:
                    trigger = True

            # If it is not in an exclusion zone, then exit loop and return the goal
            if not trigger:
                valid = True
                return next_goal

    def set_goal_orientation(self, n0, n1):
        # Sets the goal orientation to be the vector between the prior and current nodes
        # Currently using this as a proxy to get the robot to stay relatively in line with the path

        p0 = self.select_point_in_room(new_room=n0)
        p1 = self.select_point_in_room(new_room=n1)

        del_x = p1[0] - p0[0]
        del_y = p1[1] - p0[1]

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

    def use_custom_planner(self):
        #TODO: This needs A LOT of error handling
        self.select_new_goal()
        self.planner.re_init()

        if self.current_node and self.next_node:
            rospy.loginfo('Current node {}, new node {}'.format(self.current_node, self.next_node))
            path = self.planner.get_path(1000, self.current_node, self.next_node)

            self.plan_pub.publish(str(path))
            rospy.loginfo("new plan: {}".format(path))

            if path:

                for i in range(len(path)):
                    rospy.loginfo('going to {}'.format(path[i]))
                    self.next_goal = self.select_point_in_room(path[i])
                    rospy.loginfo('new point selected {}'.format(self.next_goal))
                    try:
                        self.set_goal_orientation(path[i], path[i + 1])
                    except IndexError:
                        self.goal_orientation = [0, 0, 0, 1]

                    self.movebase_client()

                    if self.current_node != path[i]:
                        self.movebase_client()
            else:
                rospy.loginfo("Path was not received from planner")
        return True

    def run_trials(self):
        rospy.loginfo('starting trial runs')
        iteration = 0
        total_iterations = 1000000

        try:
            while iteration < total_iterations and not rospy.is_shutdown():

                # Tried to break it in such a way that I could catch it. Turns out it's too good.
                # It either succeeds or can't reach the goal.
                self.final_it = False

                # if iteration == total_iterations - 2:
                #     # Out of range
                #     rowboat_robot.final_it = True

                print("##############")
                print("Iteration {}".format(iteration))
                rospy.loginfo('Iteration {}'.format(iteration))
                print("##############")

                if self.global_planner == "move_base":
                    rospy.loginfo('running standard move_base')
                    self.select_new_goal()
                    result = self.movebase_client()
                elif self.global_planner == "sampling":
                    rospy.loginfo('running sampling based global planner')
                    result = self.use_custom_planner()

                if result:

                    if result != 3:
                        self.result_from_path = result
                        redo = True
                        iteration += 1
                        rospy.loginfo("Robot did not execute proper path")

                    else:
                        redo = False
                        iteration += 1
                        rospy.loginfo("Arrived at: {}".format(self.current_position))
                        rospy.loginfo("Goal execution done!")

                else:
                    # For when something goes terribly wrong
                    rospy.loginfo("Something has gone terribly wrong")
                    break

        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
            exit()

        print("done!")

    def path_orientation(self, node):
        x_mid = (self.hosp_graph.nodes[node]['node_loc'][0].low +
                 self.hosp_graph.nodes[node]['node_loc'][0].high) / 2
        y_mid = (self.hosp_graph.nodes[node]['node_loc'][1].low +
                 self.hosp_graph.nodes[node]['node_loc'][1].high) / 2
        next_goal = (x_mid, y_mid)

    def collect_path_data(self, path):
        iter = 0
        tot_iter = 1000

        try:
            while iter < tot_iter and not rospy.is_shutdown():
                print("##############")
                print("Iteration {}".format(iter))
                rospy.loginfo('Iteration {}'.format(iter))
                print("##############")

                if self.global_planner == "move_base":
                    rospy.loginfo('running standard move_base')
                    self.select_new_goal()
                    result = self.movebase_client()
                elif self.global_planner == "sampling":
                    rospy.loginfo('running sampling based global planner')
                    result = self.use_custom_planner()

                if result:

                    if result != 3:
                        self.result_from_path = result
                        redo = True
                        iteration += 1
                        rospy.loginfo("Robot did not execute proper path")

                    else:
                        redo = False
                        iteration += 1
                        rospy.loginfo("Arrived at: {}".format(self.current_position))
                        rospy.loginfo("Goal execution done!")

                else:
                    # For when something goes terribly wrong
                    rospy.loginfo("Something has gone terribly wrong")
                    break

        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
            exit()

if __name__ == '__main__':
    rospy.init_node('run_trials_py')
    rospy.loginfo('run_trials_py node started')

    path_to_pickle = rospy.get_param('graph_file_path')
    hosp_graph = HospGraph.unpickle_it(path_to_pickle)

    # Make a rowing robot
    rowboat_robot = MoveRobotAround(hosp_graph)

    # Subscribe to a bunch of stuff to get a bunch of info.
    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, rowboat_robot.set_current_pose)
    # graph_node_sub = rospy.Subscriber('node_in_graph', String, rowboat_robot.set_current_node)

    # This is here so we don't try to run trials before getting a message from AMCL
    rospy.sleep(1)

    # rowboat_robot.run_trials()

    # Ugh.
    # This was here from before I added in action server stuff and was writing the subscribers.
    # I spent two days figuring out why the action server wasn't available.
    # Through some random github issue answer, I saw a suggestion to remove a similar line to this, which solved it.
    # Leaving it in so future me knows not to make the same mistake and spend days on end debugging.
    # You're welcome, future me.
    # rospy.spin()
