#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from random import uniform, choice

# CHANGE THIS TO POINT TO YOUR PARAMETERS FILE
from two_rooms_v3_parameters import TwoRoomsParameters as p


# Name inspired by this artist - http://www.robotsinrowboats.com/
class MoveRobotAround:
    def __init__(self, param):
        self.current_position = None        # Keeps track of robot's current x,y position
        self.current_node = None            # Keeps track of what node the robot is in

        self.next_goal = (0.0, 0.0)         # Initialize robot's initial location
        self.prior_goal = (0.0, 0.0)        # Initialize robot's prior goal with its starting position
        self.break_goal = (-3.16, 8.22)     # Goal to test the move_base- should be outside the bounds
        self.result_from_path = 3           # Result of a successful path

        self.p = param                      # Parameters of given world

        self.rooms = []                     # List of rooms - used to determine next goal
        self._init_rooms_list()             # Initialize list of rooms

    def _init_rooms_list(self):
        # Initilaize list of rooms
        self.rooms = ['r' + '%02d' % i for i in range(self.p.num_rooms)]

    def set_current_pose(self, msg):
        # Get the current position of the robot and store it
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def set_current_node(self, msg):
        # Saves robot's current node after listening to topic that broadcasts that data
        self.current_node = msg.data

    def select_new_goal(self):
        # Choose a different room to navigate to
        new_room = self.current_node
        while new_room == self.current_node:
            new_room = choice(self.rooms)

        valid = False

        while not valid:
            # Uniformly select a random point in the new room - numbers are offset by 0.5 so the goal is not close to a wall
            self.next_goal = (uniform(self.p.nodes_dict[new_room][0].low + 0.5, self.p.nodes_dict[new_room][0].high - 0.5),
                              uniform(self.p.nodes_dict[new_room][1].low + 0.5, self.p.nodes_dict[new_room][1].high - 0.5))

            # Check that it is not in a convex portion of the room
            # TODO: Change this to check ALL 'excl' portions of the map / find a better way to ignore those portions
            if not (self.next_goal[0] in self.p.nodes_dict['excl'][0] and self.next_goal[1] in self.p.nodes_dict['excl'][1]):
                valid = True

        # print("Next goal chosen:", self.next_goal)

    def movebase_client(self, redo=False):
        # Code originally copied from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        if not redo:
            # Select a new goal
            self.prior_goal = self.next_goal
            self.select_new_goal()
        else:
            # Go back to the last point
            self.next_goal = self.prior_goal

        print("new goal selected:", self.next_goal)
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Move 0.5 meters forward along the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = self.next_goal[0]
        goal.target_pose.pose.position.y = self.next_goal[1]
        goal.target_pose.pose.position.z = 0.0

        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()

        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            # rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_state()


if __name__ == '__main__':
    rospy.init_node('movebase_client_py')

    # Make a rowing robot
    rowboat_robot = MoveRobotAround(p)

    # Subscribe to a bunch of stuff to get a bunch of info.
    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, rowboat_robot.set_current_pose)
    graph_node_sub = rospy.Subscriber('node_in_graph', String, rowboat_robot.set_current_node)

    # Ugh.
    # This was here from before I added in action server stuff and was writing the subscribers.
    # I spent two days figuring out why the action server wasn't available.
    # Through some random github issue answer, I saw a suggestion to remove a similar line to this, which solved it.
    # Leaving it in so future me knows not to make the same mistake and spend days on end debugging.
    # You're welcome, future me.
    # rospy.spin()

    iteration = 0
    total_iterations = 20

    # Redo is a flag that, if true, sends the robot back to the previous point because something went wrong
    redo = False

    try:
        while iteration < total_iterations:

            # Tried to break it in such a way that I could catch it. Turns out it's too good.
            # It either succeeds or can't reach the goal.
            rowboat_robot.final_it = False

            # if iteration == total_iterations - 2:
            #     # Out of range
            #     rowboat_robot.final_it = True

            print("##############")
            print("Iteration {}".format(iteration))
            print("##############")

            result = rowboat_robot.movebase_client(redo)

            if result:

                if result != 3:
                    rowboat_robot.result_from_path = result
                    redo = True
                    iteration += 1
                    print("Robot did not execute proper path")

                else:
                    redo = False
                    iteration += 1
                    print("Arrived at:", rowboat_robot.current_position)
                    rospy.loginfo("Goal execution done!")

            else:
                # For when something goes terribly wrong
                print("Something has gone terribly wrong")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        raise

    print("done!")
