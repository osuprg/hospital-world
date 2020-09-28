import STRUCT_interval_cust as interval_cust
import STRUCT_hospital_graph_class as hospital_graph_class

"""
Takes in raw dimensions and parameters from text file. Converts it into a readable dictionary 
nodes_dict is in the format "node name": [[interval of x range], [interval of y range]]
Nodes_dict is checked against for transitions between nodes
Also builds out a dictionary of rooms to be used for choosing the next goal
"""


class HospitalParameters:
    def __init__(self):
        # "node name": [interval of x range, interval y range]
        self.nodes_dict = {}

        # Build rooms dictionary for choosing next target for trials
        self.rooms_dict = {}

        self.num_rooms = 21
        self.num_halls = 4

        self.initial_pose = (1.0, 2.0)

        self.hall_width = 2.0
        self.node_width = 0.5
        self.hall_node_width = 1.0

        self.extra_doors = ['r01']
        self.extra_door_hall_links = [('r01_d01b', 'h00'), ('r01_d01b', 'h01'), ('r01_d01b', 'r02_d00b'),
                                      ('r01_d01b', 'r01_d00b'), ('r03_d00b', 'r05_d00b'), ('r07_d00b', 'r09_d00b'),
                                      ('r15_d00b', 'r17_d00b')]

        # Associates rooms (door 0) with the adjacent hall - additional doors handled further down
        self.hall0_rooms = [0, 1]
        self.hall1_rooms = [2]
        self.hall2_rooms = [10, 11, 12]
        self.hall3_rooms = [13, 14, 15]
        self.hall_door_links = [self.hall0_rooms, self.hall1_rooms, self.hall2_rooms, self.hall3_rooms]

        self.connected_halls = [('h00', 'h03'), ('h01', 'h02')]

        self.connected_rooms = True

        filename = '/hospital-world/scripts/STRUCT_hospital_v1_parameters_raw.txt'
        data = self._read_file(filename)

        self._build_nodes_dict(data)

    def _read_file(self, filename):
        f = open(filename, 'r')
        data = []
        for line in f:
            data.append(line)
        return data

    def _build_nodes_dict(self, data):

        for line in data:
            parts = line.split()
            node_info = []

            # Ignore comment lines
            if parts[0] is '#':
                continue

            # Convert numbers to floats
            for p in parts:
                try:
                    node_info.append(float(p))
                except ValueError:
                    node_info.append(p)

            if node_info[0][0] == 'r':
                self._add_room_nodes(node_info)
                self._add_door_nodes(node_info)

            elif node_info[0][0] == 'h':
                self._add_hall_nodes(node_info)

            elif node_info[0][0] == 'e':
                self._add_excl_nodes(node_info)

            else:
                print('not sure what to do with that node type:', node_info[0])

    def _add_room_nodes(self, node_info):
        # This will give the system x,y bounds for each room
        # This is used to choose where to autonomously drive next
        if node_info[1] is not 'x':
            self.rooms_dict[node_info[0]] = [interval_cust.Interval(node_info[1], node_info[2]),
                                             interval_cust.Interval(node_info[3], node_info[4])]
            self.nodes_dict[node_info[0]] = [interval_cust.Interval(node_info[1], node_info[2]),
                                             interval_cust.Interval(node_info[3], node_info[4])]

    def _add_door_nodes(self, node_info):
        # Handle rooms first
        # Rooms will get one node in the door and one spanning the hall

        # Handles rooms then extra doors
        if 'd' not in node_info[0]:
            name_a = node_info[0] + '_d00a'
            name_b = node_info[0] + '_d00b'
        else:
            name_a = node_info[0] + 'a'
            name_b = node_info[0] + 'b'

        [xmin, xmax] = node_info[5:7]
        [ymin, ymax] = node_info[7:]

        # 'a' is the node in the doorway
        xmin_a = xmin  # 5
        xmax_a = xmax  # 6
        ymin_a = ymin  # 16
        ymax_a = ymax  # 16

        # 'b' is the node just outside the doorway that spans the hall
        xmin_b = xmin  # 5
        xmax_b = xmax  # 6
        ymin_b = ymin  # 16
        ymax_b = ymax  # 16

        # Currently can't handle doors that are not parallel to an axis
        # Currently a door is listed as a line with no width
        # This adds width of 0.4m so it can detect when it's in that space
        # Vertical
        if xmin == xmax:

            # This is where things get very hand-coded (ugh)
            # TODO: Make this less world specific
            # Spans the hallway but doesn't overlap with the door node
            if xmin < 15:
                xmin_a = xmax_a - self.node_width
                xmin_b = xmax_a
                xmax_b = xmin_b + self.hall_width
            else:
                xmax_a = xmin_a + self.node_width
                xmax_b = xmin_a
                xmin_b = xmax_b - self.hall_width

            middle_y = (ymin_a + ymax_a) / 2
            ymin_b = middle_y - self.hall_node_width / 2
            ymax_b = middle_y + self.hall_node_width / 2

        # Horizontal
        elif ymin == ymax:

            # This is where things get very hand-coded (ugh)
            # TODO: Make this less world specific
            # Spans the hallway but doesn't overlap with the door node
            if ymin < 10:
                ymin_a = ymax_a - self.node_width
                ymin_b = ymax_a
                ymax_b = ymin_b + self.hall_width
            else:
                ymax_a = ymin_a + self.node_width
                ymax_b = ymin_a
                ymin_b = ymax_b - self.hall_width

            middle_x = (xmin_a + xmax_a) / 2
            xmin_b = middle_x - self.hall_node_width / 2
            xmax_b = middle_x + self.hall_node_width / 2

        else:
            print("something has gone wrong. One of these pairs should be ==")
            print(xmin, xmax, ymin, ymax)

        # Doorway
        self.nodes_dict[name_a] = [interval_cust.Interval(xmin_a, xmax_a),
                                   interval_cust.Interval(ymin_a, ymax_a)]

        # Hallway node outside doorway
        self.nodes_dict[name_b] = [interval_cust.Interval(xmin_b, xmax_b),
                                   interval_cust.Interval(ymin_b, ymax_b)]

        # print(name_a, self.nodes_dict[name_a])
        # print(name_b, self.nodes_dict[name_b])

    def _add_hall_nodes(self, node_info):
        [xmin, xmax] = node_info[5:7]
        [ymin, ymax] = node_info[7:]

        # Currently can't handle nodes that are not parallel to an axis
        # Currently nodes are listed as a line with no width
        # This adds width of 0.4m so it can detect when it's in that space
        # Vertical
        if xmin == xmax:
            xmin = xmin - self.node_width / 2
            xmax = xmax + self.node_width / 2

        # Horizontal
        elif ymin == ymax:
            ymin = ymin - self.node_width / 2
            ymax = ymax + self.node_width / 2

        else:
            print("something has gone wrong. One of these pairs should be ==")
            print(node_info[0], xmin, xmax, ymin, ymax)

        self.nodes_dict[node_info[0]] = [interval_cust.Interval(xmin, xmax),
                                         interval_cust.Interval(ymin, ymax)]

    def _add_excl_nodes(self, node_info):
        self.nodes_dict[node_info[0]] = [interval_cust.Interval(node_info[5], node_info[6]),
                                         interval_cust.Interval(node_info[7], node_info[8])]

if __name__ == '__main__':
    p = HospitalParameters()
    building = hospital_graph_class.HospitalGraph(p.num_rooms, p.num_halls, p.extra_doors,
                                                  p.hall_door_links, p.extra_door_hall_links, p.connected_halls,
                                                  p.connected_rooms)
    # building.plot_graph()
    #
    # node = 'h00'
    # print(param.nodes_dict[node][0].low)
    # print(param.nodes_dict)
