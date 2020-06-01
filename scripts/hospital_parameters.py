import interval_cust


class TwoRoomsParameters:
    def __init__(self, filename):
        # "node name": [interval of x range, interval y range]
        self.nodes_dict = {}

        # Build rooms dictionary for choosing next target for trials
        self.rooms_dict = {}

        self.num_rooms = 2
        self.num_halls = 1

        self.hall_width = 2.0
        self.node_width = 0.4

        self.extra_doors = None
        self.extra_door_hall_links = None
        # Associates rooms (door 0) with the adjacent hall - additional doors handled further down
        self.hall0_rooms = [0, 1]
        self.hall_door_links = [self.hall0_rooms]

        self.connected_halls = None

        self._build_nodes_dict(filename)

    def _read_file(self, filename):
        f = open(filename, 'r')
        data = []
        for line in f:
            data.append(line)
        return data

    def _build_nodes_dict(self, filename):
        f = self._read_file(filename)

        for line in f:
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

            else:
                self._add_other_nodes(node_info)

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
            xmin_a = xmin - self.node_width / 2
            xmax_a = xmax + self.node_width / 2

            # This is where things get very hand-coded (ugh)
            # TODO: Make this less world specific
            # Spans the hallway but doesn't overlap with the door node
            if xmin < 15:
                xmax_b = xmin_b + self.hall_width - self.node_width / 2
            else:
                xmin_b = xmax_b - self.hall_width + self.node_width / 2

            middle_y = (ymin_a + ymax_a) / 2
            ymin_b = middle_y - self.node_width / 2
            ymax_b = middle_y + self.node_width / 2

        # Horizontal
        elif ymin == ymax:
            ymin_a = ymin - self.node_width / 2
            ymax_a = ymax + self.node_width / 2

            # This is where things get very hand-coded (ugh)
            # TODO: Make this less world specific
            # Spans the hallway but doesn't overlap with the door node
            if ymin < 10:
                ymin_b = ymax_a
                ymax_b = ymin_b + self.hall_width - self.node_width / 2
            else:
                ymax_b = ymin_a
                ymin_b = ymax_b - self.hall_width + self.node_width / 2

            middle_x = (xmin_a + xmax_a) / 2
            xmin_b = middle_x - self.node_width / 2
            xmax_b = middle_x + self.node_width / 2

        else:
            print("something has gone wrong. One of these pairs should be ==")
            print(xmin, xmax, ymin, ymax)

        # Doorway
        self.nodes_dict[name_a] = [interval_cust.Interval(xmin_a, xmax_a),
                                   interval_cust.Interval(ymin_a, ymax_a)]

        # Hallway node outside doorway
        self.nodes_dict[name_b] = [interval_cust.Interval(xmin_b, xmax_b),
                                   interval_cust.Interval(ymin_b, ymax_b)]

    def _add_other_nodes(self, node_info):
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

        self.nodes_dict[node_info[0]] = [interval_cust.Interval(node_info[5], node_info[6]),
                                         interval_cust.Interval(node_info[7], node_info[8])]


if __name__ == '__main__':
    param = TwoRoomsParameters('hospital_parameters_raw.txt')
    #
    # node = 'h00'
    # print(param.nodes_dict[node][0].low)
    # print(param.nodes_dict)
