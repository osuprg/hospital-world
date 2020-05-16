import hospital_graph_class as hosp_graph


if __name__ == "__main__":
    # Define number of rooms and halls
    num_rooms = 22
    num_halls = 5
    extra_doors = ['r07', 'r16', 'r21', 'r21']
    # Associates rooms (door 0) with the adjacent hall - additional doors handled further down
    hall0_rooms = [0, 13, 14, 15]
    hall1_rooms = [1, 2, 3, 4, 5, 16, 17]
    hall2_rooms = [6, 7]
    hall3_rooms = [8, 9, 10, 11, 12, 20]
    hall4_rooms = [18, 19, 21]
    hall_door_links = [hall0_rooms, hall1_rooms, hall2_rooms, hall3_rooms, hall4_rooms]

    extra_door_hall_links = [('r07_d01b', 'h02'),
                             ('r16_d01b', 'h03'),
                             ('r21_d01b', 'h04'),
                             ('r21_d02b', 'h03')]
    connected_halls = [('h00', 'h01'), ('h00', 'h03'),
                       ('h01', 'h02'), ('h01', 'h03'), ('h01', 'h04'),
                       ('h02', 'h03'),
                       ('h03', 'h04')]

    hospital = hosp_graph.HospitalGraph(num_rooms, num_halls, extra_doors, hall_door_links, extra_door_hall_links, connected_halls)
    hospital.plot_graph()
    # hosp_graph.pickle_it(hospital, 'hospital_pickle')


