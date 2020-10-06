#!/usr/bin/env python3

import STRUCT_hospital_graph_class as HospGraph

path_to_pickle = '/home/toothless/workspaces/research_ws/src/hospital-world/pickles/STRUCT_hospital_v1_param_pickle'

hosp_graph = HospGraph.unpickle_it(path_to_pickle)
