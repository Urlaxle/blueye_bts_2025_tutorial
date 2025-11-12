import numpy as np

# List of all corners of the ArUco frame 
t_tag_corners_list = [   
                            np.array([-0.4440889112838644, -0.1149409525217068, 1.0349648476872737], dtype='float32'),
                            np.array([-0.30442179064611474, -0.11033928717865343, 1.0264842665913911], dtype='float32'),
                            np.array([-0.29985737717754135, -0.25026283710019614, 1.0257316219644501], dtype='float32'),
                            np.array([-0.439524497815291, -0.2548645024432495, 1.0342122030603327], dtype='float32'),   
                            np.array([0.135661715174131, -0.08975424247490037, 1.007211223826282], dtype='float32'),
                            np.array([0.27540020602726833, -0.08508826373015141, 1.0000430313529864], dtype='float32'),
                            np.array([0.280125237721817, -0.2250046753221393, 1.001078410317399], dtype='float32'),
                            np.array([0.1403867468686797, -0.22967065406688827, 1.0082466027906947], dtype='float32'),
                            np.array([-0.03999999910593033, 0.03999999910593033, 0.0], dtype='float32'),
                            np.array([0.03999999910593033, 0.03999999910593033, 0.0], dtype='float32'),
                            np.array([0.03999999910593033, -0.03999999910593033, 0.0], dtype='float32'),
                            np.array([-0.03999999910593033, -0.03999999910593033, 0.0], dtype='float32'),
                            np.array([-0.16896073427006728, 0.031002677911956843, 0.008396007343673342], dtype='float32'),
                            np.array([-0.0890131099739643, 0.033843026572980314, 0.008952337093637231], dtype='float32'),
                            np.array([-0.0861491684711845, -0.04601240017977588, 0.005091201107744787], dtype='float32'),
                            np.array([-0.16609679276728748, -0.04885274884079935, 0.004534871357780898], dtype='float32'),
                            np.array([-0.2866557159111685, 0.027843659846656287, 0.016736612214386333], dtype='float32'),
                            np.array([-0.20673297726677947, 0.029415429953941107, 0.013592554293762934], dtype='float32'),
                            np.array([-0.20536912821198833, -0.05040161018976413, 0.00835998968817983], dtype='float32'),
                            np.array([-0.28529186685637736, -0.05197338029704895, 0.01150404760880323], dtype='float32'),
                    ]

# Initialize the list
pos_board = []

# Loop through the t_tag_corners_list in steps of 4 up to the 120th element
for i in range(0, 20, 4):  # from 0 to 80 by steps of 4 (to include up to t_tag_corners_list[83])
    # Create an array of four tuples of coordinates and append to pos_board
    pos_board.append(np.array([(t_tag_corners_list[i][0], t_tag_corners_list[i][1], t_tag_corners_list[i][2]), (t_tag_corners_list[i+1][0], t_tag_corners_list[i+1][1], t_tag_corners_list[i+1][2]), (t_tag_corners_list[i+2][0], t_tag_corners_list[i+2][1], t_tag_corners_list[i+2][2]), (t_tag_corners_list[i+3][0], t_tag_corners_list[i+3][1], t_tag_corners_list[i+3][2])], dtype='float32'))

# Defines the Ids of the markers in the board
id_board = np.array([[25], [26], [27], [28], [29]])