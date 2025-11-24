import numpy as np

marker_size = 0.071 # meters

cam_mat = np.array([
    [942.2778458640017, 0, 636.898111776291],
    [0, 946.896116804522, 405.45681464476786],
    [0, 0, 1],
], dtype=np.float32)

dist_coeff = np.array([
    -0.007574449506918484,
    -0.16696507835814586,
    0.005954991219207065,
    0.0016744647761098121,
    0.3191633376366791], dtype=np.float32)

win_cam_mat = np.array([
    [975.6914800562727, 0, 657.1922238570544],
    [0, 976.3474157317961, 351.54103305664995],
    [0, 0, 1]
], dtype=np.float32)

win_dist_coeff = np.array([
    0.04389964856406952,
    -0.17211539337258833,
    -0.002305299309464663,
    0.0016484276591536293,
    0.13158928243393753 
], dtype=np.float32)

single_tag_coord_system = np.array([
    [-marker_size / 2,  marker_size / 2, 0],
    [ marker_size / 2,  marker_size / 2, 0],
    [ marker_size / 2, -marker_size / 2, 0],
    [-marker_size / 2, -marker_size / 2, 0]
], dtype=np.float32).reshape(-1, 1, 3)


