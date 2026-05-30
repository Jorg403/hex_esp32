import numpy as np
import lib.constants as consts

def base_to_world(base_coords, tm_world_base, only_rotation=False):
    # Transform global coordinates to world coordinates
    
    if only_rotation:
        tm = tm_world_base.copy()
        tm[0:3, 3] = 0
    else:
        tm = tm_world_base

    # We add a 1 in the last position of leg_coords to make them homogeneous coordinates
    base_coords_h = np.hstack((base_coords, np.ones((base_coords.shape[0], 1), dtype=np.float32)))

    world_coords_h = np.array([
        tm @ base_coords_h[0],
        tm @ base_coords_h[1],
        tm @ base_coords_h[2],
        tm @ base_coords_h[3],
        tm @ base_coords_h[4],
        tm @ base_coords_h[5]
    ], dtype=np.float32)

    # Remove the last column (homogeneous coordinate) to return to 3D coordinates
    return np.round(world_coords_h[:, :3], 3)  # Return only the first three columns (x, y, z)

def world_to_base(world_coords, tm_base_world, only_rotation=False):
    # Transform world coordinates to global coordinates

    if only_rotation:
        tm = tm_base_world.copy()
        tm[0:3, 3] = 0
    else:
        tm = tm_base_world

    # We add a 1 in the last position of leg_coords to make them homogeneous coordinates
    world_coords_h = np.hstack((world_coords, np.ones((world_coords.shape[0], 1), dtype=np.float32)))

    base_coords_h = np.array([
        tm @ world_coords_h[0],
        tm @ world_coords_h[1],
        tm @ world_coords_h[2],
        tm @ world_coords_h[3],
        tm @ world_coords_h[4],
        tm @ world_coords_h[5]
    ], dtype=np.float32)

    # Remove the last column (homogeneous coordinate) to return to 3D coordinates
    return np.round(base_coords_h[:, :3], 3)  # Return only the first three columns (x, y, z)

def leg_to_base(leg_coords, tm_base_body):
    # Transform local poses to global coordinates

    # We add a 1 in the last position of leg_coords to make them homogeneous coordinates
    leg_coords_h = np.hstack((leg_coords, np.ones((leg_coords.shape[0], 1), dtype=np.float32)))


    base_coords_h = np.array([
        tm_base_body @ consts.TM_BODY_LEG0 @ leg_coords_h[0],
        tm_base_body @ consts.TM_BODY_LEG1 @ leg_coords_h[1],
        tm_base_body @ consts.TM_BODY_LEG2 @ leg_coords_h[2],
        tm_base_body @ consts.TM_BODY_LEG3 @ leg_coords_h[3],
        tm_base_body @ consts.TM_BODY_LEG4 @ leg_coords_h[4],
        tm_base_body @ consts.TM_BODY_LEG5 @ leg_coords_h[5]
    ], dtype=np.float32)
    # base_coords_h = np.array([
    #     consts.TM_BODY_LEG0 @ leg_coords_h[0],
    #     consts.TM_BODY_LEG1 @ leg_coords_h[1],
    #     consts.TM_BODY_LEG2 @ leg_coords_h[2],
    #     consts.TM_BODY_LEG3 @ leg_coords_h[3],
    #     consts.TM_BODY_LEG4 @ leg_coords_h[4],
    #     consts.TM_BODY_LEG5 @ leg_coords_h[5]
    # ], dtype=np.float32)

    # Remove the last column (homogeneous coordinate) to return to 3D coordinates
    return np.round(base_coords_h[:, :3], 3)  # Return only the first three columns (x, y, z)

def base_to_leg(base_coords, tm_body_base):
    # Transform global coordinates to local poses
    
    # We add a 1 in the last position of leg_coords to make them homogeneous coordinates
    base_coords_h = np.hstack((base_coords, np.ones((base_coords.shape[0], 1), dtype=np.float32)))

    leg_coords = np.array([
        consts.TM_LEG0_BODY @ tm_body_base @ base_coords_h[0],
        consts.TM_LEG1_BODY @ tm_body_base @ base_coords_h[1],
        consts.TM_LEG2_BODY @ tm_body_base @ base_coords_h[2],
        consts.TM_LEG3_BODY @ tm_body_base @ base_coords_h[3],
        consts.TM_LEG4_BODY @ tm_body_base @ base_coords_h[4],
        consts.TM_LEG5_BODY @ tm_body_base @ base_coords_h[5]
    ], dtype=np.float32)
    # leg_coords = np.array([
    #     consts.TM_LEG0_BODY @ base_coords_h[0],
    #     consts.TM_LEG1_BODY @ base_coords_h[1],
    #     consts.TM_LEG2_BODY @ base_coords_h[2],
    #     consts.TM_LEG3_BODY @ base_coords_h[3],
    #     consts.TM_LEG4_BODY @ base_coords_h[4],
    #     consts.TM_LEG5_BODY @ base_coords_h[5]
    # ], dtype=np.float32)

    # Remove the last column (homogeneous coordinate) to return to 3D coordinates
    return np.round(leg_coords[:, :3], 3)  # Return only the first three columns (x, y, z)