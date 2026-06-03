import numpy as np
import lib.constants as consts

def generate_new_positions(current_positions, target_positions, is_air, speed):
    """
    Advance each leg position toward its target based on speed.
    
    Args:
        current_positions (np.ndarray): Shape (6, 3)
        target_positions (np.ndarray): Shape (6, 3)
        speed (float): Movement scaling factor (0.0 to 1.0+)
    
    Returns:
        np.ndarray: Updated leg positions (6, 3)
    """
    new_positions = np.copy(current_positions)
    for i in range(6):
        delta = target_positions[i] - current_positions[i]
        dist = np.linalg.norm(delta)

        if dist == 0:
            continue  # Already at target

        # Determine phase: swing (air) or stance (ground)
        # is_air = delta[2] > 1e-2  # Heuristic: Z increases -> leg is lifting
        speed_factor = consts.MAX_SPEEDS['air'] if is_air[i] else consts.MAX_SPEEDS['ground']
        step = (delta / dist) * speed_factor * speed * consts.DT

        # print("---- leg", i, "----")
        # print("target", target_positions[i])
        # print("current", current_positions[i])
        # print("step", step)
        # print("delta", delta)
        
        # print("dist", dist, "speed_factor", speed_factor, "speed", speed)
        # Overshoot check
        for j in range(3):
            if abs(step[j]) >= abs(delta[j]):
                new_positions[i][j] = target_positions[i][j]
            else:
                new_positions[i][j] += step[j]
        # print("new pos", new_positions[i])
        # if np.linalg.norm(step) >= dist:
        #     print("--------overshoot--------")
        #     new_positions[i] = np.copy(target_positions[i])
        # else:
        #     new_positions[i] += step

    return new_positions
