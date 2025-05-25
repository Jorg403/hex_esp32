import math

import lib.constants as consts

import numpy as np
import math
import lib.constants as consts

def calculate_joint_angles(position):
    """
    Calculate the joint angles using inverse kinematics.

    Args:
        position (np.array): Target position as [x, y, z].

    Returns:
        np.array: Joint angles [theta1, theta2, theta3] in degrees.
    """
    x, y, z = position
    l1, l2, l3 = consts.LEG_LENGTHS

    # Calculate theta1 (base rotation)
    theta1 = math.atan2(y, x)

    # Projection onto the arm's plane
    r = math.sqrt(x**2 + y**2)
    s = r - l1

    # Distance to the target point
    d = math.sqrt(z**2 + s**2)

    # Check if the target has collision with the base
    base_col = not (x > consts.BASE_BOUNDING_BOX[0] or abs(y) > consts.BASE_BOUNDING_BOX[1] or z > consts.BASE_BOUNDING_BOX[2])

    # Check reachability
    if d > (l2 + l3) or d < abs(l2 - l3) or base_col:
        print("Target is unreachable")
        return np.array([None, None, None])

    # Calculate theta2 (shoulder)
    cos_theta2 = (l2**2 + d**2 - l3**2) / (2 * l2 * d)
    theta2 = (math.atan2(z, s) + math.acos(cos_theta2)) - math.pi / 2

    # Calculate theta3 (elbow)
    cos_theta3 = (l2**2 + l3**2 - d**2) / (2 * l2 * l3)
    theta3 = math.pi / 2 - math.acos(cos_theta3)

    # Convert to degrees
    angles = np.degrees([theta1, theta2, theta3])
    return angles


class IKSolver:
    def __init__(self, position):
        self.speeds = consts.MAX_SPEEDS
        self.pos = np.copy(position)

    def calculate_joint_angles_dt(self, position):
        """
        Calculate the joint angles using inverse kinematics. Time differential.

        Args:
            pos (np.ndarray): Target position (x, y, z).

        Returns:
            tuple: Joint angles (theta1, theta2, theta3) in degrees.
            bool: True if the position is reached, False otherwise.
        """
        goal_pos = np.copy(position)

        
        speed = self.speeds["air"]
        if self.pos[2] <= consts.GROUND_HEIGHT and goal_pos[2] <= consts.GROUND_HEIGHT:
            goal_pos[2] = consts.GROUND_HEIGHT
            speed = self.speeds["ground"]

        if np.linalg.norm(goal_pos - self.pos) < speed * consts.DT:
            new_pos = goal_pos
            goal_reached = True
        else:
            new_pos = self.pos + (goal_pos - self.pos) * speed * consts.DT / np.linalg.norm(goal_pos - self.pos)
            goal_reached = False
            
        
        # print(f"Current position: {self.pos}")
        # print(f"Target position: {position}")
        # print(f"New position: {new_pos}")
        # print(f"Speed: {speed}")
        # print(f"Goal reached: {goal_reached}")
        # print(f"Distance to target: {np.linalg.norm(position - self.pos)}")
        # print(f"Step size: {speed * consts.DT}")
        # print(f"Actual step size: {np.linalg.norm(new_pos - self.pos)}")

        
        self.pos = new_pos

        return calculate_joint_angles(new_pos), goal_reached
    
