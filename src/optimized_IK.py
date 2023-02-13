import numpy as np

# robot arm lengths
seg1 = 150 # segment 1 length
seg2 = 100 # segment 2 length
seg3 = 50 # segment 3 length

def IK_elbow_up(x3, y3, psi):
    '''
    Inverse Kinematics for the elbow up position
    Inputs: 
        x3: end effector x position
        y3: end effector y position 
        psi: end effector orientation [rads]
    Outputs:
        theta1: joint 1 angle [rads]
        theta2: joint 2 angle [rads]
        theta3: joint 3 angle [rads]
    '''
    # 2nd segment end position
    x2 = x3 - seg3 * np.cos(psi)
    y2 = y3 - seg3 * np.sin(psi)

    # angles [rads]
    theta2 = np.arccos((x2**2 + y2**2 - seg1**2 - seg2**2) / (2 * seg1 * seg2)) # elbow up angle
    theta1 = np.arctan2(y2, x2) - np.arctan2(seg2 * np.sin(theta2), seg1 + seg2 * np.cos(theta2))
    theta3 = psi - theta1 - theta2

    return theta1, theta2, theta3


def IK_elbow_down(x3, y3, psi):
    '''
    Inverse Kinematics for the elbow down position
    Inputs: 
        x3: end effector x position 
        y3: end effector y position
        psi: end effector orientation [rads]
    Outputs:
        theta1: joint 1 angle [rads]
        theta2: joint 2 angle [rads]
        theta3: joint 3 angle [rads]
    '''
    # 2nd segment end position
    x2 = x3 - seg3 * np.cos(psi)
    y2 = y3 - seg3 * np.sin(psi)

    # angles [rads]
    theta2 = -np.arccos((x2**2 + y2**2 - seg1**2 - seg2**2) / (2 * seg1 * seg2)) # elbow up angle
    theta1 = np.arctan2(y2, x2) - np.arctan2(seg2 * np.sin(theta2), seg1 + seg2 * np.cos(theta2))
    theta3 = psi - theta1 - theta2

    return theta1, theta2, theta3


def optimized_angles(xe, ye, psi, theta1_curr, theat2_curr, theat3_curr):
    '''
    Optimized angles for the end effector position and orientation
    Optimizations Algorithm:
        1. Find all possible angle combinations for the end effector position and orientation.
        2. Find the difference between the current angles and the possible angles.
        3. Find the max angle difference from each possible angle difference combination.
        4. Find the minimum max angle differences.
        5. The optimized angles are the angles that correspond to this minimum angle difference.
    Inputs:
        xe: end effector x position
        ye: end effector y position
        psi: end effector orientation [rads]
        theta1_curr: current joint 1 angle [rads]
        theta2_curr: current joint 2 angle [rads]
        theta3_curr: current joint 3 angle [rads]
    Outputs:
        angles_optimized: optimized angles of joint 1, 2, and 3 [rads]
    '''
    angles_curr = [theta1_curr, theat2_curr, theat3_curr] # current angles

    angles_candidate =[]
    for psi_angle in psi:
        # Elbow up case
        theta1, theta2, theta3 = IK_elbow_up(xe, ye, psi_angle) # angles
        angles_candidate.append([theta1, theta2, theta3]) # append possible angles
        # Elbow down case
        theta1, theta2, theta3 = IK_elbow_down(xe, ye, psi_angle) # angles
        angles_candidate.append([theta1, theta2, theta3]) # append possible angles

    angles_diff = np.abs(np.array(angles_candidate) - np.array(angles_curr)) # difference between current angles and possible angles
    max_angles = np.where(np.isnan(angles_diff), np.inf, angles_diff) # locate max angle differences
    max_rows = np.amax(max_angles, axis=1) # max angle differences
    idx = np.argmin(max_rows) # index of minimum max angle difference
    angles_optimized = angles_candidate[idx] # optimized angles

    return angles_optimized