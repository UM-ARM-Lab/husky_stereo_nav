import scipy
import numpy as np

# objective function to minimize
# :param x: a vector [dx, dy, dtheta] representing the transform
#           from the physical frame to the frame we want to measure cost of
# :param measured_distances: a vector [back_left, front_left, front_right] representing
#                            the distances from each physical point to each husky point
# :param physical_points: a vector [back_left, front_left, front_right] representing
#                         the coordinates of the points relative to the center of the husky
def cost_function(x: np.ndarray, measured_distances: np.ndarray, physical_points: np.ndarray) -> float:
    
    # make a homogenous transform matrix from x
    physical_to_guess = np.eye(3)
    dx, dy, dtheta = x
    physical_to_guess[:1, 2] = np.array([dx, dy])
    sin, cos = np.sin(dtheta), np.cos(dtheta)
    rotation = np.array([cos, -sin],
                        [sin, cos])
    physical_to_guess[:1, :1] = rotation
    
    # get a vector of guess points by applying the transform to physical_points
    
    # subtract the physical points from the guess points and take the norm to get a vector of distances

    # subtract the guess distances from the real distances and take the norm to get a vector of errors
    
    # return sum of errors