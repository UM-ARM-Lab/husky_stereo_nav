from scipy.optimize import minimize
import numpy as np

# objective function to minimize
# :param x: a vector [dx, dy, dtheta] representing the transform
#           from the physical frame to the frame we want to measure cost of
# :param measured_distances: a vector [back_left, front_left, front_right] representing
#                            the distances from each physical point to each husky point
# :param physical_points: a vector [back_left, front_left, front_right] representing
#                         the coordinates of the points relative to the center of the husky
# TODO: make it more vectorized
def cost_function(x: np.ndarray, measured_distances: np.ndarray, physical_points: np.ndarray) -> float:
    
    # make a homogenous transform matrix from x
    physical_to_guess = np.eye(3)
    dx, dy, dtheta = x
    sin, cos = np.sin(dtheta), np.cos(dtheta)
    rotation = np.array([cos, -sin],
                        [sin, cos])
    physical_to_guess[:1, 2] = np.array([dx, dy])
    physical_to_guess[:1, :1] = rotation
    
    # get a vector of guess points by applying the transform to physical_points
    guess_points = [physical_to_guess @ physical for physical in physical_points]
    
    # subtract the physical points from the guess points and take the norm to get a vector of distances
    distances = [np.linalg.norm(guess - physical) for guess, physical in zip(guess_points, physical_points)]

    # subtract the guess distances from the real distances to get a vector of errors
    errors = [dist - measured_dist for dist, measured_dist in zip(distances, measured_distances)]
    return sum(errors) 

def get_transform(measured_distances, physical_points):
    x_initial = np.zeros(3)
    res = minimize(cost_function, x_initial, args=(measured_distances, physical_points))
    if not res.success:
        print(f"optimization failed with message: f{res.message}")
        exit()
    return res.x

def compute_error():
    physical_points = [np.array([0, 0]),
                       np.array([1, 2]),
                       np.array([3, 1])]
    measured_distances = [1, 2, 3]
    dx, dy, dtheta = get_transform(measured_distances, physical_points)
    translation_error = np.linalg.norm(np.array([dx, dy]))
    return translation_error, abs(dtheta)