from tracemalloc import start
from scipy.optimize import minimize
import numpy as np

"""
objective function to minimize
:param x: a vector [dx, dy, dtheta] representing the transform
          from the physical frame to the frame we want to measure cost of
:param measured_distances: a vector [back_left, front_left, front_right] representing
                           the distances from each physical point to each husky point
:param physical_points: a vector [back_left, front_left, front_right] representing
                        the coordinates of the points relative to the center of the husky
"""


def cost_function(x: np.ndarray, measured_distances: np.ndarray, physical_points: np.ndarray) -> float:

    # TODO: make it more vectorized
    # make a homogenous transform matrix from x
    physical_to_guess = np.eye(3)
    dx, dy, dtheta = x
    sin, cos = np.sin(dtheta), np.cos(dtheta)
    rotation = np.array([[cos, -sin],
                        [sin, cos]])
    physical_to_guess[:2, 2] = np.array([dx, dy]).T
    physical_to_guess[:2, :2] = rotation
    # print(f"physical_to_guess: {physical_to_guess}")

    # convert physical points to homogenous coordinates
    physical_points = [np.append(point, 1) for point in physical_points]
    # print(f"physical points: {physical_points}")

    # get a vector of guess points by applying the transform to physical_points
    guess_points = [physical_to_guess @ physical
                    for physical in physical_points]

    # subtract the physical points from the guess points and take the norm to get a vector of distances
    distances = [np.linalg.norm(guess - physical)
                 for guess, physical in zip(guess_points, physical_points)]

    # subtract the guess distances from the real distances to get a vector of errors
    # difference is squared to remove negatives and make cost function more sensitive
    errors = [(dist - measured_dist)**2
              for dist, measured_dist in zip(distances, measured_distances)]
    # print(f"[{dx:.2f}, {dy:.2f}, {dtheta:.2f}]\nerror = {sum(errors):.5f}")
    return sum(errors)


def get_transform(measured_distances, physical_points):
    x_initial = np.zeros(3)
    res = minimize(cost_function, x_initial,
                   args=(measured_distances, physical_points))
    if not res.success:
        print(f"optimization failed with message: f{res.message}")
        exit()
    return res.x


def compute_error(physical_points, measured_distances):
    dx, dy, dtheta = get_transform(measured_distances, physical_points)
    translation_error = np.linalg.norm(np.array([dx, dy]))
    return translation_error, abs(dtheta)


def test():

    # generate a random set of corner points and a random transform
    p0 = np.random.random_sample(2)
    p1 = np.zeros(2)
    p2 = np.array([-p0[1], p0[0]])
    start_points = [p0, p1, p2]
    d = np.random.random_sample(3)

    physical_to_guess = np.eye(3)
    dx, dy, dtheta = d
    sin, cos = np.sin(dtheta), np.cos(dtheta)
    rotation = np.array([[cos, -sin],
                        [sin, cos]])
    physical_to_guess[:2, 2] = np.array([dx, dy]).T
    physical_to_guess[:2, :2] = rotation

    hom_points = [np.append(point, 1) for point in start_points]
    # print(f"physical_to_guess: {physical_to_guess}")
    # print(f"physical_points: {hom_points}")
    # print(physical_to_guess @ start_points[0])
    end_points = [physical_to_guess @ physical
                  for physical in hom_points]

    distances = [np.linalg.norm(end - start)
                 for start, end in zip(hom_points, end_points)]
    guess = get_transform(distances, start_points)
    print(f"Finished\ntrue tf: {d}\nguessed tf: {guess}")


def main():
    test()
    # compute_error()


if __name__ == "__main__":
    main()
