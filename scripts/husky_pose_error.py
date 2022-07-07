
from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt


def transform_points(points, deltas):

    # make a homogenous transform matrix from the deltas
    tf = np.eye(3)
    dx, dy, dtheta = deltas
    sin, cos = np.sin(dtheta), np.cos(dtheta)
    rotation = np.array([[cos, -sin],
                        [sin, cos]])
    tf[:2, 2] = np.array([dx, dy]).T
    tf[:2, :2] = rotation

    # make points homogenous by adding a one to the end of them
    hom_points = np.vstack((points, np.ones(points.shape[1])))

    # transform the points by multiplying by the transform matrix
    end_points = tf @ hom_points

    # return non homogenous coordinates by cutting of the row of ones
    return end_points[:2, :]


def cost_function2(x, corner_coord, other_distances, physical_points):
    guess_points = transform_points(physical_points, x)
    guess_corner_coord = guess_points[:, 1] - physical_points[:, 1]
    guess_other_distance = np.linalg.norm(
        guess_points[:, 0] - physical_points[:, 0])

    # distances = np.linalg.norm(guess_points - physical_points, axis=0)
    true_distances = np.array([other_distances[0], np.linalg.norm(
        corner_coord - physical_points[:, 1]), other_distances[1]])

    # + np.linalg.norm(distances - true_distances)
        # TODO: square this to get rid of negatives!!
    error = np.linalg.norm(guess_corner_coord - corner_coord) + \
        (guess_other_distance - other_distances[0])**2
    return error**2


def cost_function(x: np.ndarray, measured_distances: np.ndarray, physical_points: np.ndarray) -> float:
    """
    objective function to minimize
    :param x: a vector [dx, dy, dtheta] representing the transform
            from the physical frame to the frame we want to measure cost of
    :param measured_distances: a vector [back_left, front_left, front_right] representing
                            the distances from each physical point to each husky point
    :param physical_points: a vector [back_left, front_left, front_right] representing
                            the coordinates of the points relative to the center of the husky
    """

    guess_points = transform_points(physical_points, x)

    # subtract the physical points from the guess points and take the norm to get distances
    distances = np.linalg.norm(guess_points - physical_points, axis=0)

    # subtract the guess distances from the real distances to get a vector of errors
    error = np.linalg.norm(distances - measured_distances)

    # square error to make it more sensitive(?)
    return error**4


# def get_transform(cost_function, *args):
#     x_initial = np.zeros(3)
#     res = minimize(cost_function, x_initial,
#                    args=args)
#     if not res.success:
#         print(f"optimization failed with message: f{res.message}")
#         exit()
#     return res.x


def get_transform(distances, start_points):
    x_initial = np.zeros(3)
    res = minimize(cost_function, x_initial, args=(distances, start_points))
    if not res.success:
        print(f"optimization failed with message: f{res.message}")
        exit()
    return res.x


# def compute_error(physical_points, measured_distances):
#     dx, dy, dtheta = get_transform(measured_distances, physical_points)
#     print(dx, dy, dtheta)
#     translation_error = np.linalg.norm(np.array([dx, dy]))
#     return translation_error, abs(dtheta)


def test(start_points):
    # start_points = np.array([[-0.35, 0.35, 0.35],
    #                          [0.12025, 0.09975, -0.12025]])
    # start_points = np.array([[-0.35, 0.35, 0.35, -0.35],
    #                          [0.12025, 0.09975, -0.12025, -0.12025]])
    # start_points = np.array([[0, -0.294, 0.117, -0.004],
    #                          [0, 0.295, 0.555, 0.39]])
    # print(f"start points: {start_points}")
    d = np.random.random_sample(3)
    d -= 0.5
    d[:2] /= 5
    # d = np.array([0.2, 0.3, np.pi/4])

    end_points = transform_points(start_points, d)
    distances = np.linalg.norm(end_points - start_points, axis=0)
    # print(distances)
    guess = get_transform(distances, start_points)
    real_error = np.linalg.norm(d)
    guessed_error = np.linalg.norm(guess)
    # print(f"Finished\ntrue tf: {d}\nguessed tf: {guess}")
    # print(
    #     f"real cost: {cost_function(d, distances, start_points)}\nguessed cost: {cost_function(guess, distances, start_points)}")
    # print(
    #     f"real error: {np.linalg.norm(d)}\nguessed error: {np.linalg.norm(guess)}")

    guessed_points = transform_points(start_points, guess)

    if abs(real_error - guessed_error) >= 0.1:
        plt.plot(start_points[0, :], start_points[1, :], "bo-")
        plt.plot(end_points[0, :], end_points[1, :], "ro-")
        plt.plot(guessed_points[0, :], guessed_points[1, :], "go-")
        plt.xlim((-1, 1))
        plt.ylim((-1, 1))
        plt.show()

    return real_error, guessed_error


def test2():
    start_points = np.array([[-0.35, 0.35, 0.35],
                             [0.12025, 0.09975, -0.12025]])
    d = np.random.random_sample(3)
    d -= 0.5
    d[:2] /= 5
    # d = np.array([0.2, 0.3, np.pi/4])

    end_points = transform_points(start_points, d)
    corner_coord = end_points[:, 1] - start_points[:, 1]
    distances = np.linalg.norm(end_points - start_points, axis=0)
    other_distances = np.array([distances[0], distances[2]])

    guess = get_transform(cost_function2, corner_coord,
                          other_distances, start_points)
    real_error = np.linalg.norm(d)
    guessed_error = np.linalg.norm(guess)
    print(f"Finished\ntrue tf: {d}\nguessed tf: {guess}")
    print(
        f"real cost: {cost_function2(d, corner_coord, other_distances, start_points)}\nguessed cost: {cost_function2(guess, corner_coord, other_distances, start_points)}")
    print(
        f"real error: {np.linalg.norm(d)}\nguessed error: {np.linalg.norm(guess)}")

    guessed_points = transform_points(start_points, guess)

    plt.clf()
    plt.plot(start_points[0, :], start_points[1, :], "bo-")
    plt.plot(end_points[0, :], end_points[1, :], "ro-")
    plt.plot(guessed_points[0, :], guessed_points[1, :], "go-")
    plt.xlim((-1, 1))
    plt.ylim((-1, 1))
     # plt.show()

    return real_error, guessed_error


def test_batch():
    for _ in range(50):
        num_points = 4
        start_points = np.random.random_sample((2, num_points))
        flag = False
        for _ in range(1000):
            real, guessed = test(start_points)
            if abs(real - guessed) >= 1e-3:
                print(f"difference found: real={real}, guessed={guessed}")
                # plt.show()
                flag = True
                break
        if not flag:
            print(f"success: {start_points}")
            plt.plot(start_points[0, :], start_points[1, :], "bo-")
            plt.xlim((-1, 1))
            plt.ylim((-1, 1))
            plt.show()


def main():
    # test_batch()
    # test2()
    test_batch()
    # bottom_left = float(input("bottom left distance: "))
    # top_left = float(input("top left distance: "))
    # top_right = float(input("top right distance: "))

    # distances = np.array([bottom_left, top_left, top_right])
    # physical_points = np.array([[-0.35, 0.35, 0.35],
    #                             [0.12025, 0.09975, -0.12025]])
    # position, rotation = compute_error(physical_points, distances)
    # print(f"position error: {position}")
    # print(f"rotation error: {rotation}")


if __name__ == "__main__":
    main()
