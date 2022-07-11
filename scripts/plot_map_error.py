import numpy as np
import matplotlib.pyplot as plt

true = np.array([
[-0.85, 2.34],
[3.65, 3.3],
[6.98, 6.25],
[3.84, 7.56],
])

maps = [
    np.array([[-0.849, 2.27],
              [3.7, 3.15],
              [6.83, 6.29],
              [3.72, 7.45]]),

    np.array([
        [-0.911, 2.27],
        [3.61, 3.26],
        [6.67, 6.35],
        [3.69, 7.53]
    ]),

    np.array([
        [-0.98, 2.4],
        [3.52, 3.4],
        [6.7, 6.41],
        [3.71, 7.63]
    ]),

    np.array([
        [-0.863, 2.24],
        [3.63, 3.28],
        [6.8, 6.33],
        [3.74, 7.5],
    ]),

    np.array([
        [-0.905, 2.25],
        [3.55, 3.26],
        [6.76, 6.3],
        [3.67, 7.53],
    ]),

    np.array([
        [-0.827, 2.4],
        [3.68, 3.2],
        [6.89, 6.17],
        [3.98, 7.32],
    ]),

    np.array([
        [-0.845, 2.31],
        [3.58, 3.17],
        [6.86, 5.99],
        [3.97, 7.28],
    ])]

plt.xlabel("Waypoint")
plt.ylabel("Error (m)")
plt.xticks([1, 2, 3, 4])
for i, map in enumerate(maps):
    # plt.plot(map[:, 0], map[:, 1], "bo")
    error = map - true
    print(error)
    distances = np.linalg.norm(error, axis=1)
    print(distances)
    plt.plot(np.arange(1, 5), distances, "o-", label=f"map {i+1}")
    
plt.legend()
plt.show()
