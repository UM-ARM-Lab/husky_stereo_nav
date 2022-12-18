import numpy as np
from sklearn.cluster import KMeans

DIST_THRESH = 100

def get_dominant_color(colors: np.ndarray, n_clusters) -> np.ndarray:
    """
    :param colors: nx3 numpy array containing RGB values for each point in the pointcloud
    """

    kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(colors)
    dominant_colors = kmeans.cluster_centers_
    # print(f"dominant_colors array: {dominant_colors}")
    # for color in dominant_colors:
    #     r, g, b = color
        # print(f"({r}, {g}, {b})")
    labeled_colors = kmeans.predict(colors)
    # (108, 95, 79)
    # (64, 56, 50)

    # TODO: make this code better, no hardcoded cluster num
    if np.count_nonzero(labeled_colors == 0) >= np.count_nonzero(
        labeled_colors == 1
    ):
        # print(f"dominant color: {dominant_colors[0]}")
        return dominant_colors[0]
    # print(f"dominant color: {dominant_colors[1]}")
    return dominant_colors[1]

def color_distance_grouping(colors):
    floor_color = get_dominant_color(colors, n_clusters=2)
    color_distances = np.linalg.norm(colors - floor_color, axis=1)
    return color_distances

def segment_rgb_dist(img: np.ndarray):
    colors = img.reshape((-1, 3))
    distances = color_distance_grouping(colors)
    obstacle_mask = (distances > DIST_THRESH).reshape((img.shape[0], img.shape[1]))
    return obstacle_mask