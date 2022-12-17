import numpy as np
import cv2
import utils

H_THRESH = 0.15
S_THRESH = 0.2
I_THRESH = 0.0

def segment_ref_hist_hsi(img: np.ndarray, ref_mask: np.ndarray):
    img = cv2.GaussianBlur(img, (5, 5), 0)

    # convert to HSI, flatten, and remove NANs
    hsi_img = utils.rgb_to_hsi(img)
    hsi_pixels = hsi_img.reshape((-1, 3))
    hsi_pixels[np.isnan(hsi_pixels)] = 0

    ref_pixels = ref_mask.reshape((-1, 1)).flatten()

    floor_hsis = hsi_pixels[ref_pixels == 1]

    # run histogram filter using reference hsvs from labeled floor
    obstacle_labels = histogram_filter_hsi(hsi_pixels, floor_hsis)
    obstacle_mask = obstacle_labels.reshape((img.shape[0], img.shape[1]))
    return obstacle_mask

def histogram_filter_hsi(hsis, ref_hsis):
    h_hist, h_bins = np.histogram(ref_hsis[:, 0], bins=20, density=True)
    s_hist, s_bins = np.histogram(ref_hsis[:, 1], bins=20, density=True)
    i_hist, i_bins = np.histogram(ref_hsis[:, 2], bins=20, density=True)

    # filter threshold out low V and S values
    valid_hues = (hsis[:, 2] > 0.1) & (hsis[:, 1] > 0.1)
    valid_sats = hsis[:, 2] > 0.1
    
    # get values of each point in image in each histogram
    # if either histogram value is too low, its an obstacle
    # if a value is outside of the range of the histogram, it's definitely an obstacle because it has zero entries 
    h_ids = np.digitize(hsis[:, 0], h_bins) - 1
    out_of_bounds_ids = (h_ids <= 0) | (h_ids >= 20)
    h_ids[out_of_bounds_ids] = 1
    s_ids = np.digitize(hsis[:, 1], s_bins) - 1
    out_of_bounds_ids = (s_ids <= 0) | (s_ids >= 20)
    s_ids[out_of_bounds_ids] = 1
    i_ids = np.digitize(hsis[:, 2], i_bins) - 1
    out_of_bounds_ids = (i_ids <= 0) | (i_ids >= 20)
    i_ids[out_of_bounds_ids] = 1

    h_obstacles = h_hist[h_ids] < H_THRESH
    s_obstacles = s_hist[s_ids] < S_THRESH
    i_obstacles = i_hist[i_ids] < I_THRESH

    is_obstacle = h_obstacles | s_obstacles | i_obstacles | out_of_bounds_ids
    # is_obstacle = h_obstacles | s_obstacles | out_of_bounds_ids
    # is_obstacle = (h_obstacles & valid_hues) | (s_obstacles & valid_sats) | out_of_bounds_ids
    return is_obstacle
