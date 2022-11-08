# Adapted from https://github.com/UM-ARM-Lab/plant_selector/blob/main/src/plant_modeling.py

import ctypes
import struct
import numpy as np
import cv2


def load_pc_frame(filename: str) -> np.ndarray:
    """
    Load a numpy array of points from a space separated value text file

    :param filename: the name of the file to read from
    :returns: Nx3 numpy array where each row is [r, g, b] in range of [0, 1]
    """
    points = np.loadtxt(filename)
    points = points[points[:, 0] != np.inf]
    return points


def load_img_frame(filename: str, blur=False) -> np.ndarray:
    """
    Load a numpy array of points from an RGB image

    :param filename: the name of the file to read from
    :returns: Nx3 numpy array where each row is [r, g, b] in range of [0, 1]
    """
    img = cv2.imread(filename)
    if blur:
        img = cv2.GaussianBlur(img, (5, 5), 0)
        # cv2.imshow("blur", img)
        # cv2.waitKey(0)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    colors = img.reshape((-1, 3)).astype("float64") / 255.0
    return colors


def load_img_labels(filename: str) -> np.ndarray:
    """
    Load an Nx1 numpy array of labels from an RGB image where labeled points are white

    :param filename: the name of the file to read from
    :returns: 1D numpy array where each each value is either a 0, 0.5, or 1
              0 = not hose, not ground
              0.5 = hose
              1 = ground
    """
    colors = load_img_frame(filename)
    labels = np.zeros(colors.shape[0])
    labels[colors[:, 0] == 1.0] = 0.5
    labels[colors[:, 1] == 1.0] = 1
    return labels


def float_to_rgb(float_rgb: float) -> np.ndarray:
    """
    Converts a packed float RGB format to an RGB numpy array

    :param float_rgb: RGB value packed as a float
    :returns: numpy array [R, G, B] of integer colors from 0-255
    """
    s = struct.pack(">f", float_rgb)
    i = struct.unpack(">l", s)[0]
    pack = ctypes.c_uint32(i).value

    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = pack & 0x000000FF

    return np.array([r, g, b])

def rgb_to_hsi(img: np.ndarray) -> np.ndarray:

    with np.errstate(divide="ignore", invalid="ignore"):

        # Load image with 32 bit floats as variable type
        bgr = np.float32(img) / 255

        # Separate color channels
        blue = bgr[:, :, 0]
        green = bgr[:, :, 1]
        red = bgr[:, :, 2]

        # Calculate Intensity
        intensity = np.average(bgr, axis=2)

        # Calculate Saturation
        minimum = np.amin(bgr, axis=2)
        saturation = 1 - (3 / (np.sum(bgr, axis=2) + 0.001) * minimum)

        # Calculate Hue
        hue = np.arccos(
            0.5
            * ((red - green) + (red - blue))
            / np.sqrt((red - green) ** 2 + ((red - blue) * (green - blue)))
        )
        ids = blue > green
        hue[ids] = 2 * np.pi - hue[ids]

        return np.stack((hue, saturation, intensity), axis=2)

