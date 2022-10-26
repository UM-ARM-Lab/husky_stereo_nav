# Adapted from https://github.com/UM-ARM-Lab/plant_selector/blob/main/src/plant_modeling.py

import ctypes
import struct
import numpy as np

def float_to_rgb(float_rgb: float) -> np.ndarray:
    """
    Converts a packed float RGB format to an RGB numpy array

    :param float_rgb: RGB value packed as a float
    :returns: numpy array [R, G, B] of integer colors from 0-255
    """
    s = struct.pack('>f', float_rgb)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value

    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    return np.array([r, g, b])
