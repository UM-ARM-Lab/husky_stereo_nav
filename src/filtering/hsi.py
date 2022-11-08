import cv2
import numpy as np


def rgb_to_hsi(img):

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

        # Merge channels into picture and return image
        # hsi = cv2.merge((hue, saturation, intensity))
        print(np.min(hue), np.max(hue), np.min(saturation), np.max(saturation), np.min(intensity), np.max(intensity))
        return np.stack((hue, saturation, intensity), axis=2)
        # return hsi


if __name__ == "__main__":
    img = cv2.imread("data/rgb_img_zed1.png")
    hsi = rgb_to_hsi(img)
    # cv2.imwrite("hsi_ref_image.png", hsi)
    cv2.imshow("test", hsi)
    cv2.waitKey()
