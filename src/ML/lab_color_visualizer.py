import cv2
import numpy as np
import matplotlib.pyplot as plt

def lab_to_rgb_opencv(L, a, b):
    """Convert OpenCV-style LAB (0–255) to RGB."""
    lab_pixel = np.uint8([[[L, a, b]]])
    rgb_pixel = cv2.cvtColor(lab_pixel, cv2.COLOR_Lab2RGB)
    return rgb_pixel[0][0]

def visualize_lab_color(L, a, b):
    """Visualize color from OpenCV LAB values."""
    rgb = lab_to_rgb_opencv(L, a, b)
    rgb_normalized = rgb / 255.0

    plt.figure(figsize=(4, 4))
    plt.imshow([[rgb_normalized]])
    plt.title(f"L={L}, a={a}, b={b}\nRGB={tuple(rgb)}")
    plt.axis('off')
    plt.show()

if __name__ == "__main__":
    print("Enter OpenCV-style LAB values (each 0–255)")
    try:
        L, a, b = 59.0,152.0,99.0
        visualize_lab_color(L, a, b)
    except ValueError:
        print("Invalid input. Please enter numeric values.")