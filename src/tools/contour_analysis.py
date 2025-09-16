import cv2
import numpy as np
import matplotlib.pyplot as plt

# Coordinates of the polygon
points = np.array([
    [169, 0],
    [219, 24],
    [219, 59],
    [119, 59],
    [76, 24],
    [72, 20],
    [72, 4],
    [74, 0]
], dtype=np.int32)

# Reshape for OpenCV (expects Nx1x2 shape)
pts = points.reshape((-1, 1, 2))

# Create a blank canvas
canvas = np.zeros((100, 250, 3), dtype=np.uint8)

# Draw the polygon
cv2.polylines(canvas, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

# Fill polygon if you want (uncomment)
# cv2.fillPoly(canvas, [pts], color=(0, 255, 0))

# Show with matplotlib
plt.imshow(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
plt.title("Polygon from Given Points")
plt.axis("equal")
plt.show()