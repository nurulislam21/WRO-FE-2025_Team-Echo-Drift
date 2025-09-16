import cv2
import numpy as np
import matplotlib.pyplot as plt

point1 = (220, 0)
point2 = (220, 60)

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
canvas = np.zeros((100, 350, 3), dtype=np.uint8)

# offset to center the polygon in the canvas
offset_x, offset_y = 50, 20
pts += np.array([offset_x, offset_y])
point1 = (point1[0] + offset_x, point1[1] + offset_y)
point2 = (point2[0] + offset_x, point2[1] + offset_y)


def replace_closest(polygon, new_point):
    # Compute distances from new_point to each polygon vertex
    distances = np.linalg.norm(polygon - new_point, axis=1)
    # Index of the closest point
    idx = np.argmin(distances)
    # Replace that vertex
    polygon[idx] = new_point
    return polygon

# Replace the closest point in the polygon to point1 and point2
points = replace_closest(points, np.array(point1))
points = replace_closest(points, np.array(point2))

# Draw the polygon outline in green
cv2.polylines(canvas, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

# Mark each point with a blue dot
for i, (x, y) in enumerate(points):
    cv2.circle(canvas, (x, y), radius=3, color=(255, 0, 0), thickness=-1)
    cv2.putText(canvas, str(i), (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

# Show with matplotlib
plt.imshow(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
plt.title("Polygon with Marked Points")
plt.axis("equal")
plt.show()