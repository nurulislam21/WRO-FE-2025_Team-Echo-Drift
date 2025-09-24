import cv2
import numpy as np

def transform_danger_zone_xshift(points, steering_angle, sensitivity=2):
    transformed = []
    for i, line in enumerate(points):
        x1, y1, x2, y2 = line["x1"], line["y1"], line["x2"], line["y2"]

        # shift top x1 only (keep y same, keep bottom fixed)
        if i == 0:  # left line
            x1_new = int(x1 - steering_angle * sensitivity)
        else:  # right line
            x1_new = int(x1 - steering_angle * sensitivity)

        transformed.append({"x1": x1_new, "y1": y1, "x2": x2, "y2": y2})
    return transformed


# === settings ===
OBS_REGION = [0, 100, 640, 480]  # [x1, y1, x2, y2]
DANGER_ZONE_POINTS = [
    {"x1": 295, "y1": OBS_REGION[1], "x2": 205, "y2": OBS_REGION[3]},
    {"x1": 350, "y1": OBS_REGION[1], "x2": 435, "y2": OBS_REGION[3]},
]

steering_angle = 150  # positive = right, negative = left
sensitivity = 2      # adjust how much x shifts per degree
steering_angle = 90 - steering_angle   # convert to -90 to +90 range

# === create blank image ===
img = np.zeros((480, 640, 3), dtype=np.uint8)

# original zone (green)
for line in DANGER_ZONE_POINTS:
    cv2.line(img, (line["x1"], line["y1"]), (line["x2"], line["y2"]), (0, 255, 0), 2)

# shifted zone (red)
shifted = transform_danger_zone_xshift(DANGER_ZONE_POINTS, steering_angle, sensitivity)
for line in shifted:
    cv2.line(img, (line["x1"], line["y1"]), (line["x2"], line["y2"]), (0, 0, 255), 2)

cv2.imshow("Danger Zone X-Shift", img)
cv2.waitKey(0)
cv2.destroyAllWindows()