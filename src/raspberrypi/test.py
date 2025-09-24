
import cv2
import numpy as np
from math import atan


def create_mask(hsv, lower_limit, upper_limit, kernel):
    """Creates a mask with given HSV limits and applies morphological operations using a precomputed kernel."""
    mask = cv2.inRange(hsv, lower_limit, upper_limit)
    mask = cv2.dilate(mask, kernel, iterations=2)
    mask = cv2.erode(mask, kernel, iterations=2)
    return mask


def signal_detection(image, signal_size, weight, object_size, focal_distance, px, l):
    img = image.copy()
    # Reduce median blur kernel size for faster processing
    blurred = cv2.medianBlur(img, 7)  # Reduced from 15 to 7
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    height, width = img.shape[:2]

    # Precompute frequently used values
    area_threshold = height * width * 0.012
    left_boundary = width // 2 - object_size // 2
    right_boundary = width // 2 + object_size // 2

    # Create kernel once for morphological operations
    kernel = np.ones((11, 11), np.uint8)

    # Define color parameters in order (green, red, blue)
    color_params = [
        # (np.array([25, 150, 20]), np.array([85, 230, 255]), "Green", 1),
        # (np.array([97, 170, 50]), np.array([255, 255, 255]), "Red", 0),
        # (np.array([100, 100, 100]), np.array([135, 255, 255]), "Blue", 2)

        # Green (more flexible range)
        (np.array([35, 100, 50]), np.array([85, 255, 255]), "Green", 1),
        # (np.array([35, 100, 50]), np.array([85, 255, 255]), "Green", 1),
        # Red (split into two ranges because red wraps around 0 in HSV)
        (np.array([160, 100, 200]), np.array([180, 255, 255]), "Red", 0)  # Upper red
        # (np.array([160, 100, 60]), np.array([180, 255, 255]), "Red", 0)  # Upper red

    ]

    for lower, upper, color_name, color_id in color_params:
        mask = create_mask(hsv, lower, upper, kernel)
        # Use RETR_EXTERNAL for faster contour retrieval
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Process largest contours first
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for cnt in contours:
            if cv2.contourArea(cnt) < area_threshold:
                continue  # Skip small contours

            (cx, cy), _ = cv2.minEnclosingCircle(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            # Draw bounding box and label
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            cv2.putText(img, color_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Calculate distance and lateral position
            dis, latx = d_l(x + w // 2, h, signal_size, focal_distance, l)
            print(f"{color_name} Object lateral distance: {latx} cm")
            print(f"{color_name} Object distance: {dis} cm")

            # Calculate angle, handle division by zero
            try:
                angle = 100 - (180 / np.pi * (atan(dis / abs(latx))))
            except ZeroDivisionError:
                angle = 30
            angle = max(angle, 40)  # Ensure minimum angle

            if dis < 60:
                # Determine position relative to center
                if (color_id == 1 and cx < left_boundary) or \
                        (color_id == 0 and cx > right_boundary) or \
                        (color_id == 2 and cx < left_boundary):
                    pos = 1  # Left side
                else:
                    pos = 0  # Right side
                return img, [color_id, pos, angle, dis, latx]

    # No signal detected
    return img, [2, 0, 0, 0, 0]


def d_l(sx, sy, object_size, f, window_size):
    y_cm = sy * 0.0264583333  # Precompute conversion factor
    distance = int((f * object_size) / y_cm)
    x_cm = ((window_size // 2) - sx) * 0.0264583333
    lateral = int((x_cm * distance) / f)
    return distance, lateral