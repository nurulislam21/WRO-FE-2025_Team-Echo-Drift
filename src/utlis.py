#
# import cv2
# import numpy as np
# from math import atan
#
# def create_mask(hsv, lower_limit, upper_limit, kernel_size=(11, 11)):
#     """Creates a mask with given HSV limits and applies morphological operations."""
#     kernel = np.ones(kernel_size, np.uint8)
#     mask = cv2.inRange(hsv, lower_limit, upper_limit)
#     mask = cv2.dilate(mask, kernel, iterations=2)
#     mask = cv2.erode(mask, kernel, iterations=2)
#     return mask
#
#
# def signal_detection(image, signal_size, weight, object_size, focal_distance, px, l):
#     img = image
#     blurred = cv2.medianBlur(img, 15)
#     hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#     height, width = img.shape[:2]
#
#     # Green mask
#     lower_limit = np.array([25, 150, 40])
#     upper_limit = np.array([85, 230, 255])
#     # lower_limit = np.array([58, 62, 70])
#     # upper_limit = np.array([96, 255, 255])
#     mask1 = cv2.inRange(hsv, lower_limit, upper_limit)
#     kernel = np.ones((11, 11), np.uint8)
#     mask1 = cv2.dilate(mask1, kernel, iterations=2)
#     mask1 = cv2.erode(mask1, kernel, iterations=2)
#
#     # Red mask
#     lower_limit = np.array([97, 170, 70])
#     upper_limit = np.array([180, 255, 255])
#     # lower_limit = np.array([168, 175, 50])
#     # upper_limit = np.array([180, 255, 255])
#     mask2 = cv2.inRange(hsv, lower_limit, upper_limit)
#     mask2 = cv2.dilate(mask2, kernel, iterations=2)
#     mask2 = cv2.erode(mask2, kernel, iterations=2)
#
#     # Blue mask
#     # lower_limit = np.array([106, 124, 57])
#     # upper_limit = np.array([134, 255, 138])
#     lower_limit = np.array([100, 100, 100])
#     upper_limit = np.array([135, 255, 255])
#     mask3 = cv2.inRange(hsv, lower_limit, upper_limit)
#     mask3 = cv2.dilate(mask3, kernel, iterations=2)
#     mask3 = cv2.erode(mask3, kernel, iterations=2)
#
#     # Finding contours for each color and marking them
#     contours1, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
#     for cnt in contours1:
#         if cv2.contourArea(cnt) > height * width * 0.012:
#             (cx, cy), radius = cv2.minEnclosingCircle(cnt)
#             x, y, w, h = cv2.boundingRect(cnt)
#             cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
#             font = cv2.FONT_HERSHEY_SIMPLEX
#             cv2.putText(img, "Green", (x, y), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
#             dis, latx = d_l(x + w // 2, h, signal_size, focal_distance, l)
#             print("Green Object lateral distance :", latx, "cm")
#             print("Green Object distance :", dis, "cm")
#             try:
#                 angle = 100 - (180 / np.pi * (atan(dis / abs(latx))))
#                 print("Degrees to turn :", 100 - (180 / np.pi * (atan(dis / abs(latx)))))
#             except ZeroDivisionError:
#                 angle = 30
#                 print("Degrees to turn :", 30)
#             if angle < 40:
#                 angle = 40
#             if dis < 60:
#                 if cx < (width // 2 - object_size // 2):
#                     return img, [1, 1, angle, dis, latx]  # Green object
#                 else:
#                     return img, [1, 0, angle, dis, latx]  # Green object
#
#     contours2, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
#     for cnt in contours2:
#         if cv2.contourArea(cnt) > height * width * 0.012:
#             (cx, cy), radius = cv2.minEnclosingCircle(cnt)
#             x, y, w, h = cv2.boundingRect(cnt)
#             cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
#             font = cv2.FONT_HERSHEY_SIMPLEX
#             cv2.putText(img, "Red", (x, y), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
#             dis, latx = d_l(x + w // 2, h, signal_size, focal_distance, l)
#             print("Red Object lateral distance :", latx, "cm")
#             print("Red Object distance :", dis, "cm")
#             try:
#                 angle = 100 - (180 / np.pi * (atan(dis / abs(latx))))
#                 print("Degrees to turn :", angle)
#             except ZeroDivisionError:
#                 angle = 30
#                 print("Degrees to turn :", 30)
#             if angle < 40:
#                 angle = 40
#             if dis < 60:
#                 if cx > (width // 2 + object_size // 2):
#                     return img, [0, 1, angle, dis, latx]  # Red object
#                 else:
#                     return img, [0, 0, angle, dis, latx]  # Red object
#
#     contours3, _ = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
#     for cnt in contours3:
#         if cv2.contourArea(cnt) > height * width * 0.012:
#             (cx, cy), radius = cv2.minEnclosingCircle(cnt)
#             x, y, w, h = cv2.boundingRect(cnt)
#             cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
#             font = cv2.FONT_HERSHEY_SIMPLEX
#             cv2.putText(img, "Blue", (x, y), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
#             dis, latx = d_l(x + w // 2, h, signal_size, focal_distance, l)
#             print("Blue Object lateral distance :", latx, "cm")
#             print("Blue Object distance :", dis, "cm")
#             try:
#                 angle = 100 - (180 / np.pi * (atan(dis / abs(latx))))
#                 print("Degrees to turn :", angle)
#             except ZeroDivisionError:
#                 angle = 30
#                 print("Degrees to turn :", 30)
#             if angle < 40:
#                 angle = 40
#             if dis < 60:
#                 if cx < (width // 2 - object_size // 2):
#                     return img, [2, 1, angle, dis, latx]  # Blue object
#                 else:
#                     return img, [2, 0, angle, dis, latx]  # Blue object
#
#     return img, [2, 0, 0]  # No signal detected
#
#
# # This function returns the distance and lateral distance of the signal from the car
# def d_l(sx, sy, object_size, f, window_size):
#     y = sy * 0.0264583333  # Converting to cm
#     distance_from_object = int((f * object_size) / y)
#     x = ((window_size // 2) - sx) * 0.0264583333  # Converting to cm
#     rx = int(((x * distance_from_object) / f))
#     return distance_from_object, rx

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
        (np.array([35, 50, 50]), np.array([85, 255, 255]), "Green", 1),
        # Red (split into two ranges because red wraps around 0 in HSV)
        (np.array([160, 100, 100]), np.array([180, 255, 255]), "Red", 0)  # Upper red
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