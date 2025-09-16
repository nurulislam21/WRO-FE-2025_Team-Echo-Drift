import cv2
import numpy as np
from collections import defaultdict


def replace_closest(polygon, new_point):
    # Compute distances from new_point to each polygon vertex
    distances = np.linalg.norm(polygon - new_point, axis=1)
    # Index of the closest point
    idx = np.argmin(distances)
    # Replace that vertex
    polygon[idx] = new_point
    return polygon


def find_contours(frame, lower_color, upper_color, roi, direction=None):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]
    labImg = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2Lab)
    img_blur = cv2.GaussianBlur(labImg, (9, 9), 0)
    mask = cv2.inRange(img_blur, lower_color, upper_color)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if direction is not None and contours:
        if max_contour_area(contours)[0] < 1000:
            return contours

        # Combine all points into one array safely
        all_points = np.concatenate(contours, axis=0)
        hull = cv2.convexHull(all_points)
        hull = hull.reshape(-1, 2)

        if direction == "right":
            hull = replace_closest(hull, np.array([roi[2] - roi[0], 0]))
            hull = replace_closest(hull, np.array([roi[2] - roi[0], roi[3] - roi[1]]))

        elif direction == "left":
            hull = replace_closest(hull, np.array([0, 0]))
            hull = replace_closest(hull, np.array([0, roi[3] - roi[1]]))

        contours = [hull.reshape(-1, 1, 2)]

    return contours


def max_contour_area(contours):
    if len(contours) == 0:
        return (0, None)
    largest = max(contours, key=cv2.contourArea)
    return (cv2.contourArea(largest), largest)


def display_roi(frame, rois, color):
    for roi in rois:
        x1, y1, x2, y2 = roi
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    return frame


def display_debug_screen(
    CAM_WIDTH,
    CAM_HEIGHT,
    frame,
    LEFT_REGION,
    RIGHT_REGION,
    LAP_REGION,
    OBS_REGION,
    FRONT_WALL_REGION,
    REVERSE_REGION,
    left_result,
    right_result,
    orange_result,
    blue_result,
    green_result,
    red_result,
    reverse_result,
    angle,
    current_intersections,
    left_area,
    right_area,
    obstacle_wall_pivot,
):
    debug_frame = frame.copy()
    debug_frame = display_roi(debug_frame, [LEFT_REGION, RIGHT_REGION, LAP_REGION, OBS_REGION], (255, 0, 255))
    debug_frame = display_roi(debug_frame, [REVERSE_REGION, FRONT_WALL_REGION], (0, 255, 255))

    if obstacle_wall_pivot != (None, None):
        cv2.circle(
            debug_frame,
            obstacle_wall_pivot,
            5,
            (0, 0, 255),
        )

    cv2.line(
        debug_frame,
        (CAM_WIDTH // 2, 0),
        (CAM_WIDTH // 2, CAM_HEIGHT),
        (255, 0, 0),
        1,
    )  # blue line in the center

    # Draw contours using the latest results
    if left_result.contours:
        cv2.drawContours(
            debug_frame[LEFT_REGION[1] : LEFT_REGION[3], LEFT_REGION[0] : LEFT_REGION[2]],
            left_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    if right_result.contours:
        cv2.drawContours(
            debug_frame[RIGHT_REGION[1] : RIGHT_REGION[3], RIGHT_REGION[0] : RIGHT_REGION[2]],
            right_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    if orange_result.contours:
        cv2.drawContours(
            debug_frame[LAP_REGION[1] : LAP_REGION[3], LAP_REGION[0] : LAP_REGION[2]],
            orange_result.contours,
            -1,
            (0, 165, 255),
            2,
        )
    if blue_result.contours:
        cv2.drawContours(
            debug_frame[LAP_REGION[1] : LAP_REGION[3], LAP_REGION[0] : LAP_REGION[2]],
            blue_result.contours,
            -1,
            (0, 165, 255),
            2,
        )

    if green_result.contours:
        cv2.drawContours(
            debug_frame[OBS_REGION[1] : OBS_REGION[3], OBS_REGION[0] : OBS_REGION[2]],
            green_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    if red_result.contours:
        cv2.drawContours(
            debug_frame[OBS_REGION[1] : OBS_REGION[3], OBS_REGION[0] : OBS_REGION[2]],
            red_result.contours,
            -1,
            (0, 0, 255),
            2,
        )

    if reverse_result.contours:
        cv2.drawContours(
            debug_frame[REVERSE_REGION[1] : REVERSE_REGION[3], REVERSE_REGION[0] : REVERSE_REGION[2]],
            reverse_result.contours,
            -1,
            (255, 0, 0),
            2,
        )

    status = f"Angle: {angle} | Turns: {current_intersections/4} | L: {left_area} | R: {right_area}"
    cv2.putText(
        debug_frame,
        status,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )
    cv2.imshow("Debug View", debug_frame)


def get_max_y_coord(contours) -> tuple[int, int] | tuple[None, None]:
    if not contours:  # empty list
        return (None, None)

    all_points = np.vstack(contours).reshape(-1, 2)
    max_idx = np.argmax(all_points[:, 1])  # index of largest y
    return tuple(all_points[max_idx])  # (x, y)


def get_min_x_coord(contours) -> tuple[int, int] | tuple[None, None]:
    if not contours:  # empty list
        print("No contours found for min x coord")
        return (None, None)

    all_points = np.vstack(contours).reshape(-1, 2)
    min_idx = np.argmin(all_points[:, 0])  # index of smallest x
    return tuple(all_points[min_idx])  # (x, y)


def get_max_x_coord(contours) -> tuple[int, int] | tuple[None, None]:
    if not contours:  # empty list
        print("No contours found for max x coord")
        return (None, None)

    all_points = np.vstack(contours).reshape(-1, 2)
    max_idx = np.argmax(all_points[:, 0])  # index of largest x
    return tuple(all_points[max_idx])  # (x, y)


import cv2


def get_overall_centroid(contours):
    """
    Calculate the single centroid of multiple contours combined.

    Args:
        contours (list): List of contours (each contour is a numpy array of points)

    Returns:
        tuple: (cx, cy) as the centroid of all contours combined, or (None, None) if invalid
    """
    # Compute moments for all contours together
    M = {"m00": 0, "m10": 0, "m01": 0}

    for contour in contours:
        m = cv2.moments(contour)
        M["m00"] += m["m00"]
        M["m10"] += m["m10"]
        M["m01"] += m["m01"]

    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
    else:
        return (None, None)  # invalid if total area = 0
