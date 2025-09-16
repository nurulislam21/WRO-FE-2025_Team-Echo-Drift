import cv2
import numpy as np


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

    if direction is not None:
        modified_contours = []

        for contour in contours:
            # reshape for easier handling
            pts = contour.reshape(-1, 2)

            # group points by y
            y_to_x = {}
            for x, y in pts:
                if y not in y_to_x:
                    y_to_x[y] = []
                y_to_x[y].append(x)

            # build modified contour
            new_pts = []
            for x, y in pts:
                max_x = max(y_to_x[y])  # max x for this y
                if x == max_x:
                    new_pts.append([roi[3], y])  # replace max x with constant
                else:
                    new_pts.append([x, y])  # keep original

            modified_contours.append(
                np.array(new_pts, dtype=np.int32).reshape(-1, 1, 2)
            )
            contours = modified_contours

    return contours


def max_contour(contours):
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
    ROI1,
    ROI2,
    ROI3,
    ROI4,
    REVERSE_TRIGGER_X_MIN,
    REVERSE_TRIGGER_X_MAX,
    REVERSE_TRIGGER_Y,
    left_result,
    right_result,
    orange_result,
    blue_result,
    green_result,
    red_result,
    angle,
    current_intersections,
    left_area,
    right_area,
    orange_area,
    blue_area,
    obstacle_wall_pivot,
):
    debug_frame = frame.copy()
    debug_frame = display_roi(debug_frame, [ROI1, ROI2, ROI3, ROI4], (255, 0, 255))

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

    # draw reverse trigger zone
    cv2.rectangle(
        debug_frame,
        (ROI4[0] + REVERSE_TRIGGER_X_MIN, ROI4[1] + REVERSE_TRIGGER_Y),
        (ROI4[0] + REVERSE_TRIGGER_X_MAX, ROI4[3]),
        (255, 0, 0),
        2,
    )

    # Draw contours using the latest results
    if left_result.contours:
        cv2.drawContours(
            debug_frame[ROI1[1] : ROI1[3], ROI1[0] : ROI1[2]],
            left_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    if right_result.contours:
        cv2.drawContours(
            debug_frame[ROI2[1] : ROI2[3], ROI2[0] : ROI2[2]],
            right_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    if orange_result.contours:
        cv2.drawContours(
            debug_frame[ROI3[1] : ROI3[3], ROI3[0] : ROI3[2]],
            orange_result.contours,
            -1,
            (0, 165, 255),
            2,
        )
    if blue_result.contours:
        cv2.drawContours(
            debug_frame[ROI3[1] : ROI3[3], ROI3[0] : ROI3[2]],
            blue_result.contours,
            -1,
            (0, 165, 255),
            2,
        )

    if green_result.contours:
        cv2.drawContours(
            debug_frame[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
            green_result.contours,
            -1,
            (0, 255, 0),
            2,
        )
    if red_result.contours:
        cv2.drawContours(
            debug_frame[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
            red_result.contours,
            -1,
            (0, 0, 255),
            2,
        )

    status = f"Angle: {angle} | Turns: {current_intersections/4} | L: {left_area} | R: {right_area} | OR: {orange_area} | BL: {blue_area}"
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
