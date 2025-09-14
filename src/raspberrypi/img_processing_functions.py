import cv2
import numpy as np


def find_contours(frame, lower_color, upper_color, roi):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]
    labImg = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2Lab)
    img_blur = cv2.GaussianBlur(labImg, (7, 7), 0)
    mask = cv2.inRange(img_blur, lower_color, upper_color)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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


def get_max_y_coord(contours) -> tuple[int, int] | tuple[None, None]:
    if not contours:  # empty list
        return (None, None)
    
    all_points = np.vstack(contours).reshape(-1, 2)
    max_idx = np.argmax(all_points[:, 1])  # index of largest y
    return tuple(all_points[max_idx])      # (x, y)

def get_min_x_coord(contours) -> tuple[int, int] | tuple[None, None]:
    if not contours:  # empty list
        print("No contours found for min x coord")
        return (None, None)
    
    all_points = np.vstack(contours).reshape(-1, 2)
    min_idx = np.argmin(all_points[:, 0])  # index of smallest x
    return tuple(all_points[min_idx])      # (x, y)
