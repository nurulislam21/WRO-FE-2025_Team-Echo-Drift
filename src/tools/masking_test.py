import cv2
import numpy as np

# LAB range
LOWER = np.array([59, 115, 86])
UPPER = np.array([119, 155, 126])

def find_contours(frame, lower_color, upper_color, roi):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]
    labImg = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2Lab)
    mask = cv2.inRange(labImg, lower_color, upper_color)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # shift contours back to original image coordinates
    shifted_contours = []
    for cnt in contours:
        cnt = cnt + [x1, y1]   # add ROI offset
        shifted_contours.append(cnt)

    return shifted_contours

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    roi = (200, 200, 720, 720)
    contours = find_contours(frame, LOWER, UPPER, roi)

    # draw ROI box
    cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (255, 0, 255), 2)

    # draw contours
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()