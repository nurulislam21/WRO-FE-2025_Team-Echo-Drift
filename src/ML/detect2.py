import cv2
import numpy as np
import math


# -------------------------------
# Object class
# -------------------------------
class DetectedObject:
    def __init__(self, left_line, right_line):
        self.left_line = left_line
        self.right_line = right_line

        # Extract coordinates
        (lx1, ly1), (lx2, ly2) = left_line
        (rx1, ry1), (rx2, ry2) = right_line

        # Compute properties
        self.left_x = int((lx1 + lx2) / 2)
        self.right_x = int((rx1 + rx2) / 2)
        self.width = abs(self.right_x - self.left_x)

        self.left_height = abs(ly2 - ly1)
        self.right_height = abs(ry2 - ry1)
        self.height = (self.left_height + self.right_height) / 2

        self.center = (
            (self.left_x + self.right_x) // 2,
            (int((ly1 + ly2 + ry1 + ry2) / 4)),
        )

    def __repr__(self):
        return f"DetectedObject(width={self.width}, height={self.height:.1f}, center={self.center})"


# -------------------------------
# Main code
# -------------------------------
cap = cv2.VideoCapture("video3.mp4")  # or use 0 for webcam
# seek to specific frame if needed
# cap.set(cv2.CAP_PROP_POS_FRAMES, 1500)

OBS_REGION = [85, 175, 555, 340]  # obstacle detection

while True:
    ret, frame = cap.read()
    if not ret:
        break
    # frame = cv2.imread("img.png")
    # if frame is None:
    #     print("❌ Cannot open image or video frame.")
    #     exit()

    frame = cv2.resize(frame, (640, 480))
    roi_frame = frame[OBS_REGION[1]:OBS_REGION[3], OBS_REGION[0]:OBS_REGION[2]]
    # bilateral filter
    roi_frame = cv2.bilateralFilter(roi_frame, d=9, sigmaColor=75, sigmaSpace=75)

    # Edge detection    
    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 20, 200)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_display = roi_frame.copy()

    vertical_lines = []

    # Detect merged vertical lines
    for contour in contours:
        raw_vertical_segments = []

        for j in range(len(contour)):
            p1 = contour[j][0]
            p2 = contour[(j + 1) % len(contour)][0]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]

            if dx == 0:
                angle = 90.0
            else:
                angle = abs(math.degrees(math.atan2(dy, dx)))
            if angle > 180:
                angle -= 180

            if 80 <= angle <= 100:
                raw_vertical_segments.append((tuple(p1), tuple(p2)))

        if not raw_vertical_segments:
            continue

        # Merge small vertical pieces
        all_points = [p for seg in raw_vertical_segments for p in seg]
        all_points = sorted(all_points, key=lambda p: p[0])
        grouped = []
        group = [all_points[0]]

        for p in all_points[1:]:
            if abs(p[0] - group[-1][0]) <= 3:
                group.append(p)
            else:
                grouped.append(group)
                group = [p]
        grouped.append(group)

        for group in grouped:
            xs = [p[0] for p in group]
            ys = [p[1] for p in group]
            avg_x = int(np.mean(xs))
            top_y = int(min(ys))
            bottom_y = int(max(ys))
            vertical_lines.append(((avg_x, top_y), (avg_x, bottom_y)))


    # -------------------------------
    # Pair vertical lines to form rectangles
    # -------------------------------
    MIN_DIST = 20  # min distance between vertical sides
    MAX_DIST = 100  # max distance between vertical sides
    ASPECT_RATIO_TOL = 0.25  # tolerance in height ratio (±25%)
    MIN_HEIGHT = 30  # minimum height of vertical lines to consider

    detected_objects = []
    used_lines = set()  # keep indices of already used vertical lines

    for i in range(len(vertical_lines)):
        if i in used_lines:
            continue  # skip if already paired

        (x1a, y1a), (x2a, y2a) = vertical_lines[i]
        h1 = abs(y2a - y1a)

        if h1 < MIN_HEIGHT:
            continue

        for j in range(i + 1, len(vertical_lines)):
            if j in used_lines:
                continue  # skip if already used in another pair

            (x1b, y1b), (x2b, y2b) = vertical_lines[j]
            h2 = abs(y2b - y1b)

            if h2 < MIN_HEIGHT:
                continue

            dist = abs(x1b - x1a)
            if dist < MIN_DIST and dist > MAX_DIST:
                continue

            # check height similarity ratio
            ratio = min(h1, h2) / max(h1, h2)
            if ratio < (1 - ASPECT_RATIO_TOL):
                continue

            # store object
            detected_objects.append(DetectedObject(vertical_lines[i], vertical_lines[j]))

            # mark both lines as used
            used_lines.add(i)
            used_lines.add(j)

            break  # stop searching for another match for this line

    # -------------------------------
    # Draw results
    # -------------------------------
    for line in vertical_lines:
        cv2.line(roi_display, line[0], line[1], (0, 255, 0), 2)

    for obj in detected_objects:
        cv2.line(roi_display, obj.left_line[0], obj.left_line[1], (255, 0, 0), 2)
        cv2.line(roi_display, obj.right_line[0], obj.right_line[1], (255, 0, 0), 2)
        cx, cy = obj.center
        cv2.circle(roi_display, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(
            roi_display,
            f"W:{obj.width} H:{int(obj.height)}",
            (cx - 40, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )

    print("\n🟩 Detected rectangle-like objects:")
    for obj in detected_objects:
        print(obj)

    cv2.imshow("Edges", edges)
    cv2.imshow("Detected Objects", roi_display)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.waitKey(0)
cv2.destroyAllWindows()
