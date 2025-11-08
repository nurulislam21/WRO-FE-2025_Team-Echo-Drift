import os
import json
import sys
import cv2
import numpy as np
from picamera2 import Picamera2

# low [ 62 127 155]
# high [101 126 169]
# Clicked LAB: [151 119 180]
# [104 130 147]

color_ranges = {}
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    tools_dir = os.path.join(script_dir, "..", "tools")
    tools_dir = os.path.abspath(tools_dir)
    with open(os.path.join(tools_dir, "color_ranges.json"), "r") as f:
        color_ranges = json.load(f)
except FileNotFoundError:
    print("Color ranges file not found. Using default values.")


# get color name from cli arg
if len(sys.argv) > 1:
    color_name = sys.argv[1].lower()
else:
    print("Usage: python masking_test_pi.py <color_name>")
    sys.exit(1)

# validate color name
if color_name.lower() not in ["red", "green", "blue", "orange", "magenta", "black"]:
    print(
        "Invalid color name. Choose from: red, green, blue, orange, magenta, black"
    )
    sys.exit(1)


if color_name == "red":
    LOWER = np.array(color_ranges["LOWER_RED"])
    UPPER = np.array(color_ranges["UPPER_RED"])
elif color_name == "green":
    LOWER = np.array(color_ranges["LOWER_GREEN"])
    UPPER = np.array(color_ranges["UPPER_GREEN"])
elif color_name == "blue":
    LOWER = np.array(color_ranges["LOWER_BLUE"])
    UPPER = np.array(color_ranges["UPPER_BLUE"])
elif color_name == "orange":
    LOWER = np.array(color_ranges["LOWER_ORANGE"])
    UPPER = np.array(color_ranges["UPPER_ORANGE"])
elif color_name == "magenta":
    LOWER = np.array(color_ranges["LOWER_MAGENTA"])
    UPPER = np.array(color_ranges["UPPER_MAGENTA"])
elif color_name == "black":
    LOWER = np.array(color_ranges["LOWER_BLACK"])
    UPPER = np.array(color_ranges["UPPER_BLACK"])

# LOWER = np.array([30, 95, 138])
# UPPER = np.array([88, 135, 178])



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

def main():
    # Initialize Raspberry Pi Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)})
    picam2.configure(config)
    # load camera settings from file
    try:
        with open("camera_settings.json", "r") as f:
            import json
            settings = json.load(f)
            picam2.set_controls(settings)
            print("Loaded camera settings from file.")
    except FileNotFoundError:
        # Default settings if no file found
        print("No camera settings file found. Using default settings.")
        return

    picam2.start()

    roi = (100, 100, 540, 380)  # (x1, y1, x2, y2)

    while True:
        frame = picam2.capture_array()

        contours = find_contours(frame, LOWER, UPPER, roi)

        # draw ROI box
        cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (255, 0, 255), 2)

        # draw contours
        for cnt in contours:
            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)

        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()