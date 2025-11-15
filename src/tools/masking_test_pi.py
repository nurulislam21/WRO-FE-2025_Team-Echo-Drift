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
if len(sys.argv) > 2:
    color_name = sys.argv[1].lower()
    method = sys.argv[2].upper()
else:
    print("Usage: python masking_test_pi.py <color_name> <method>")
    sys.exit(1)

# validate color name
if color_name.lower() not in ["red", "green", "blue", "orange", "magenta", "black"]:
    print(
        "Invalid color name. Choose from: red, green, blue, orange, magenta, black"
    )
    sys.exit(1)


if color_name == "red":
    if method == "LAB":    
        LOWER = np.array(color_ranges["LOWER_RED"])
        UPPER = np.array(color_ranges["UPPER_RED"])
    else:
        LOWER = np.array(color_ranges["LOWER_RED_HSV"])
        UPPER = np.array(color_ranges["UPPER_RED_HSV"])
elif color_name == "green":
    if method == "LAB":    
        LOWER = np.array(color_ranges["LOWER_GREEN"])
        UPPER = np.array(color_ranges["UPPER_GREEN"])
    else:
        LOWER = np.array(color_ranges["LOWER_GREEN_HSV"])
        UPPER = np.array(color_ranges["UPPER_GREEN_HSV"])
elif color_name == "blue":
    if method == "LAB":    
        LOWER = np.array(color_ranges["LOWER_BLUE"])
        UPPER = np.array(color_ranges["UPPER_BLUE"])
    else:
        LOWER = np.array(color_ranges["LOWER_BLUE_HSV"])
        UPPER = np.array(color_ranges["UPPER_BLUE_HSV"])
elif color_name == "orange":
    if method == "LAB":    
        LOWER = np.array(color_ranges["LOWER_ORANGE"])
        UPPER = np.array(color_ranges["UPPER_ORANGE"])
    else:
        LOWER = np.array(color_ranges["LOWER_ORANGE_HSV"])
        UPPER = np.array(color_ranges["UPPER_ORANGE_HSV"])
elif color_name == "magenta":
    if method == "LAB":    
        LOWER = np.array(color_ranges["LOWER_MAGENTA"])
        UPPER = np.array(color_ranges["UPPER_MAGENTA"])
    else:
        LOWER = np.array(color_ranges["LOWER_MAGENTA_HSV"])
        UPPER = np.array(color_ranges["UPPER_MAGENTA_HSV"])
elif color_name == "black":
    if method == "LAB":    
        LOWER = np.array(color_ranges["LOWER_BLACK"])
        UPPER = np.array(color_ranges["UPPER_BLACK"])
    else:
        LOWER = np.array(color_ranges["LOWER_BLACK_HSV"])
        UPPER = np.array(color_ranges["UPPER_BLACK_HSV"])

# LOWER = np.array([30, 95, 138])
# UPPER = np.array([88, 135, 178])



def find_contours(frame, lower_color, upper_color, roi, method, blur, consider_area=None):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]

    if method == "LAB":
        img = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2Lab)        
    else:
        img = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2HSV)
    # blur and mask
    img_blur = cv2.medianBlur(img, blur)
    # img_blur = cv2.bilateralFilter(labImg, d=7, sigmaColor=75, sigmaSpace=75)
    mask = cv2.inRange(img_blur, lower_color, upper_color)
    kernel = np.ones((13, 13), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if consider_area is not None:
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= consider_area]

    # shift contours back to original image coordinates
    shifted_contours = []
    for cnt in contours:
        cnt = cnt + [x1, y1]   # add ROI offset
        shifted_contours.append(cnt)

    return shifted_contours

def main():
    # Initialize Raspberry Pi Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    # load camera settings from file
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        tools_dir = os.path.join(script_dir, "..", "tools")
        tools_dir = os.path.abspath(tools_dir)
        with open(os.path.join(tools_dir, "camera_settings.json"), "r") as f:
            import json
            settings = json.load(f)
            picam2.set_controls(settings)
            print("Loaded camera settings from file.")
    except FileNotFoundError:
        # Default settings if no file found
        print("No camera settings file found. Using default settings.")
        return

    picam2.start()

    roi = (100, 10, 540, 380)  # (x1, y1, x2, y2)

    while True:
        frame = picam2.capture_array()

        contours = find_contours(frame, LOWER, UPPER, roi, method, blur=11, consider_area=700)

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