import cv2
import numpy as np
from picamera2 import Picamera2

# 1
# LAB Lower: [119, 86, 166]
# LAB Upper: [200, 126, 206]

# 2 [191  97 203]
# LAB Lower: [161, 77, 183]
# LAB Upper: [221, 117, 223]



LOWER = np.array([150, 80, 179])
UPPER = np.array([215, 120, 219])


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
    picam2.set_controls({
        "ExposureTime": 16000,
        "AnalogueGain": 42.0,
        "AeEnable": False,
        "AwbEnable": False,
        "FrameDurationLimits": (40000, 40000)
    })
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