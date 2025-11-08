import json
import os
import cv2
import numpy as np
import time
from picamera2 import Picamera2

# Global variables
clicked_pixel = None
lab_img = None
hsv_img = None
color_selected = False
lower_bound = None
upper_bound = None
lower_bound_hsv = None
upper_bound_hsv = None
color_ranges = {}
color_name = ""


# Trackbar callback (does nothing but required by OpenCV)
def nothing(x):
    pass


def save_color_ranges():
    """Save current color ranges to JSON files"""
    global color_ranges, color_name, lower_bound, upper_bound, lower_bound_hsv, upper_bound_hsv

    # Update color_ranges dictionary
    color_upper = color_name.upper()
    color_ranges[f"LOWER_{color_upper}"] = lower_bound.tolist()
    color_ranges[f"UPPER_{color_upper}"] = upper_bound.tolist()
    color_ranges[f"LOWER_{color_upper}_HSV"] = lower_bound_hsv.tolist()
    color_ranges[f"UPPER_{color_upper}_HSV"] = upper_bound_hsv.tolist()

    # Save to current directory
    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)
    with open(os.path.join(script_dir, "color_ranges.json"), "w") as f:
        json.dump(color_ranges, f, indent=4)

    # Save to raspberrypi directory
    raspberrypi_dir = os.path.join(script_dir, "..", "raspberrypi")
    raspberrypi_dir = os.path.abspath(raspberrypi_dir)
    with open(os.path.join(raspberrypi_dir, "color_ranges.json"), "w") as f:
        json.dump(color_ranges, f, indent=4)

    print(f"Color ranges saved: LAB {lower_bound.tolist()} - {upper_bound.tolist()}")
    print(
        f"                   HSV {lower_bound_hsv.tolist()} - {upper_bound_hsv.tolist()}"
    )


def create_slider_window():
    """Create window with sliders for adjusting color ranges"""
    cv2.namedWindow("Color Range Adjustment")

    # LAB sliders
    cv2.createTrackbar(
        "L Min", "Color Range Adjustment", int(lower_bound[0]), 255, nothing
    )
    cv2.createTrackbar(
        "L Max", "Color Range Adjustment", int(upper_bound[0]), 255, nothing
    )
    cv2.createTrackbar(
        "A Min", "Color Range Adjustment", int(lower_bound[1]), 255, nothing
    )
    cv2.createTrackbar(
        "A Max", "Color Range Adjustment", int(upper_bound[1]), 255, nothing
    )
    cv2.createTrackbar(
        "B Min", "Color Range Adjustment", int(lower_bound[2]), 255, nothing
    )
    cv2.createTrackbar(
        "B Max", "Color Range Adjustment", int(upper_bound[2]), 255, nothing
    )

    # HSV sliders
    cv2.createTrackbar(
        "H Min", "Color Range Adjustment", int(lower_bound_hsv[0]), 179, nothing
    )
    cv2.createTrackbar(
        "H Max", "Color Range Adjustment", int(upper_bound_hsv[0]), 179, nothing
    )
    cv2.createTrackbar(
        "S Min", "Color Range Adjustment", int(lower_bound_hsv[1]), 255, nothing
    )
    cv2.createTrackbar(
        "S Max", "Color Range Adjustment", int(upper_bound_hsv[1]), 255, nothing
    )
    cv2.createTrackbar(
        "V Min", "Color Range Adjustment", int(lower_bound_hsv[2]), 255, nothing
    )
    cv2.createTrackbar(
        "V Max", "Color Range Adjustment", int(upper_bound_hsv[2]), 255, nothing
    )


def update_ranges_from_sliders():
    """Read slider values and update global bounds"""
    global lower_bound, upper_bound, lower_bound_hsv, upper_bound_hsv

    # Read LAB values
    l_max = cv2.getTrackbarPos("L Max", "Color Range Adjustment")
    a_max = cv2.getTrackbarPos("A Max", "Color Range Adjustment")
    b_max = cv2.getTrackbarPos("B Max", "Color Range Adjustment")

    l_min = cv2.getTrackbarPos("L Min", "Color Range Adjustment")
    a_min = cv2.getTrackbarPos("A Min", "Color Range Adjustment")
    b_min = cv2.getTrackbarPos("B Min", "Color Range Adjustment")

    lower_bound = np.array([l_min, a_min, b_min])
    upper_bound = np.array([l_max, a_max, b_max])

    # Read HSV values
    h_max = cv2.getTrackbarPos("H Max", "Color Range Adjustment")
    s_max = cv2.getTrackbarPos("S Max", "Color Range Adjustment")
    v_max = cv2.getTrackbarPos("V Max", "Color Range Adjustment")

    h_min = cv2.getTrackbarPos("H Min", "Color Range Adjustment")
    s_min = cv2.getTrackbarPos("S Min", "Color Range Adjustment")
    v_min = cv2.getTrackbarPos("V Min", "Color Range Adjustment")

    lower_bound_hsv = np.array([h_min, s_min, v_min])
    upper_bound_hsv = np.array([h_max, s_max, v_max])


# Mouse callback function
def pick_color(event, x, y, flags, param):
    global clicked_pixel, lab_img, hsv_img
    global color_selected, lower_bound, upper_bound
    global lower_bound_hsv, upper_bound_hsv

    if event == cv2.EVENT_LBUTTONDOWN:
        # LAB value at click
        clicked_pixel = lab_img[y, x]
        L, A, B = clicked_pixel

        # HSV value at click
        clicked_pixel_hsv = hsv_img[y, x]
        H, S, V = clicked_pixel_hsv

        # Define tolerance (adjustable)
        L_tol, A_tol, B_tol = 30, 20, 20
        H_tol, S_tol, V_tol = 10, 40, 40

        # LAB bounds
        lower_bound = np.array(
            [max(L - L_tol, 0), max(A - A_tol, 0), max(B - B_tol, 0)]
        )
        upper_bound = np.array(
            [min(L + L_tol, 255), min(A + A_tol, 255), min(B + B_tol, 255)]
        )

        # HSV bounds
        lower_bound_hsv = np.array(
            [max(H - H_tol, 0), max(S - S_tol, 0), max(V - V_tol, 0)]
        )
        upper_bound_hsv = np.array(
            [min(H + H_tol, 179), min(S + S_tol, 255), min(V + V_tol, 255)]
        )

        # Print LAB range
        print(f"\nClicked LAB: {clicked_pixel}")
        print(f"LOWER = np.array({lower_bound.tolist()})")
        print(f"UPPER = np.array({upper_bound.tolist()})")

        # Print HSV range
        print(f"Clicked HSV: {clicked_pixel_hsv}")
        print(f"HSV Lower: {lower_bound_hsv.tolist()}")
        print(f"HSV Upper: {upper_bound_hsv.tolist()}")

        print("Color selected! Starting video masking with sliders...")
        color_selected = True

        # Save initial selection
        save_color_ranges()

        # Create slider window
        create_slider_window()


def main(color_name_input):
    global color_name, lab_img, hsv_img
    global color_selected, lower_bound, upper_bound, color_ranges

    color_name = color_name_input.lower()

    if color_name not in ["red", "green", "blue", "orange", "magenta", "black"]:
        print(
            "Invalid color name. Choose from: red, green, blue, orange, magenta, black"
        )
        return

    # Initialize Raspberry Pi Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "BGR888", "size": (640, 480)}
    )
    picam2.configure(config)

    # load camera settings from file
    try:
        with open("camera_settings.json", "r") as f:
            settings = json.load(f)
            picam2.set_controls(settings)
            print("Loaded camera settings from file.")
    except FileNotFoundError:
        print("No camera settings file found. Using default settings.")
        return

    picam2.start()
    print("Starting Pi Camera. Click on a color to select it for masking...")
    print("Press 's' to save current ranges, 'r' to reset color selection, 'q' to quit")

    # FPS calculation variables
    prev_time = time.time()
    fps = 0
    frame_count = 0

    while True:
        frame = picam2.capture_array()  # Get frame from Pi Camera

        # FPS calculation
        frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - prev_time
        if elapsed_time >= 1.0:  # update FPS every 1 second
            fps = frame_count / elapsed_time
            frame_count = 0
            prev_time = current_time

        # Overlay FPS on frame
        cv2.putText(
            frame,
            f"FPS: {fps:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        # Convert BGR → LAB and HSV
        lab_img = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if not color_selected:
            # Color selection mode
            cv2.imshow("Pi Camera - Click to select color", frame)
            cv2.setMouseCallback("Pi Camera - Click to select color", pick_color)
        else:
            # Update ranges from sliders
            update_ranges_from_sliders()

            # Video masking mode (LAB mask)
            mask = cv2.inRange(lab_img, lower_bound, upper_bound)

            # Apply morphological operations
            kernel = np.ones((2, 2), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Create masked result
            masked_result = cv2.bitwise_and(frame, frame, mask=mask)

            # Overlay FPS
            cv2.putText(
                masked_result,
                f"FPS: {fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            # Display results
            cv2.imshow("Original", frame)
            cv2.imshow("Mask (LAB)", mask)

            # HSV mask
            mask_hsv = cv2.inRange(hsv_img, lower_bound_hsv, upper_bound_hsv)
            masked_result_hsv = cv2.bitwise_and(frame, frame, mask=mask_hsv)
            cv2.putText(
                masked_result_hsv,
                f"FPS: {fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Mask (HSV)", mask_hsv)

            # Remove color selection callback
            cv2.setMouseCallback("Original", lambda *args: None)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("s") and color_selected:
            # Save current ranges
            save_color_ranges()
            print("Ranges saved!")
        elif key == ord("r"):
            # Reset color selection
            color_selected = False
            clicked_pixel = None
            lower_bound = None
            upper_bound = None
            cv2.destroyAllWindows()
            print("Color selection reset. Click on a new color...")

    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys

    # Load color mapping json file
    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)
    try:
        with open(os.path.join(script_dir, "color_ranges.json"), "r") as f:
            color_ranges = json.load(f)
    except FileNotFoundError:
        color_ranges = {}
        print("No existing color_ranges.json found. Will create new file.")

    if len(sys.argv) != 2:
        print("Usage: python color_picker.py <color_name>")
    else:
        color_name = sys.argv[1]
        main(color_name)
