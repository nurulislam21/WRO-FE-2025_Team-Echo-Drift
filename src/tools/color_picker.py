import cv2
import numpy as np
import time
from picamera2 import Picamera2

# Global variables
clicked_pixel = None
lab_img = None
color_selected = False
lower_bound = None
upper_bound = None

# Mouse callback function
def pick_color(event, x, y, flags, param):
    global clicked_pixel, lab_img, color_selected, lower_bound, upper_bound

    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_pixel = lab_img[y, x]  # LAB value at click
        L, A, B = clicked_pixel

        # Define tolerance (adjustable)
        L_tol = 30
        A_tol = 20
        B_tol = 20

        lower_bound = np.array([max(L - L_tol, 0), max(A - A_tol, 0), max(B - B_tol, 0)])
        upper_bound = np.array([min(L + L_tol, 255), min(A + A_tol, 255), min(B + B_tol, 255)])

        print(f"\nClicked LAB: {clicked_pixel}")
        print(f"Lower LAB: {lower_bound[0]}, {lower_bound[1]}, {lower_bound[2]}")
        print(f"Upper LAB: {upper_bound[0]}, {upper_bound[1]}, {upper_bound[2]}")
        print("Color selected! Starting video masking...")

        color_selected = True

def main():
    global lab_img, color_selected, lower_bound, upper_bound

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

    print("Starting Pi Camera. Click on a color to select it for masking...")
    print("Press 'r' to reset color selection, 'q' to quit")

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
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Convert BGR → LAB for color selection
        lab_img = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)

        if not color_selected:
            # Color selection mode
            cv2.imshow("Pi Camera - Click to select color", frame)
            cv2.setMouseCallback("Pi Camera - Click to select color", pick_color)
        else:
            # Video masking mode
            mask = cv2.inRange(lab_img, lower_bound, upper_bound)

            # Apply morphological operations
            kernel = np.ones((2, 2), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Create masked result
            masked_result = cv2.bitwise_and(frame, frame, mask=mask)
            colored_mask = cv2.applyColorMap(mask, cv2.COLORMAP_JET)

            # Overlay FPS on each window
            cv2.putText(masked_result, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(colored_mask, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Display results
            cv2.imshow("Original", frame)
            cv2.imshow("Mask", mask)
            cv2.imshow("Masked Result", masked_result)
            cv2.imshow("Colored Mask", colored_mask)

            # Remove the color selection callback
            cv2.setMouseCallback("Original", lambda *args: None)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('r'):
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
    main()