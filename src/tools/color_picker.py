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

# Trackbar callback - saves on every change
def on_trackbar_change(x):
    update_color_ranges_from_trackbars()
    save_color_ranges()

# Function to save color ranges to file
def save_color_ranges():
    global color_ranges, color_name
    
    # Save to color_ranges.json
    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)
    with open(os.path.join(script_dir, "color_ranges.json"), "w") as f:
        json.dump(color_ranges, f, indent=4)
    
    print(f"Color ranges saved to files")

# Function to update color ranges from trackbars
def update_color_ranges_from_trackbars():
    global lower_bound, upper_bound, lower_bound_hsv, upper_bound_hsv, color_ranges, color_name
    
    # Only update if the Controls window exists
    try:
        # Get LAB values from trackbars
        lower_bound = np.array([
            cv2.getTrackbarPos('L Min', 'Controls'),
            cv2.getTrackbarPos('A Min', 'Controls'),
            cv2.getTrackbarPos('B Min', 'Controls')
        ])
        upper_bound = np.array([
            cv2.getTrackbarPos('L Max', 'Controls'),
            cv2.getTrackbarPos('A Max', 'Controls'),
            cv2.getTrackbarPos('B Max', 'Controls')
        ])
        
        # Get HSV values from trackbars
        lower_bound_hsv = np.array([
            cv2.getTrackbarPos('H Min', 'Controls'),
            cv2.getTrackbarPos('S Min', 'Controls'),
            cv2.getTrackbarPos('V Min', 'Controls')
        ])
        upper_bound_hsv = np.array([
            cv2.getTrackbarPos('H Max', 'Controls'),
            cv2.getTrackbarPos('S Max', 'Controls'),
            cv2.getTrackbarPos('V Max', 'Controls')
        ])
    except cv2.error:
        # Window doesn't exist yet, skip update
        return
    
    # Update color_ranges dictionary
    if color_name == "red":
        color_ranges["LOWER_RED"] = lower_bound.tolist()
        color_ranges["UPPER_RED"] = upper_bound.tolist()
        color_ranges["LOWER_RED_HSV"] = lower_bound_hsv.tolist()
        color_ranges["UPPER_RED_HSV"] = upper_bound_hsv.tolist()
    elif color_name == "green":
        color_ranges["LOWER_GREEN"] = lower_bound.tolist()
        color_ranges["UPPER_GREEN"] = upper_bound.tolist()
        color_ranges["LOWER_GREEN_HSV"] = lower_bound_hsv.tolist()
        color_ranges["UPPER_GREEN_HSV"] = upper_bound_hsv.tolist()
    elif color_name == "blue":
        color_ranges["LOWER_BLUE"] = lower_bound.tolist()
        color_ranges["UPPER_BLUE"] = upper_bound.tolist()
        color_ranges["LOWER_BLUE_HSV"] = lower_bound_hsv.tolist()
        color_ranges["UPPER_BLUE_HSV"] = upper_bound_hsv.tolist()
    elif color_name == "orange":
        color_ranges["LOWER_ORANGE"] = lower_bound.tolist()
        color_ranges["UPPER_ORANGE"] = upper_bound.tolist()
        color_ranges["LOWER_ORANGE_HSV"] = lower_bound_hsv.tolist()
        color_ranges["UPPER_ORANGE_HSV"] = upper_bound_hsv.tolist()
    elif color_name == "magenta":
        color_ranges["LOWER_MAGENTA"] = lower_bound.tolist()
        color_ranges["UPPER_MAGENTA"] = upper_bound.tolist()
        color_ranges["LOWER_MAGENTA_HSV"] = lower_bound_hsv.tolist()
        color_ranges["UPPER_MAGENTA_HSV"] = upper_bound_hsv.tolist()
    elif color_name == "black":
        color_ranges["LOWER_BLACK"] = lower_bound.tolist()
        color_ranges["UPPER_BLACK"] = upper_bound.tolist()
        color_ranges["LOWER_BLACK_HSV"] = lower_bound_hsv.tolist()
        color_ranges["UPPER_BLACK_HSV"] = upper_bound_hsv.tolist()
    
    # Create a color preview window with swatches for LAB and HSV
    lab_min_bgr = cv2.cvtColor(cv2.cvtColor(np.uint8([[lower_bound]]), cv2.COLOR_Lab2BGR), cv2.COLOR_BGR2RGB)[0][0]
    lab_max_bgr = cv2.cvtColor(cv2.cvtColor(np.uint8([[upper_bound]]), cv2.COLOR_Lab2BGR), cv2.COLOR_BGR2RGB)[0][0]
    hsv_min_bgr = cv2.cvtColor(cv2.cvtColor(np.uint8([[lower_bound_hsv]]), cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2RGB)[0][0]
    hsv_max_bgr = cv2.cvtColor(cv2.cvtColor(np.uint8([[upper_bound_hsv]]), cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2RGB)[0][0]

    # Create a 100x100 swatch for each
    swatches = np.zeros((100, 400, 3), dtype=np.uint8)
    swatches[:, :100] = lab_min_bgr        # LAB Min
    swatches[:, 100:200] = lab_max_bgr     # LAB Max
    swatches[:, 200:300] = hsv_min_bgr     # HSV Min
    swatches[:, 300:] = hsv_max_bgr        # HSV Max

    # Add labels
    cv2.putText(swatches, "LAB Min", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(swatches, "LAB Max", (110, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(swatches, "HSV Min", (210, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(swatches, "HSV Max", (310, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Show the swatches
    cv2.imshow("Color Preview", swatches)


# Function to create control window with trackbars
def create_trackbars():
    cv2.namedWindow('Controls')

    print(f"L Min: {lower_bound[0]}, A Min: {lower_bound[1]}, B Min: {lower_bound[2]}")
    print(f"L Max: {upper_bound[0]}, A Max: {upper_bound[1]}, B Max: {upper_bound[2]}")

    print(f"H Min: {lower_bound_hsv[0]}, S Min: {lower_bound_hsv[1]}, V Min: {lower_bound_hsv[2]}")
    print(f"H Max: {upper_bound_hsv[0]}, S Max: {upper_bound_hsv[1]}, V Max: {upper_bound_hsv[2]}")
    
    h_min = lower_bound_hsv[0]
    s_min = lower_bound_hsv[1]
    v_min = lower_bound_hsv[2]
    
    h_max = upper_bound_hsv[0]
    s_max = upper_bound_hsv[1]
    v_max = upper_bound_hsv[2]

    l_min = lower_bound[0]
    a_min = lower_bound[1]
    b_min = lower_bound[2]

    l_max = upper_bound[0]
    a_max = upper_bound[1]
    b_max = upper_bound[2]

    # LAB trackbars
    cv2.createTrackbar('L Min', 'Controls', l_min, 255, on_trackbar_change)    
    cv2.createTrackbar('A Min', 'Controls', a_min, 255, on_trackbar_change)    
    cv2.createTrackbar('B Min', 'Controls', b_min, 255, on_trackbar_change)    
    cv2.createTrackbar('L Max', 'Controls', l_max, 255, on_trackbar_change)
    cv2.createTrackbar('A Max', 'Controls', a_max, 255, on_trackbar_change)
    cv2.createTrackbar('B Max', 'Controls', b_max, 255, on_trackbar_change)
    
    # HSV trackbars
    cv2.createTrackbar('H Min', 'Controls', h_min, 179, on_trackbar_change)
    cv2.createTrackbar('S Min', 'Controls', s_min, 255, on_trackbar_change)
    cv2.createTrackbar('V Min', 'Controls', v_min, 255, on_trackbar_change)
    cv2.createTrackbar('H Max', 'Controls', h_max, 179, on_trackbar_change)
    cv2.createTrackbar('S Max', 'Controls', s_max, 255, on_trackbar_change)
    cv2.createTrackbar('V Max', 'Controls', v_max, 255, on_trackbar_change)

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

        print("Color selected! Starting video masking...")
        print("Adjust trackbars in 'Controls' window to fine-tune ranges")
        print("Changes auto-save in real-time")
        color_selected = True
        
        # Create trackbar window first
        create_trackbars()
        
        # Then update and save
        update_color_ranges_from_trackbars()
        save_color_ranges()

        print("Adjust trackbars in 'Controls' window to fine-tune ranges")
        print("Changes auto-save in real-time")
        color_selected = True
        
        # Create trackbar window first
        create_trackbars()
        
        # Then update and save
        update_color_ranges_from_trackbars()
        save_color_ranges()

def main(color_name):
    if color_name.lower() not in ["red", "green", "blue", "orange", "magenta", "black"]:
        print(
            "Invalid color name. Choose from: red, green, blue, orange, magenta, black"
        )
        return

    global lab_img, hsv_img
    global color_selected, lower_bound, upper_bound    

    # Initialize Raspberry Pi Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "BGR888", "size": (640, 480)}
    )
    picam2.configure(config)

    # load camera settings from file
    try:
        script_path = os.path.abspath(__file__)
        script_dir = os.path.dirname(script_path)
        with open(os.path.join(script_dir, "camera_settings.json"), "r") as f:
            import json

            settings = json.load(f)
            picam2.set_controls(settings)
            print("Loaded camera settings from file.")
    except FileNotFoundError:
        # Default settings if no file found
        print("No camera settings file found. Using default settings.")
        return

    picam2.start()
    print("Starting Pi Camera. Click on a color to select it for masking...")
    print("Press 'r' to reset color selection, 'q' to quit")
    print("Note: Trackbar changes auto-save in real-time")

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
            cv2.imshow("Pi Camera - Click to select color", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            cv2.setMouseCallback("Pi Camera - Click to select color", pick_color)
        else:
            # Update bounds from trackbars
            update_color_ranges_from_trackbars()
            
            # Video masking mode (LAB mask)
            mask_lab = cv2.inRange(lab_img, lower_bound, upper_bound)
            # Video masking mode (LAB mask by default)
            mask = cv2.inRange(lab_img, lower_bound, upper_bound)

            # Apply morphological operations
            kernel = np.ones((2, 2), np.uint8)
            mask_lab = cv2.morphologyEx(mask_lab, cv2.MORPH_CLOSE, kernel)
            mask_lab = cv2.morphologyEx(mask_lab, cv2.MORPH_OPEN, kernel)

            # HSV mask
            mask_hsv = cv2.inRange(hsv_img, lower_bound_hsv, upper_bound_hsv)
            mask_hsv = cv2.morphologyEx(mask_hsv, cv2.MORPH_CLOSE, kernel)
            mask_hsv = cv2.morphologyEx(mask_hsv, cv2.MORPH_OPEN, kernel)
            
            # Convert to RGB for display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Create side-by-side comparison
            h, w = mask_lab.shape
            
            # Convert masks to BGR for labeling
            mask_lab_bgr = cv2.cvtColor(mask_lab, cv2.COLOR_GRAY2BGR)
            mask_hsv_bgr = cv2.cvtColor(mask_hsv, cv2.COLOR_GRAY2BGR)
            
            # Add labels and FPS
            cv2.putText(mask_lab_bgr, "LAB Mask", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(mask_hsv_bgr, "HSV Mask", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(mask_lab_bgr, f"FPS: {fps:.2f}", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(mask_hsv_bgr, f"FPS: {fps:.2f}", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Stack masks horizontally
            masks_display = np.hstack([mask_lab_bgr, mask_hsv_bgr])

            # Display results
            cv2.imshow("Pi Camera - RGB", frame_rgb)
            # cv2.imshow("Masks", masks_display)            
            # cv2.imshow("Masked Result", masked_result)
            # cv2.imshow("Colored Mask", colored_mask)

            # Update bounds from trackbars
            update_color_ranges_from_trackbars()
            
            # Video masking mode (LAB mask)
            mask_lab = cv2.inRange(lab_img, lower_bound, upper_bound)

            # Apply morphological operations
            kernel = np.ones((2, 2), np.uint8)
            mask_lab = cv2.morphologyEx(mask_lab, cv2.MORPH_CLOSE, kernel)
            mask_lab = cv2.morphologyEx(mask_lab, cv2.MORPH_OPEN, kernel)

            # mask HSV as well
            mask_hsv = cv2.inRange(hsv_img, lower_bound_hsv, upper_bound_hsv)
            masked_result_hsv = cv2.bitwise_and(frame, frame, mask=mask_hsv)
            colored_mask_hsv = cv2.applyColorMap(mask_hsv, cv2.COLORMAP_JET)
            cv2.putText(
                masked_result_hsv,
                f"FPS: {fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                colored_mask_hsv,
                f"FPS: {fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
            # cv2.imshow("Mask (HSV)", mask_hsv)
            mask_hsv = cv2.morphologyEx(mask_hsv, cv2.MORPH_CLOSE, kernel)
            mask_hsv = cv2.morphologyEx(mask_hsv, cv2.MORPH_OPEN, kernel)
            
            # Convert to RGB for display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Create side-by-side comparison
            h, w = mask_lab.shape
            
            # Convert masks to BGR for labeling
            mask_lab_bgr = cv2.cvtColor(mask_lab, cv2.COLOR_GRAY2BGR)
            mask_hsv_bgr = cv2.cvtColor(mask_hsv, cv2.COLOR_GRAY2BGR)
            
            # Add labels and FPS
            cv2.putText(mask_lab_bgr, "LAB Mask", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(mask_hsv_bgr, "HSV Mask", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(mask_lab_bgr, f"FPS: {fps:.2f}", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(mask_hsv_bgr, f"FPS: {fps:.2f}", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Stack masks horizontally
            masks_display = np.hstack([mask_lab_bgr, mask_hsv_bgr])

            # Display results
            cv2.imshow("Pi Camera - RGB", frame_rgb)
            cv2.imshow("Masks", masks_display)

            # Remove color selection callback
            cv2.setMouseCallback("Pi Camera - RGB", lambda *args: None)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
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
    # get color name from cli args
    import sys
    # load color mapping json file
    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)

    with open(os.path.join(script_dir, "color_ranges.json"), "r") as f:
        color_ranges = json.load(f)

    if len(sys.argv) != 2:
        print("Usage: python color_picker.py <color_name>")
    else:
        color_name = sys.argv[1]
        main(color_name)