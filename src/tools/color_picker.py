import cv2
import numpy as np

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
    
    # Initialize webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam")
        return
    
    print("Starting webcam. Click on a color to select it for masking...")
    print("Press 'r' to reset color selection, 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame from webcam")
            break
        
        # Convert BGR → LAB for color selection
        lab_img = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
        
        if not color_selected:
            # Color selection mode
            cv2.imshow("Webcam - Click to select color", frame)
            cv2.setMouseCallback("Webcam - Click to select color", pick_color)
        else:
            # Video masking mode
            # Create mask based on selected color
            mask = cv2.inRange(lab_img, lower_bound, upper_bound)
            
            # Apply morphological operations to clean up the mask
            kernel = np.ones((2, 2), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Create masked result
            masked_result = cv2.bitwise_and(frame, frame, mask=mask)
            
            # Optional: Show mask in different colors for better visualization
            colored_mask = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
            
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
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()