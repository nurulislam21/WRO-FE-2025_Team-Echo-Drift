import cv2
import numpy as np


# --- Define fitted curve equation ---
def fitted_b(a):
    return  (-0.0309 * a ** 2) + (12.8072 * a) + -1148.4739


# --- Video Capture ---
cap = cv2.VideoCapture(0)  # webcam

threshold = 10  # distance tolerance in 'b' axis, tune this for your lighting conditions

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to Lab color space
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    L, a, b = cv2.split(lab)

    # Convert to float for precision
    a = a.astype(np.float32)
    b = b.astype(np.float32)

    # Predict b from the fitted curve
    b_pred = fitted_b(a)

    # Compute absolute difference between actual b and predicted b
    diff = np.abs(b - b_pred)

    # Threshold the difference to get mask
    mask = (diff < threshold).astype(np.uint8) * 255

    # Optional: Apply morphological operations to clean mask
    mask = cv2.medianBlur(mask, 5)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours
    result = frame.copy()
    cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

    # Display results
    cv2.imshow("Mask", mask)
    cv2.imshow("Detected Color Contours", result)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
