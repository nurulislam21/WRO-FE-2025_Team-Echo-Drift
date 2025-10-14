import cv2
import numpy as np

selected_point = None
lab_value = None

def mouse_callback(event, x, y, flags, param):
    global selected_point, lab_value
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_point = (x, y)
        frame = param["frame"]
        pixel_bgr = frame[y, x].reshape(1, 1, 3)
        pixel_lab = cv2.cvtColor(pixel_bgr, cv2.COLOR_BGR2Lab)[0, 0]

        # Store both OpenCV and true Lab
        L_cv, a_cv, b_cv = pixel_lab
        L_true = (L_cv / 255) * 100
        a_true = a_cv - 128
        b_true = b_cv - 128

        lab_value = {
            "opencv_lab": (L_cv, a_cv, b_cv),
            "true_lab": (round(L_true, 2), round(a_true, 2), round(b_true, 2)),
        }
        print(f"Clicked Pixel at {x},{y}")
        print(f"OpenCV LAB: {lab_value['opencv_lab']}")
        print(f"CIE LAB: {lab_value['true_lab']}\n")

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Cannot access webcam.")
        return

    print("🎥 Webcam started — click anywhere to get LAB color values.")
    print("Press 'q' to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_copy = frame.copy()
        if selected_point:
            cv2.circle(frame_copy, selected_point, 5, (0, 0, 255), -1)
            cv2.putText(frame_copy, "Selected", (selected_point[0] + 10, selected_point[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Webcam - Click to get LAB color", frame_copy)
        cv2.setMouseCallback("Webcam - Click to get LAB color", mouse_callback, {"frame": frame})

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()