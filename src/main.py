import cv2 as cv
import numpy as np
import time
import utlis

# Constants
PX_TO_CM = 0.00264583333
FOCAL_DISTANCE = 3  # cm
SIGNAL_SIZE = 10  # cm
WEIGHT = 5
OBJECT_SIZE = 30  # pixels
FRAME_WIDTH = 680
FRAME_HEIGHT = 400
BOX_HEIGHT = FRAME_HEIGHT // 2  # Bottom half of the frame
BOX_Y_START = FRAME_HEIGHT - BOX_HEIGHT  # Start position of the box
LAP_TIME_THRESHOLD = 30  # Minimum time (seconds) to detect a new lap

# Initialize camera
camera = cv.VideoCapture(0)
camera.set(cv.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

order = 0
lap_number = 0
start_time = time.time()

def process_signal_data(data):
    """Processes the signal detection data and returns the action."""
    if data[0] == 1:
        if data[1] == 0 and data[4] < 15:
            return "LEFT"
        elif data[4] < 15:
            return "LEFT"
        else:
            return "CENTERED"
    elif data[0] == 0:
        if data[1] == 0 and data[4] < 15:
            return "RIGHT"
        elif data[4] < 15:
            return "RIGHT"
        else:
            return "CENTERED"
    else:
        return "WALL FOLLOW"

def detect_lap(frame):
    """Detects a lap using a visual marker (blue line at the start/finish)."""
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    if cv.countNonZero(mask) > 500:  # If enough blue pixels detected
        return True
    return False

def main_loop():
    global order, lap_number, start_time
    prev_lap_detected = False

    while True:
        ret, frame = camera.read()
        if not ret:
            camera.set(cv.CAP_PROP_POS_FRAMES, 0)  # Restart video from the beginning
            continue

        current_time = time.time()

        # Define the detection region (bottom half of the frame)
        detection_box = frame[BOX_Y_START:FRAME_HEIGHT, :]
        cv.rectangle(frame, (0, BOX_Y_START), (FRAME_WIDTH, FRAME_HEIGHT), (0, 255, 0), 2)

        # Lap detection (visual marker)
        lap_detected = detect_lap(frame)
        if lap_detected and not prev_lap_detected:
            print(f"Lap: {lap_number}")
            prev_lap_detected = True
        elif not lap_detected:
            prev_lap_detected = False

        if lap_number != 3:
            if order == 0:
                # Signal Detection within the defined region
                signal_img, signal_data = utlis.signal_detection(detection_box, SIGNAL_SIZE, WEIGHT, OBJECT_SIZE,
                                                                 FOCAL_DISTANCE, PX_TO_CM, FRAME_WIDTH)
                cv.imshow("Signal Image", signal_img)
                action = process_signal_data(signal_data)
                print(action)
                if action == "WALL FOLLOW":
                    order += 0
            else:
                # Wall Detection
                wall_img, wall_data, _ = utlis.wall_detection(frame, FRAME_WIDTH, FRAME_HEIGHT, 6)
                print(wall_data)
                if wall_data == "N":
                    order -= 1

            end = time.time()
            print(f"FPS: {1 / (end - current_time):.2f}")

            # Check if 'q' is pressed to exit
            if cv.waitKey(1) & 0xFF == ord('q'):
                print("Quitting...")
                break
        else:
            print("Race completed!")
            break

        cv.imshow("Frame", frame)  # Show the main frame with the detection box

    camera.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main_loop()
