import copy
from picamera2 import Picamera2
import cv2
from contour_workers import ContourWorkers
from simple_pid import PID
import numpy as np
import threading
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import queue


# Simulated camera settings
CAM_WIDTH = 640
CAM_HEIGHT = 480
MAX_SPEED = 110
MIN_SPEED = 60

# Region of Interest coordinates
ROI1 = [20, 220, 240, 280]  # left
ROI2 = [400, 220, 620, 280]  # right
ROI3 = [200, 300, 440, 350]  # lap detection
ROI4 = [90, 175, 540, 280]  # obstacle detection

# Color ranges
LOWER_BLACK = np.array([21, 109, 112])
UPPER_BLACK = np.array([81, 149, 152])

LOWER_ORANGE = np.array([105, 125, 87])
UPPER_ORANGE = np.array([185, 165, 127])

LOWER_BLUE = np.array([92, 150, 166])
UPPER_BLUE = np.array([152, 190, 206])

# obstacle color ranges
LOWER_RED = np.array([33, 137, 70])
UPPER_RED = np.array([93, 177, 110])

LOWER_GREEN = np.array([60, 88, 150])
UPPER_GREEN = np.array([120, 128, 190])

contour_workers = ContourWorkers(
    mode="NO_OBSTACLE",
    lower_blue=LOWER_BLUE,
    upper_blue=UPPER_BLUE,
    lower_black=LOWER_BLACK,
    upper_black=UPPER_BLACK,
    lower_orange=LOWER_ORANGE,
    upper_orange=UPPER_ORANGE,
    lower_red=LOWER_RED,
    upper_red=UPPER_RED,
    lower_green=LOWER_GREEN,
    upper_green=UPPER_GREEN,
    roi1=ROI1,
    roi2=ROI2,
    roi3=ROI3,
    roi4=ROI4,
)

STRAIGHT_CONST = 95
turnThresh = 150
exitThresh = 1500

maxRight = STRAIGHT_CONST + 30
maxLeft = STRAIGHT_CONST - 30
slightRight = STRAIGHT_CONST + 20
slightLeft = STRAIGHT_CONST - 20

wall_detector_boundary_area = (ROI1[2] - ROI1[0]) * (ROI1[3] - ROI1[1])

# Initial PID values
kp = 0.02
kd = 0.003
ki = 0.0
pid = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=STRAIGHT_CONST)
pid.output_limits = (maxLeft, maxRight)


class PIDTuningGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Autonomous Vehicle - PID Tuning & Live Feed")
        self.root.geometry("1200x800")

        # Variables for PID values
        self.kp_var = tk.DoubleVar(value=kp)
        self.ki_var = tk.DoubleVar(value=ki)
        self.kd_var = tk.DoubleVar(value=kd)

        # Variables for display info
        self.left_area_var = tk.StringVar(value="0")
        self.right_area_var = tk.StringVar(value="0")
        self.area_diff_var = tk.StringVar(value="0")
        self.angle_var = tk.StringVar(value="0")
        self.pid_output_var = tk.StringVar(value="0")

        # Queue for frame updates
        self.frame_queue = queue.Queue(maxsize=2)

        self.setup_gui()

        # Start the GUI update loop
        self.root.after(50, self.update_display)

    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # Left panel for controls
        control_frame = ttk.LabelFrame(main_frame, text="PID Controls", padding="10")
        control_frame.grid(
            row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10)
        )

        # PID sliders
        self.create_pid_slider(
            control_frame, "Kp (Proportional)", self.kp_var, 0, 0.5, 0.001, 0
        )
        self.create_pid_slider(
            control_frame, "Ki (Integral)", self.ki_var, 0, 0.1, 0.0001, 1
        )
        self.create_pid_slider(
            control_frame, "Kd (Derivative)", self.kd_var, 0, 0.05, 0.0001, 2
        )

        # Info display
        info_frame = ttk.LabelFrame(control_frame, text="Vehicle Info", padding="10")
        info_frame.grid(
            row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(20, 0)
        )

        ttk.Label(info_frame, text="Left Area:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.left_area_var).grid(
            row=0, column=1, sticky=tk.W, padx=(10, 0)
        )

        ttk.Label(info_frame, text="Right Area:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.right_area_var).grid(
            row=1, column=1, sticky=tk.W, padx=(10, 0)
        )

        ttk.Label(info_frame, text="Area Diff:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.area_diff_var).grid(
            row=2, column=1, sticky=tk.W, padx=(10, 0)
        )

        ttk.Label(info_frame, text="Target Angle:").grid(row=3, column=0, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.angle_var).grid(
            row=3, column=1, sticky=tk.W, padx=(10, 0)
        )

        ttk.Label(info_frame, text="PID Output:").grid(row=4, column=0, sticky=tk.W)
        ttk.Label(info_frame, textvariable=self.pid_output_var).grid(
            row=4, column=1, sticky=tk.W, padx=(10, 0)
        )

        # Reset button
        ttk.Button(control_frame, text="Reset PID", command=self.reset_pid).grid(
            row=4, column=0, columnspan=2, pady=(20, 0)
        )

        # Right panel for video feed
        video_frame = ttk.LabelFrame(main_frame, text="Live Camera Feed", padding="10")
        video_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Video display
        self.video_label = ttk.Label(video_frame)
        self.video_label.pack(expand=True)

    def create_pid_slider(self, parent, label, variable, from_, to, resolution, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=(0, 5))

        slider = ttk.Scale(
            parent,
            from_=from_,
            to=to,
            orient=tk.HORIZONTAL,
            variable=variable,
            command=lambda v: self.update_pid(),
        )
        slider.grid(row=row, column=1, sticky=(tk.W, tk.E), padx=(10, 0), pady=(0, 5))

        # Value display
        value_label = ttk.Label(parent, text=f"{variable.get():.4f}")
        value_label.grid(row=row, column=2, sticky=tk.W, padx=(10, 0), pady=(0, 5))

        # Update value display when slider changes
        def update_value_display(*args):
            value_label.config(text=f"{variable.get():.4f}")
            self.update_pid()

        variable.trace_add("write", update_value_display)

        parent.columnconfigure(1, weight=1)

    def update_pid(self):
        global pid
        pid.tunings = (self.kp_var.get(), self.ki_var.get(), self.kd_var.get())

    def reset_pid(self):
        global pid
        pid.reset()

    def update_display(self):
        # Update frame if available
        try:
            frame_data = self.frame_queue.get_nowait()
            frame_with_info, left_area, right_area, area_diff, angle, pid_output = (
                frame_data
            )

            # Convert frame to PhotoImage for tkinter
            frame_rgb = cv2.cvtColor(frame_with_info, cv2.COLOR_BGR2RGB)
            frame_pil = Image.fromarray(frame_rgb)
            # Resize for display
            frame_pil = frame_pil.resize((600, 450), Image.Resampling.LANCZOS)
            frame_photo = ImageTk.PhotoImage(frame_pil)

            self.video_label.configure(image=frame_photo)
            self.video_label.image = frame_photo  # Keep a reference

            # Update info displays
            self.left_area_var.set(f"{left_area:.1f}")
            self.right_area_var.set(f"{right_area:.1f}")
            self.area_diff_var.set(f"{area_diff:.1f}")
            self.angle_var.set(f"{angle:.1f}")
            self.pid_output_var.set(f"{pid_output:.1f}")

        except queue.Empty:
            pass

        # Schedule next update
        self.root.after(50, self.update_display)

    def run(self):
        self.root.mainloop()


def draw_roi_and_info(frame, left_area, right_area, area_diff, angle, pid_output):
    """Draw ROI rectangles and information overlay on frame"""
    frame_copy = frame.copy()

    # Draw ROI rectangles
    cv2.rectangle(
        frame_copy, (ROI1[0], ROI1[1]), (ROI1[2], ROI1[3]), (0, 255, 0), 2
    )  # Left ROI - Green
    cv2.rectangle(
        frame_copy, (ROI2[0], ROI2[1]), (ROI2[2], ROI2[3]), (0, 255, 0), 2
    )  # Right ROI - Green
    cv2.rectangle(
        frame_copy, (ROI3[0], ROI3[1]), (ROI3[2], ROI3[3]), (255, 0, 0), 2
    )  # Lap detection - Blue
    cv2.rectangle(
        frame_copy, (ROI4[0], ROI4[1]), (ROI4[2], ROI4[3]), (0, 0, 255), 2
    )  # Obstacle detection - Red

    # Add text overlay
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness = 2

    # Text information
    texts = [
        f"Left Area: {left_area:.1f}",
        f"Right Area: {right_area:.1f}",
        f"Area Diff: {area_diff:.1f}",
        f"Target Angle: {angle:.1f}",
        f"PID Output: {pid_output:.1f}",
        f"PID: P={pid.tunings[0]:.3f} I={pid.tunings[1]:.4f} D={pid.tunings[2]:.4f}",
    ]

    # Draw text background and text
    y_offset = 30
    for i, text in enumerate(texts):
        y_pos = y_offset + i * 25
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]

        # Draw background rectangle
        cv2.rectangle(
            frame_copy, (10, y_pos - 20), (text_size[0] + 20, y_pos + 5), (0, 0, 0), -1
        )

        # Draw text
        cv2.putText(
            frame_copy, text, (15, y_pos), font, font_scale, (255, 255, 255), thickness
        )

    return frame_copy


def main():
    global pid

    # Initialize PiCamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
    picam2.configure(config)
    picam2.start()

    # Initialize GUI
    gui = PIDTuningGUI()

    # Start worker threads
    threads = []
    workers = [
        contour_workers.left_contour_worker,
        contour_workers.right_contour_worker,
    ]

    for worker in workers:
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        threads.append(thread)

    print(f"Started {len(threads)} processing threads")

    time.sleep(2)  # Allow camera to warm up

    def processing_loop():
        """Main processing loop running in separate thread"""
        while True:
            try:
                # Capture frame
                frame = picam2.capture_array()

                # Convert RGB to BGR for OpenCV
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Distribute frame to all processing threads (non-blocking)
                frame_copy = copy.deepcopy(frame)
                contour_workers.put_frames_in_queues(frame_copy)

                # Retrieve all results from queues (non-blocking)
                (
                    left_result,
                    right_result,
                    _,
                    _,
                    _,
                    _,
                ) = contour_workers.collect_results()

                left_area = left_result.area
                right_area = right_result.area

                area_diff = (
                    (left_area - right_area) / wall_detector_boundary_area
                ) * 500  # limit from 0 to 500/-500
                area_mapped_angle = np.interp(
                    area_diff, [-500, 500], [maxLeft, maxRight]
                )
                angle = pid(area_mapped_angle)

                # Create frame with overlay information
                frame_with_info = draw_roi_and_info(
                    frame_bgr,
                    left_area,
                    right_area,
                    area_diff,
                    area_mapped_angle,
                    angle,
                )

                # Send frame to GUI (non-blocking)
                try:
                    gui.frame_queue.put_nowait(
                        (
                            frame_with_info,
                            left_area,
                            right_area,
                            area_diff,
                            area_mapped_angle,
                            angle,
                        )
                    )
                except queue.Full:
                    # If queue is full, skip this frame
                    try:
                        gui.frame_queue.get_nowait()  # Remove old frame
                        gui.frame_queue.put_nowait(
                            (
                                frame_with_info,
                                left_area,
                                right_area,
                                area_diff,
                                area_mapped_angle,
                                angle,
                            )
                        )
                    except queue.Empty:
                        pass

                # You can add your motor control code here
                # motor.set_angle(angle)

                time.sleep(0.033)  # ~30 FPS

            except Exception as e:
                print(f"Error in processing loop: {e}")
                time.sleep(0.1)

    # Start processing thread
    processing_thread = threading.Thread(target=processing_loop, daemon=True)
    processing_thread.start()

    try:
        # Run GUI (blocking)
        gui.run()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        picam2.stop()
        print("Camera stopped")


if __name__ == "__main__":
    main()
