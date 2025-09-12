import cv2
import numpy as np
from picamera2 import Picamera2
import threading
import time


class CameraController:
    def __init__(self):
        # Initialize PiCamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)}
        )
        self.picam2.configure(config)

        # Initial camera settings
        self.exposure_time = 16000
        self.analogue_gain = 42.0
        self.frame_duration = 40000

        # Set initial controls
        self.update_camera_controls()
        self.picam2.start()

        # Control window setup
        self.setup_control_window()

        # Threading control
        self.running = True

    def setup_control_window(self):
        """Setup the control window with trackbars"""
        cv2.namedWindow("Camera Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Controls", 500, 300)

        # Create trackbars with reasonable ranges
        # ExposureTime: 1000 to 200000 microseconds (1ms to 200ms)
        cv2.createTrackbar(
            "Exposure Time (x1000µs)",
            "Camera Controls",
            self.exposure_time // 1000,
            200,
            self.on_exposure_change,
        )

        # AnalogueGain: 1.0 to 100.0 (multiplied by 10 for trackbar)
        cv2.createTrackbar(
            "Analogue Gain (x0.1)",
            "Camera Controls",
            int(self.analogue_gain * 10),
            1000,
            self.on_gain_change,
        )

        # FrameDurationLimits: 10000 to 200000 microseconds (10ms to 200ms)
        cv2.createTrackbar(
            "Frame Duration (x1000µs)",
            "Camera Controls",
            self.frame_duration // 1000,
            200,
            self.on_frame_duration_change,
        )

        # Add text labels
        self.create_info_image()

    def create_info_image(self):
        """Create an info image for the control window"""
        info_img = np.zeros((300, 500, 3), dtype=np.uint8)

        # Add text information
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(
            info_img, "Camera Control Panel", (10, 30), font, 0.7, (255, 255, 255), 2
        )
        cv2.putText(
            info_img,
            "Adjust sliders to change camera settings",
            (10, 60),
            font,
            0.5,
            (200, 200, 200),
            1,
        )

        cv2.putText(
            info_img, f"Current Settings:", (10, 100), font, 0.6, (0, 255, 0), 2
        )
        cv2.putText(
            info_img,
            f"Exposure: {self.exposure_time}µs",
            (10, 130),
            font,
            0.5,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            info_img,
            f"Gain: {self.analogue_gain:.1f}",
            (10, 150),
            font,
            0.5,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            info_img,
            f"Frame Duration: {self.frame_duration}µs",
            (10, 170),
            font,
            0.5,
            (255, 255, 255),
            1,
        )

        cv2.putText(info_img, "Tips:", (10, 210), font, 0.6, (0, 255, 255), 2)
        cv2.putText(
            info_img,
            "- Lower exposure = less motion blur",
            (10, 230),
            font,
            0.4,
            (200, 200, 200),
            1,
        )
        cv2.putText(
            info_img,
            "- Higher gain = brighter but more noise",
            (10, 245),
            font,
            0.4,
            (200, 200, 200),
            1,
        )
        cv2.putText(
            info_img,
            "- Frame duration affects FPS",
            (10, 260),
            font,
            0.4,
            (200, 200, 200),
            1,
        )
        cv2.putText(
            info_img, "Press ESC or Q to quit", (10, 285), font, 0.4, (0, 0, 255), 1
        )

        # convert to RGB
        info_img = cv2.cvtColor(info_img, cv2.COLOR_BGR2RGB)
        cv2.imshow("Camera Controls", info_img)

    def on_exposure_change(self, val):
        """Callback for exposure time trackbar"""
        self.exposure_time = val * 1000  # Convert back to microseconds
        self.update_camera_controls()
        self.update_info_display()

    def on_gain_change(self, val):
        """Callback for analogue gain trackbar"""
        self.analogue_gain = val / 10.0  # Convert back to float
        self.update_camera_controls()
        self.update_info_display()

    def on_frame_duration_change(self, val):
        """Callback for frame duration trackbar"""
        self.frame_duration = val * 1000  # Convert back to microseconds
        self.update_camera_controls()
        self.update_info_display()

    def update_camera_controls(self):
        """Update camera controls with current values"""
        try:
            self.picam2.set_controls(
                {
                    "ExposureTime": self.exposure_time,
                    "AnalogueGain": self.analogue_gain,
                    "AeEnable": False,
                    "AwbEnable": False,
                    "FrameDurationLimits": (self.frame_duration, self.frame_duration),
                }
            )
        except Exception as e:
            print(f"Error setting camera controls: {e}")

    def update_info_display(self):
        """Update the info display with current values"""
        self.create_info_image()

    def capture_and_display(self):
        """Capture and display camera frames"""
        while self.running:
            try:
                # Capture frame
                frame = self.picam2.capture_array()

                # Convert RGB to BGR for OpenCV
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Add overlay with current settings
                self.add_settings_overlay(frame_bgr)

                # Display the frame
                cv2.imshow("Camera Feed", frame_bgr)

                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:  # ESC or Q
                    self.running = False
                    break
                elif key == ord("s"):  # Save current frame
                    timestamp = int(time.time())
                    filename = f"capture_{timestamp}.jpg"
                    cv2.imwrite(filename, frame_bgr)
                    print(f"Frame saved as {filename}")
                elif key == ord("r"):  # Reset to default values
                    self.reset_to_defaults()

            except Exception as e:
                print(f"Error capturing frame: {e}")
                time.sleep(0.1)

    def add_settings_overlay(self, frame):
        """Add current settings as overlay on the frame"""
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Semi-transparent background for text
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (350, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        # Add text
        cv2.putText(
            frame,
            f"Exposure: {self.exposure_time}us",
            (15, 30),
            font,
            0.5,
            (0, 255, 0),
            1,
        )
        cv2.putText(
            frame,
            f"Gain: {self.analogue_gain:.1f}",
            (15, 50),
            font,
            0.5,
            (0, 255, 0),
            1,
        )
        cv2.putText(
            frame,
            f"Frame Duration: {self.frame_duration}us",
            (15, 70),
            font,
            0.5,
            (0, 255, 0),
            1,
        )
        cv2.putText(
            frame,
            "Press S to save, R to reset, Q/ESC to quit",
            (15, 90),
            font,
            0.4,
            (255, 255, 255),
            1,
        )

    def reset_to_defaults(self):
        """Reset camera settings to default values"""
        self.exposure_time = 16000
        self.analogue_gain = 42.0
        self.frame_duration = 40000

        # Update trackbars
        cv2.setTrackbarPos(
            "Exposure Time (x1000µs)", "Camera Controls", self.exposure_time // 1000
        )
        cv2.setTrackbarPos(
            "Analogue Gain (x0.1)", "Camera Controls", int(self.analogue_gain * 10)
        )
        cv2.setTrackbarPos(
            "Frame Duration (x1000µs)", "Camera Controls", self.frame_duration // 1000
        )

        # Update camera
        self.update_camera_controls()
        self.update_info_display()
        print("Settings reset to defaults")

    def start(self):
        """Start the camera controller"""
        print("Camera Controller Started!")
        print("Controls:")
        print("- Use sliders in 'Camera Controls' window to adjust settings")
        print("- Press 'S' to save current frame")
        print("- Press 'R' to reset to default settings")
        print("- Press 'Q' or ESC to quit")

        try:
            self.capture_and_display()
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        self.running = False
        self.picam2.stop()
        cv2.destroyAllWindows()
        print("Camera stopped and windows closed.")


if __name__ == "__main__":
    try:
        controller = CameraController()
        controller.start()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        print(
            "Make sure you're running this on a Raspberry Pi with camera module connected"
        )
