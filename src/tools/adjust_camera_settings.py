import cv2
import numpy as np
from picamera2 import Picamera2
import time
import json
import os


class CameraController:
    def __init__(self):
        # Initialize PiCamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (800, 600)}
        )
        self.picam2.configure(config)

        # load the settings from the json file if it exists
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            tools_dir = os.path.join(script_dir, "..", "tools")
            tools_dir = os.path.abspath(tools_dir)
            with open(os.path.join(tools_dir, "camera_settings.json"), "r") as f:
                settings = json.load(f)
                self.exposure_time = settings.get("ExposureTime", 6000)
                self.analogue_gain = settings.get("AnalogueGain", 9.4)
                self.ae_enable = settings.get("AeEnable", False)
                self.awb_enable = settings.get("AwbEnable", False)
                frame_duration_limits = settings.get("FrameDurationLimits", [40000, 40000])
                self.frame_duration = frame_duration_limits[0]
                colour_gains = settings.get("ColourGains", [0.9, 1.1])
                self.red_gain = colour_gains[0]
                self.blue_gain = colour_gains[1]
                self.contrast = settings.get("Contrast", 1.1)
                self.saturation = settings.get("Saturation", 1.2)
                self.brightness = settings.get("Brightness", 0.0)
                self.sharpness = settings.get("Sharpness", 1.0)
                self.awb_mode = settings.get("AwbMode", 0)
                self.colour_temp = settings.get("ColourTemp", 3200)
                print("Loaded camera settings from camera_settings file.")
        except FileNotFoundError:
            print("No existing camera settings file found. Using default settings.")        

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
        cv2.resizeWindow("Camera Controls", 600, 600)

        # Exposure controls
        cv2.createTrackbar(
            "Exposure Time (x1000µs)",
            "Camera Controls",
            self.exposure_time // 1000,
            200,
            self.on_exposure_change,
        )

        cv2.createTrackbar(
            "Analogue Gain (x0.1)",
            "Camera Controls",
            int(self.analogue_gain * 10),
            639,
            self.on_gain_change,
        )

        cv2.createTrackbar(
            "Frame Duration (x1000µs)",
            "Camera Controls",
            self.frame_duration // 1000,
            200,
            self.on_frame_duration_change,
        )

        # Color controls
        cv2.createTrackbar(
            "Saturation (x0.1)",
            "Camera Controls",
            int(self.saturation * 10),
            320,
            self.on_saturation_change,
        )

        cv2.createTrackbar(
            "Contrast (x0.1)",
            "Camera Controls",
            int(self.contrast * 10),
            320,
            self.on_contrast_change,
        )

        cv2.createTrackbar(
            "Brightness (x100)",
            "Camera Controls",
            int((self.brightness + 1.0) * 100),
            200,
            self.on_brightness_change,
        )

        cv2.createTrackbar(
            "Sharpness (x0.1)",
            "Camera Controls",
            int(self.sharpness * 10),
            160,
            self.on_sharpness_change,
        )

        # White balance controls
        cv2.createTrackbar(
            "AWB Enable (0=Off, 1=On)",
            "Camera Controls",
            int(self.awb_enable),
            1,
            self.on_awb_enable_change,
        )

        cv2.createTrackbar(
            "AWB Mode",
            "Camera Controls",
            self.awb_mode,
            7,
            self.on_awb_mode_change,
        )

        cv2.createTrackbar(
            "Color Temp (x100K)",
            "Camera Controls",
            self.colour_temp // 100,
            1000,
            self.on_colour_temp_change,
        )

        cv2.createTrackbar(
            "Red Gain (x0.1)",
            "Camera Controls",
            int(self.red_gain * 10),
            320,
            self.on_red_gain_change,
        )

        cv2.createTrackbar(
            "Blue Gain (x0.1)",
            "Camera Controls",
            int(self.blue_gain * 10),
            320,
            self.on_blue_gain_change,
        )

        # Add text labels
        self.create_info_image()

    def create_info_image(self):
        """Create an info image for the control window"""
        info_img = np.zeros((600, 600, 3), dtype=np.uint8)

        font = cv2.FONT_HERSHEY_SIMPLEX
        y_pos = 20
        line_height = 25

        # Title
        cv2.putText(
            info_img,
            "Camera Control Panel - Color Tuning",
            (10, y_pos),
            font,
            0.6,
            (255, 255, 255),
            2,
        )
        y_pos += line_height + 10

        # Exposure settings
        cv2.putText(
            info_img, "EXPOSURE SETTINGS:", (10, y_pos), font, 0.5, (0, 255, 0), 2
        )
        y_pos += line_height
        cv2.putText(
            info_img,
            f"  Exposure: {self.exposure_time}µs",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height - 5
        cv2.putText(
            info_img,
            f"  Gain: {self.analogue_gain:.1f}",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height - 5
        cv2.putText(
            info_img,
            f"  Frame Duration: {self.frame_duration}µs",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height + 5

        # Color settings
        cv2.putText(
            info_img, "COLOR SETTINGS:", (10, y_pos), font, 0.5, (255, 200, 0), 2
        )
        y_pos += line_height
        cv2.putText(
            info_img,
            f"  Saturation: {self.saturation:.1f}",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height - 5
        cv2.putText(
            info_img,
            f"  Contrast: {self.contrast:.1f}",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height - 5
        cv2.putText(
            info_img,
            f"  Brightness: {self.brightness:.2f}",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height - 5
        cv2.putText(
            info_img,
            f"  Sharpness: {self.sharpness:.1f}",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height + 5

        # White balance settings
        cv2.putText(
            info_img, "WHITE BALANCE:", (10, y_pos), font, 0.5, (0, 200, 255), 2
        )
        y_pos += line_height

        awb_status = "ON" if self.awb_enable else "OFF"
        awb_modes = [
            "Manual",
            "Auto",
            "Tungsten",
            "Fluorescent",
            "Indoor",
            "Daylight",
            "Cloudy",
            "Custom",
        ]
        awb_mode_name = (
            awb_modes[self.awb_mode]
            if self.awb_mode < len(awb_modes)
            else f"Mode {self.awb_mode}"
        )

        cv2.putText(
            info_img,
            f"  AWB: {awb_status} | Mode: {awb_mode_name}",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height - 5
        cv2.putText(
            info_img,
            f"  Color Temp: {self.colour_temp}K",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height - 5
        cv2.putText(
            info_img,
            f"  Red Gain: {self.red_gain:.1f}",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height - 5
        cv2.putText(
            info_img,
            f"  Blue Gain: {self.blue_gain:.1f}",
            (10, y_pos),
            font,
            0.4,
            (255, 255, 255),
            1,
        )
        y_pos += line_height + 10

        # Tips section
        cv2.putText(info_img, "QUICK GUIDE:", (10, y_pos), font, 0.5, (0, 255, 255), 2)
        y_pos += line_height

        tips = [
            "Saturation: 0=grayscale, 1=normal, >1=vibrant",
            "Contrast: 0=flat, 1=normal, >1=high contrast",
            "Brightness: -1=dark, 0=normal, 1=bright",
            "Sharpness: 0=soft, 1=normal, >1=sharp",
            "AWB Enable: Turn on for auto white balance",
            "Color Temp: 2500K=warm, 6500K=daylight",
            "Manual WB: Disable AWB, adjust R/B gains",
            "Red/Blue Gains: Higher=more of that color",
        ]

        for tip in tips:
            cv2.putText(
                info_img, f"  {tip}", (10, y_pos), font, 0.35, (200, 200, 200), 1
            )
            y_pos += line_height - 7

        y_pos += 10
        cv2.putText(
            info_img,
            "CONTROLS: S=Save | R=Reset | Q/ESC=Quit",
            (10, y_pos),
            font,
            0.4,
            (0, 0, 255),
            1,
        )

        info_img = cv2.cvtColor(info_img, cv2.COLOR_BGR2RGB)
        cv2.imshow("Camera Controls", info_img)

    # Callback functions
    def on_exposure_change(self, val):
        self.exposure_time = val * 1000
        self.update_camera_controls()
        self.update_info_display()

    def on_gain_change(self, val):
        self.analogue_gain = val / 10.0
        self.update_camera_controls()
        self.update_info_display()

    def on_frame_duration_change(self, val):
        self.frame_duration = val * 1000
        self.update_camera_controls()
        self.update_info_display()

    def on_saturation_change(self, val):
        self.saturation = val / 10.0
        self.update_camera_controls()
        self.update_info_display()

    def on_contrast_change(self, val):
        self.contrast = val / 10.0
        self.update_camera_controls()
        self.update_info_display()

    def on_brightness_change(self, val):
        self.brightness = (val / 100.0) - 1.0
        self.update_camera_controls()
        self.update_info_display()

    def on_sharpness_change(self, val):
        self.sharpness = val / 10.0
        self.update_camera_controls()
        self.update_info_display()

    def on_awb_enable_change(self, val):
        self.awb_enable = bool(val)
        self.update_camera_controls()
        self.update_info_display()

    def on_awb_mode_change(self, val):
        self.awb_mode = val
        self.update_camera_controls()
        self.update_info_display()

    def on_colour_temp_change(self, val):
        self.colour_temp = val * 100
        self.update_camera_controls()
        self.update_info_display()

    def on_red_gain_change(self, val):
        self.red_gain = val / 10.0
        self.update_camera_controls()
        self.update_info_display()

    def on_blue_gain_change(self, val):
        self.blue_gain = val / 10.0
        self.update_camera_controls()
        self.update_info_display()

    def update_camera_controls(self):
        """Update camera controls with current values"""
        try:
            controls = {
                "ExposureTime": self.exposure_time,
                "AnalogueGain": self.analogue_gain,
                "AeEnable": False,
                "FrameDurationLimits": (self.frame_duration, self.frame_duration),
                "Saturation": self.saturation,
                "Contrast": self.contrast,
                "Brightness": self.brightness,
                "Sharpness": self.sharpness,
            }

            # save the current settings to a json file
            settings = {
                "ExposureTime": self.exposure_time,
                "AnalogueGain": self.analogue_gain,
                "AeEnable": self.ae_enable,
                "AwbEnable": self.awb_enable,
                "FrameDurationLimits": (self.frame_duration, self.frame_duration),
                "ColourGains": [self.red_gain, self.blue_gain],
                "Contrast": self.contrast,
                "Saturation": self.saturation,
                "Brightness": self.brightness,
            }

            # write to settings.json
            script_path = os.path.abspath(__file__)
            script_dir = os.path.dirname(script_path)

            # write in tools directory
            with open(os.path.join(script_dir, "camera_settings.json"), "w") as f:
                json.dump(settings, f, indent=4)
            
            # write in raspberry directory            
            data_dir = os.path.join(script_dir, "..", "raspberrypi")
            data_dir = os.path.abspath(data_dir)

            with open(os.path.join(data_dir, "camera_settings.json"), "w") as f:
                json.dump(settings, f, indent=4)

            # Add white balance controls
            if self.awb_enable:
                controls["AwbEnable"] = True
                controls["AwbMode"] = self.awb_mode
            else:
                controls["AwbEnable"] = False
                controls["ColourGains"] = (self.red_gain, self.blue_gain)

            self.picam2.set_controls(controls)
        except Exception as e:
            print(f"Error setting camera controls: {e}")

    def update_info_display(self):
        """Update the info display with current values"""
        self.create_info_image()

    def capture_and_display(self):
        """Capture and display camera frames"""
        while self.running:
            try:
                frame = self.picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.add_settings_overlay(frame_bgr)
                frame_bgr = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2BGR)
                cv2.imshow("Camera Feed", frame_bgr)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    self.running = False
                    break
                elif key == ord("s"):
                    timestamp = int(time.time())
                    filename = f"capture_{timestamp}.jpg"
                    cv2.imwrite(filename, frame_bgr)
                    print(f"Frame saved as {filename}")
                    print(
                        f"Settings: Sat={self.saturation:.1f}, Cont={self.contrast:.1f}, "
                        f"Bright={self.brightness:.2f}, Sharp={self.sharpness:.1f}, "
                        f"AWB={self.awb_enable}, R={self.red_gain:.1f}, B={self.blue_gain:.1f}"
                    )
                elif key == ord("r"):
                    self.reset_to_defaults()

            except Exception as e:
                print(f"Error capturing frame: {e}")
                time.sleep(0.1)

    def add_settings_overlay(self, frame):
        """Add current settings as overlay on the frame"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (380, 55), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        y = 28        
        cv2.putText(
            frame,
            "S=Save | R=Reset | Q/ESC=Quit",
            (15, y),
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
        self.saturation = 1.0
        self.contrast = 1.0
        self.brightness = 0.0
        self.sharpness = 1.0
        self.awb_enable = False
        self.awb_mode = 0
        self.colour_temp = 3200
        self.red_gain = 2.0
        self.blue_gain = 2.0

        # Update all trackbars
        cv2.setTrackbarPos(
            "Exposure Time (x1000µs)", "Camera Controls", self.exposure_time // 1000
        )
        cv2.setTrackbarPos(
            "Analogue Gain (x0.1)", "Camera Controls", int(self.analogue_gain * 10)
        )
        cv2.setTrackbarPos(
            "Frame Duration (x1000µs)", "Camera Controls", self.frame_duration // 1000
        )
        cv2.setTrackbarPos(
            "Saturation (x0.1)", "Camera Controls", int(self.saturation * 10)
        )
        cv2.setTrackbarPos(
            "Contrast (x0.1)", "Camera Controls", int(self.contrast * 10)
        )
        cv2.setTrackbarPos(
            "Brightness (x100)", "Camera Controls", int((self.brightness + 1.0) * 100)
        )
        cv2.setTrackbarPos(
            "Sharpness (x0.1)", "Camera Controls", int(self.sharpness * 10)
        )
        cv2.setTrackbarPos(
            "AWB Enable (0=Off, 1=On)", "Camera Controls", int(self.awb_enable)
        )
        cv2.setTrackbarPos("AWB Mode", "Camera Controls", self.awb_mode)
        cv2.setTrackbarPos(
            "Color Temp (x100K)", "Camera Controls", self.colour_temp // 100
        )
        cv2.setTrackbarPos(
            "Red Gain (x0.1)", "Camera Controls", int(self.red_gain * 10)
        )
        cv2.setTrackbarPos(
            "Blue Gain (x0.1)", "Camera Controls", int(self.blue_gain * 10)
        )

        self.update_camera_controls()
        self.update_info_display()
        print("Settings reset to defaults")

    def start(self):
        """Start the camera controller"""
        print("=== Camera Color Tuning Controller ===")
        print("All color-affecting parameters are now available!")
        print("\nControls:")
        print("- Use sliders to adjust all color parameters")
        print("- Press 'S' to save frame with current settings")
        print("- Press 'R' to reset to defaults")
        print("- Press 'Q' or ESC to quit")
        print("\nTip: Disable AWB for manual white balance control")

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
            "Make sure you're running this on a Raspberry Pi with OV5647 camera connected"
        )
