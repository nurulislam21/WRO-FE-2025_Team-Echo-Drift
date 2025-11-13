import time
import matplotlib.pyplot as plt
import math
from typing import List, Tuple


class OdometryTracker:
    """
    A class to track robot position and orientation using encoder and gyroscope data.
    """

    def __init__(
        self,
        wheel_radius: float = 0.033,
        ticks_per_rev: int = 1000,
        gear_ratio: float = 1.0,
    ):
        """
        Initialize the odometry tracker.

        Args:
            wheel_radius: Wheel radius in meters (default: 0.033m = 33mm)
            ticks_per_rev: Encoder ticks per wheel revolution (default: 1000)
            gear_ratio: Gear ratio if gearbox is present (default: 1.0)
        """
        # Robot parameters
        self.wheel_radius = wheel_radius
        self.ticks_per_rev = ticks_per_rev
        self.gear_ratio = gear_ratio

        # Position and orientation state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # History tracking
        self.positions: List[Tuple[float, float]] = [(0.0, 0.0)]
        self.prev_ticks = 0

    def reset_position(self):
        """Reset position and orientation to origin."""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.positions = [(0.0, 0.0)]
        self.prev_ticks = 0

    def get_distance_from_ticks(self, ticks: int) -> float:
        """
        Calculate distance traveled from encoder ticks.

        Args:
            ticks: Number of encoder ticks

        Returns:
            Distance in meters
        """
        return (
            2
            * math.pi
            * self.wheel_radius
            * (ticks / self.ticks_per_rev)
            / self.gear_ratio
        )

    def update(self, ticks: int, gyro_angle: float):
        """
        Update position based on encoder ticks and gyroscope reading.

        Args:
            ticks: Current encoder tick count
            gyro_angle: Current gyroscope angle in degrees
        """
        # Calculate distance traveled since last update
        delta_ticks = ticks - self.prev_ticks
        distance = self.get_distance_from_ticks(delta_ticks)
        self.prev_ticks = ticks

        # Update orientation and position
        self.theta = math.radians(gyro_angle)
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        # Store position history
        self.positions.append((self.x, self.y))

    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current position and orientation.

        Returns:
            Tuple of (x, y, theta) where theta is in radians
        """
        return (self.x, self.y, self.theta)

    def get_position_history(self, index=None) -> List[Tuple[float, float]]:
        """
        Get complete position history.

        Returns:
            List of (x, y) tuples
        """
        if index is not None:
            return self.positions[index]
        return self.positions.copy()


class OdometryVisualizer:
    """
    A class to visualize odometry data in real-time.
    """

    def __init__(
        self, title: str = "Odometry Path", start_zone_rect: list = [0.55, 1.5]
    ):
        """
        Initialize the visualizer.

        Args:
            title: Plot title
        """
        self.title = title
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()

        # Move the plot window to top-left corner
        self._move_window_top_left()

        self.start_zone_rect_x = start_zone_rect[0]
        self.start_zone_rect_y = start_zone_rect[1]

    def _move_window_top_left(self):
        """Attempt to move the Matplotlib window to the top-left corner of the screen."""
        try:
            # Tkinter backend
            self.fig.canvas.manager.window.wm_geometry("+0+0")
        except Exception:
            try:
                # Qt backend
                self.fig.canvas.manager.window.move(0, 0)
            except Exception:
                # Other or unsupported backends (e.g., Agg)
                pass

    def update_plot(self, positions: List[Tuple[float, float]]):
        """
        Update the plot with new position data.

        Args:
            positions: List of (x, y) position tuples
        """
        self.ax.clear()

        if len(positions) > 0:
            xs, ys = zip(*positions)
            self.ax.plot(xs, ys, marker=".", linewidth=1.5, markersize=4)

            # draw a zone on start point
            self.ax.add_patch(
                plt.Rectangle(
                    (0 - self.start_zone_rect_x, 0 - self.start_zone_rect_y),
                    2 * self.start_zone_rect_x,
                    2 * self.start_zone_rect_y,
                    color="green",
                    alpha=0.3,
                )
            )

        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title(self.title)
        self.ax.axis("equal")
        self.ax.grid(True, alpha=0.3)

        plt.pause(0.01)

    def close(self):
        """Close the plot window."""
        plt.ioff()
        plt.close()


def clamp_angle(totalAngle, threshold=5):
    """Clamp totalAngle to nearest multiple of 90 if within the threshold."""
    # Find the nearest multiple of 90
    nearest = round(totalAngle / 90) * 90

    # If the difference is within the threshold, return the clamped value
    if abs(totalAngle - nearest) <= threshold:
        return nearest

    # Otherwise return original
    return totalAngle


def main():
    """Main function to run odometry tracking simulation."""

    # Sample odometry data (ticks, gyro_angle)
    odometry_data = [
        (2706, 0),
        (3221, 0),
        (5546, -40.49),
        (6932, -25.56),
        (10165, 0),
        (11927, 0),
        (14835, 0),
        (16226, 0),
        (18639, -36.93),
        (19768, -57.7),
        (22387, -38.5),
        (23570, -52.28),
        (26139, -90),
        (27903, -90),
        (31665, -90),
        (33575, -90),
        (37138, -90),
        (38463, -90),
        (40726, -117.02),
        (41848, -90),
        (44521, -90),
        (46006, -116.72),
        (48666, -146.7),
        (50067, -180),
        (51824, -159.21),
        (55456, -180),
        (57260, -180),
        (60662, -180),
        (62068, -206.6),
        (64642, -180),
        (65884, -180),
        (68859, -180),
        (70160, -219.2),
        (71509, -237.79),
        (75064, -235.85),
        (76844, -243.67),
        (80608, -244.32),
        (82476, -246.55),
        (85740, -270),
        (87067, -249.1),
        (89370, -270),
        (90463, -270),
        (91928, -270),
        (94895, -270),
        (96775, -270),
        (99403, -297.83),
        (100522, -320.66),
        (102894, -360),
        (104504, -360),
        (107808, -337.74),
        (109611, -360),
        (113248, -338.14),
        (114630, -360),
        (117001, -360),
        (118332, -338.04),
        (120702, -360),
        (121874, -394.85),
        (124814, -385.03),
        (126035, -405.23),
        (127432, -421.35),
        (131075, -418.79),
        (133004, -417.31),
        (136841, -416.91),
        (138412, -428.01),
        (140851, -450),
        (141981, -450),
        (144325, -450),
        (145548, -450),
        (147282, -450),
        (150135, -486.21),
        (151590, -499.15),
        (155143, -493.03),
        (156858, -501.57),
        (160503, -505.56),
        (161875, -540),
        (164347, -516.0),
        (165745, -503.57),
        (168252, -540),
        (169608, -540),
        (172651, -563.83),
        (174380, -570.25),
        (177911, -583.41),
        (179813, -579.47),
        (183492, -589.27),
        (185356, -591.33),
        (188205, -630),
        (189380, -604.1),
        (190673, -589.74),
        (193139, -630),
        (194741, -630),
        (197329, -667.58),
        (198817, -661.84),
        (202185, -662.71),
        (203975, -671.69),
        (207705, -668.55),
        (209194, -691.65),
        (210533, -695.58),
        (213026, -671.9),
        (214152, -696.14),
        (216864, -720),
        (218202, -720),
        (220830, -746.19),
        (222268, -720),
        (224944, -759.39),
        (226673, -762.39),
        (230429, -755.4),
        (232348, -756.61),
        (235482, -784.48),
        (236734, -787.63),
        (239116, -767.67),
        (240180, -787.72),
        (243518, -810),
        (245343, -810),
        (247956, -837.7),
        (249457, -845.96),
        (251183, -839.27),
        (254815, -843.02),
        (256545, -853.45),
        (259918, -858.77),
        (261189, -900),
        (263512, -865.18),
        (264744, -847.3),
        (268352, -844.89),
        (269922, -853.78),
        (272363, -900),
        (273458, -920.45),
        (275805, -962.18),
        (277409, -967.73),
        (278783, -947.77),
        (281905, -931.1),
        (283779, -930.48),
        (287380, -928.43),
        (288701, -952.37),
        (291004, -940.28),
        (292358, -927.72),
        (294841, -963.68),
        (296300, -966.46),
        (299162, -990),
        (300954, -990),
        (304426, -1013.39),
        (306242, -990),
        (309773, -1016.18),
        (311524, -1022.25),
        (314505, -1032.91),
        (315658, -1054.84),
        (318016, -1029.87),
    ]

    lap_samples = {}

    # Initialize tracker and visualizer
    tracker = OdometryTracker(wheel_radius=0.046, ticks_per_rev=2220, gear_ratio=1.0)
    visualizer = OdometryVisualizer(
        title="Odometry Path (Single Encoder + Gyro)", start_zone_rect=[0.55, 2]
    )

    print("Starting odometry simulation...")
    time.sleep(1)

    try:
        lap = 0
        last_time = time.time()
        interval = 0.5  # seconds

        smoothed_drift_x = 0.0
        smoothed_drift_y = 0.0
        alpha = 0.3  # between 0.05–0.3, depending on how noisy your drift is
        MAX_DRIFT_THRESHOLD = 0.5

        for ticks, gyro_angle in odometry_data:
            # Update tracker with new data
            tracker.update(ticks, gyro_angle)
            # Get current position
            x, y, theta = tracker.get_position()
            if lap_samples.get(lap) is None:
                lap_samples[lap] = []
            lap_samples[lap].append((x, y))
            # x -= offset_x
            # y -= offset_y
            # print(f"Position: x={x:.3f}m, y={y:.3f}m, θ={math.degrees(theta):.1f}°")

            if lap > 0:
                # skip correction for first few samples
                if len(lap_samples[lap]) > 6:
                    # get closest point to (x, y) from previous lap, because robot starts from a different position
                    prev_lap_points = lap_samples[lap - 1]
                    closest_point = min(
                        prev_lap_points,
                        key=lambda p: math.sqrt((p[0] - x) ** 2 + (p[1] - y) ** 2),
                    )
                    # Raw drift measurement
                    new_drift_x = x - closest_point[0]
                    new_drift_y = y - closest_point[1]
                    drift_mag = math.sqrt(new_drift_x**2 + new_drift_y**2)
                    if drift_mag < MAX_DRIFT_THRESHOLD:

                        # Smoothed drift (exponential moving average)
                        smoothed_drift_x = (
                            alpha * new_drift_x + (1 - alpha) * smoothed_drift_x
                        )
                        smoothed_drift_y = (
                            alpha * new_drift_y + (1 - alpha) * smoothed_drift_y
                        )

                        # Apply partial correction
                        correction_rate = 0.25
                        tracker.x -= smoothed_drift_x * correction_rate
                        tracker.y -= smoothed_drift_y * correction_rate

            if time.time() - last_time >= interval and (
                abs(x) <= visualizer.start_zone_rect_x
                and abs(y) <= visualizer.start_zone_rect_y
            ):
                last_time = time.time()
                lap += 1
                print(f"Lap {lap} completed.")
            # Update visualization
            visualizer.update_plot(tracker.get_position_history())

            # time.sleep(0.01)

        print("\nSimulation complete!")
        print(f"Final position: x={tracker.x:.3f}m, y={tracker.y:.3f}m")

        # Keep plot open
        plt.ioff()
        plt.show()

    except KeyboardInterrupt:
        print("\nOdometry stopped by user.")
    finally:
        visualizer.close()


if __name__ == "__main__":
    main()
