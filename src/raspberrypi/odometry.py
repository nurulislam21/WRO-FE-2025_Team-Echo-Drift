import time
import matplotlib.pyplot as plt
import math
from typing import List, Tuple
import numpy as np


class OdometryTracker:
    """
    A class to track robot position and orientation using encoder and gyroscope data.
    """

    def __init__(
        self,
        wheel_radius: float = 0.033,
        ticks_per_rev: int = 1000,
        gear_ratio: float = 1.0,
        debug: bool = False,
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
        self.debug = debug

        # Position and orientation state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # History tracking
        self.positions: List[Tuple[float, float]] = [(0.0, 0.0)]
        self.prev_ticks = 0

        # open a log file if debug is enabled
        if self.debug:
            self.log_file = open("odometry_debug_log.txt", "w")

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

        if self.debug:
            log_entry = f"({ticks}, {gyro_angle}),"
            self.log_file.write(log_entry + "\n")

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
    
    def close(self):
        """Close any resources."""
        if self.debug:
            self.log_file.close()


class OdometryVisualizer:
    """
    A class to visualize odometry data in real-time with inner and outer boundaries.
    """

    def __init__(
        self,
        title: str = "Odometry Path",
        start_zone_rect: list = [0.55, 1.5],
        inner_margin: float = 1,  # Default margin from outer boundary
        debug: bool = False,
    ):
        """
        Initialize the visualizer.

        Args:
            title: Plot title
            start_zone_rect: [width, height] of start zone
            inner_margin: Margin between outer and inner boundaries
        """
        self.title = title
        self.debug = debug

        if self.debug:
            plt.ion()  # Enable interactive mode
            self.fig, self.ax = plt.subplots()
        

        # Move the plot window to top-left corner
        self._move_window_top_left()

        self.start_zone_rect_x = start_zone_rect[0]
        self.start_zone_rect_y = start_zone_rect[1]
        self.direction = "auto"

        # Boundary line coordinates
        self.padding = 0.3
        self.inner_margin = inner_margin

        # Outer boundary
        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None

        # Inner boundary (will be computed from outer)
        self.inner_x_min = None
        self.inner_x_max = None
        self.inner_y_min = None
        self.inner_y_max = None

        # middle boundary
        self.middle_x_min = None
        self.middle_x_max = None
        self.middle_y_min = None
        self.middle_y_max = None        

    def set_dir(self, dir: str):
        """
        Set direction for predefined boundaries.

        Args:
            - 'ccw', 'cw' for obstacle challenge,
            - 'auto' for open challenge
        """
        self.direction = dir
        if dir == "cw":
            self.x_min = -1
            self.x_max = 1
            self.y_min = -3
            self.y_max = 0

        elif dir == "ccw":
            self.x_min = -1
            self.x_max = 1
            self.y_min = 0
            self.y_max = 3
        else:
            self.x_min = None
            self.x_max = None
            self.y_min = None
            self.y_max = None

        # Update inner boundary when outer is set
        self._update_inner_boundary()

    def _move_window_top_left(self):
        if self.debug:            
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

    def _update_inner_boundary(self):
        """Calculate inner boundary based on outer boundary and margins."""
        if all(v is not None for v in [self.x_min, self.x_max, self.y_min, self.y_max]):
            # total_margin = self.inner_margin + self.inner_margin_adjust
            # always set inner boundary to be 1/3 of outer boundary size
            width = self.x_max - self.x_min
            height = self.y_max - self.y_min            
            margin_x = (width) / 3
            margin_y = (height) / 3
            
            self.inner_x_min = self.x_min + margin_x
            self.inner_x_max = self.x_max - margin_x
            self.inner_y_min = self.y_min + margin_y
            self.inner_y_max = self.y_max - margin_y

            # Middle boundary (for reference)
            self.middle_x_min = (self.x_min + self.inner_x_min) / 2
            self.middle_x_max = (self.x_max + self.inner_x_max) / 2
            self.middle_y_min = (self.y_min + self.inner_y_min) / 2
            self.middle_y_max = (self.y_max + self.inner_y_max) / 2


    def compute_best_fit_boundaries(
        self, positions: List[Tuple[float, float]], percentile: float = 95
    ):
        """
        Compute best-fit outer and inner boundaries from position data.

        Args:
            positions: List of (x, y) tuples
            percentile: Percentile to use for boundary detection (to ignore outliers)
        """
        if len(positions) < 10:
            return

        xs, ys = zip(*positions)
        xs, ys = np.array(xs), np.array(ys)

        # Use percentile to be robust against outliers
        lower_p = (100 - percentile) / 2
        upper_p = 100 - lower_p

        self.x_min = np.percentile(xs, lower_p)
        self.x_max = np.percentile(xs, upper_p)
        self.y_min = np.percentile(ys, lower_p)
        self.y_max = np.percentile(ys, upper_p)

        # Update inner boundary
        self._update_inner_boundary()    
    
    def intersects_middle_rectangle(self, ax, ay, bx, by):
        # 1. Check if A or B lies inside the rectangle
        if (self.middle_x_min <= ax <= self.middle_x_max and self.middle_y_min <= ay <= self.middle_y_max) or \
        (self.middle_x_min <= bx <= self.middle_x_max and self.middle_y_min <= by <= self.middle_y_max):
            return True

        # Helper: Check line segment intersection
        def ccw(x1, y1, x2, y2, x3, y3):
            return (y3 - y1) * (x2 - x1) > (y2 - y1) * (x3 - x1)

        def intersect(p1, p2, p3, p4):
            return (ccw(*p1, *p3, *p4) != ccw(*p2, *p3, *p4)) and \
                (ccw(*p1, *p2, *p3) != ccw(*p1, *p2, *p4))

        # 2. Rectangle edges as line segments
        edges = [
            ((self.middle_x_min, self.middle_y_min), (self.middle_x_max, self.middle_y_min)),  # bottom
            ((self.middle_x_max, self.middle_y_min), (self.middle_x_max, self.middle_y_max)),  # right
            ((self.middle_x_max, self.middle_y_max), (self.middle_x_min, self.middle_y_max)),  # top
            ((self.middle_x_min, self.middle_y_max), (self.middle_x_min, self.middle_y_min)),  # left
        ]

        for e1, e2 in edges:
            if intersect((ax, ay), (bx, by), e1, e2):
                return True

        return False

    def get_boundary_vector_proximity(self, prev_x: float, prev_y: float, x: float, y: float) -> str:
        if self.intersects_middle_rectangle(prev_x, prev_y, x, y):
            print("intersects middle rectangle")
            ref_x = (self.middle_x_min + self.middle_x_max) / 2
            ref_y = (self.middle_y_min + self.middle_y_max) / 2
            current_dist = math.sqrt((x - ref_x) ** 2 + (y - ref_y) ** 2)
            prev_dist = math.sqrt((prev_x - ref_x) ** 2 + (prev_y - ref_y) ** 2)
            if current_dist < prev_dist:
                return "close_inner"
            else:
                return "close_outer"
        else:
            return "close_outer"
    
    def get_boundary_proximity(self, x: float, y: float) -> str:
        # check if current position is within middle boundary
        if x >= self.middle_x_min and x <= self.middle_x_max and y >= self.middle_y_min and y <= self.middle_y_max:
            return "close_inner"
        else:
            return "close_outer"

    def update_plot(self, positions: List[Tuple[float, float]], auto_fit: bool = False):
        """
        Update the plot with new position data.

        Args:
            positions: List of (x, y) position tuples
            auto_fit: If True, automatically compute best-fit boundaries
        """
        if len(positions) > 0:
            xs, ys = zip(*positions)

            # Auto-fit boundaries if requested
            if auto_fit:
                self.compute_best_fit_boundaries(positions)
            else:
                # Dynamic boundary expansion
                if self.x_min is None or min(xs) < self.x_min:
                    self.x_min = min(xs)
                if self.x_max is None or max(xs) > self.x_max:
                    self.x_max = max(xs)
                if self.y_min is None or min(ys) < self.y_min:
                    self.y_min = min(ys)
                if self.y_max is None or max(ys) > self.y_max:
                    self.y_max = max(ys)                            

                self._update_inner_boundary()


            if self.debug:
                self.ax.clear()
                # Plot the path
                self.ax.plot(
                    xs,
                    ys,
                    marker=".",
                    linewidth=1.5,
                    markersize=4,
                    color="blue",
                    label="Path",
                )

                # Draw start zone
                self.ax.add_patch(
                    plt.Rectangle(
                        (0 - self.start_zone_rect_x, 0 - self.start_zone_rect_y),
                        2 * self.start_zone_rect_x,
                        2 * self.start_zone_rect_y,
                        color="green",
                        alpha=0.3,
                        label="Start Zone",
                    )
                )

                # Draw outer boundary (red dashed)
                outer_coords = [
                    [self.x_min - self.padding, self.x_max + self.padding],
                    [self.y_min - self.padding, self.y_min - self.padding],
                ]
                self.ax.plot(*outer_coords, color="red", linestyle="--", linewidth=2)

                outer_coords = [
                    [self.x_min - self.padding, self.x_max + self.padding],
                    [self.y_max + self.padding, self.y_max + self.padding],
                ]
                self.ax.plot(*outer_coords, color="red", linestyle="--", linewidth=2)

                outer_coords = [
                    [self.x_min - self.padding, self.x_min - self.padding],
                    [self.y_min - self.padding, self.y_max + self.padding],
                ]
                self.ax.plot(*outer_coords, color="red", linestyle="--", linewidth=2)

                outer_coords = [
                    [self.x_max + self.padding, self.x_max + self.padding],
                    [self.y_min - self.padding, self.y_max + self.padding],
                ]
                self.ax.plot(
                    *outer_coords,
                    color="red",
                    linestyle="--",
                    linewidth=2,
                    label="Outer Boundary",
                )
                

                # Draw inner boundary (orange solid) if defined
                if all(
                    v is not None
                    for v in [
                        self.inner_x_min,
                        self.inner_x_max,
                        self.inner_y_min,
                        self.inner_y_max,
                    ]
                ):
                    # Bottom
                    self.ax.plot(
                        [self.inner_x_min, self.inner_x_max],
                        [self.inner_y_min, self.inner_y_min],
                        color="orange",
                        linestyle="-",
                        linewidth=2,
                    )
                    # Top
                    self.ax.plot(
                        [self.inner_x_min, self.inner_x_max],
                        [self.inner_y_max, self.inner_y_max],
                        color="orange",
                        linestyle="-",
                        linewidth=2,
                    )
                    # Left
                    self.ax.plot(
                        [self.inner_x_min, self.inner_x_min],
                        [self.inner_y_min, self.inner_y_max],
                        color="orange",
                        linestyle="-",
                        linewidth=2,
                    )
                    # Right
                    self.ax.plot(
                        [self.inner_x_max, self.inner_x_max],
                        [self.inner_y_min, self.inner_y_max],
                        color="orange",
                        linestyle="-",
                        linewidth=2,
                        label="Inner Boundary",
                    )


                # Draw middle boundary (purple solid) if defined
                if all(
                    v is not None
                    for v in [
                        self.middle_x_min,
                        self.middle_x_max,
                        self.middle_y_min,
                        self.middle_y_max,
                    ]
                ):
                    # Bottom
                    self.ax.plot(
                        [self.middle_x_min, self.middle_x_max],
                        [self.middle_y_min, self.middle_y_min],
                        color="gray",
                        linestyle="--",
                        linewidth=1,
                    )
                    # Top
                    self.ax.plot(
                        [self.middle_x_min, self.middle_x_max],
                        [self.middle_y_max, self.middle_y_max],
                        color="gray",
                        linestyle="--",
                        linewidth=1,
                    )
                    # Left
                    self.ax.plot(
                        [self.middle_x_min, self.middle_x_min],
                        [self.middle_y_min, self.middle_y_max],
                        color="gray",
                        linestyle="--",
                        linewidth=1,
                    )
                    # Right
                    self.ax.plot(
                        [self.middle_x_max, self.middle_x_max],
                        [self.middle_y_min, self.middle_y_max],
                        color="gray",
                        linestyle="--",
                        linewidth=1,
                        label="Middle Boundary",
                    )

                self.ax.set_xlabel("X (m)")
                self.ax.set_ylabel("Y (m)")
                self.ax.set_title(
                    f"{self.title}\nInner Margin: {self.inner_margin}m"
                )
                self.ax.axis("equal")
                self.ax.grid(True, alpha=0.3)
                self.ax.legend(loc="upper right", fontsize=8)

                plt.pause(0.01)

    def close(self):
        if self.debug:
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
    from odometry_log import od

    # Sample odometry data (ticks, gyro_angle)
    odometry_data = od
    lap_samples = {}

    # Initialize tracker and visualizer
    tracker = OdometryTracker(wheel_radius=0.046, ticks_per_rev=2220, gear_ratio=1.0, debug=True)
    visualizer = OdometryVisualizer(
        title="Odometry Path (Single Encoder + Gyro)",
        start_zone_rect=[0.55, 2],
        inner_margin=0.75,
        debug=True,
    )
    visualizer.set_dir("cw")  # set direction for boundary lines

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

        count = 0

        for ticks, gyro_angle in odometry_data:
            count += 1  

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

            if count > 81:
                print(f"ticks: {ticks}, gyro: {gyro_angle}) index: {count}")
                print(visualizer.get_boundary_vector_proximity(tracker.positions[-2][0], tracker.positions[-2][1], tracker.x, tracker.y))
                # print(f"heading {visualizer.heading_toward_rectangle(tracker.positions[-2][0], tracker.positions[-2][1], tracker.x, tracker.y)}")
                # time.sleep(1)

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
