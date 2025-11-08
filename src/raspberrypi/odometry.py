import time
import matplotlib.pyplot as plt
import math
from typing import List, Tuple


class OdometryTracker:
    """
    A class to track robot position and orientation using encoder and gyroscope data.
    """
    
    def __init__(self, wheel_radius: float = 0.033, ticks_per_rev: int = 1000, 
                 gear_ratio: float = 1.0):
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
        return 2 * math.pi * self.wheel_radius * (ticks / self.ticks_per_rev) / self.gear_ratio
    
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
    
    def get_position_history(self) -> List[Tuple[float, float]]:
        """
        Get complete position history.
        
        Returns:
            List of (x, y) tuples
        """
        return self.positions.copy()


class OdometryVisualizer:
    """
    A class to visualize odometry data in real-time.
    """
    
    def __init__(self, title: str = "Odometry Path"):
        """
        Initialize the visualizer.
        
        Args:
            title: Plot title
        """
        self.title = title
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        
    def update_plot(self, positions: List[Tuple[float, float]]):
        """
        Update the plot with new position data.
        
        Args:
            positions: List of (x, y) position tuples
        """
        self.ax.clear()
        
        if len(positions) > 0:
            xs, ys = zip(*positions)
            self.ax.plot(xs, ys, marker='.', linewidth=1.5, markersize=4)
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title(self.title)
        self.ax.axis('equal')
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
        (12, -0.7), (155, 1.5), (303, -1.2), (462, 0.3), (612, -0.9),
        (757, 2.1), (907, 1.8), (1050, -0.4), (1203, -1.8), (1358, 0.9),
        (1504, 89.3), (1658, 88.2), (1807, 90.5), (1960, 91.7), (2102, 89.4),
        (2261, 92.6), (2413, 88.8), (2555, 91.1), (2710, 89.0), (2863, 88.3),
        (3006, 178.1), (3152, 181.2), (3308, 179.0), (3451, 180.7), (3610, 179.4),
        (3765, 182.3), (3908, 179.1), (4060, 181.7), (4215, 178.8), (4358, 182.0),
        (4503, 272.5), (4655, 269.7), (4808, 270.8), (4962, 268.1), (5103, 271.6),
        (5260, 269.2), (5412, 273.0), (5566, 269.3), (5707, 271.1), (5859, 269.4),
        (6009, 359.6), (6161, 358.3), (6304, 0.5), (6462, -1.4), (6610, 0.9),
        (6761, -0.5), (6918, 1.1), (7060, -0.8), (7214, 0.3), (7362, -1.6),
        (7510, 0.4), (7653, 89.9), (7805, 90.7), (7961, 92.1), (8102, 88.2),
        (8260, 90.8), (8412, 91.6), (8565, 88.9), (8718, 90.1), (8862, 91.3),
        (9014, 89.5), (9155, 179.3), (9302, 180.6), (9457, 181.4), (9612, 179.9),
        (9758, 182.1), (9914, 180.7), (10060, 178.4), (10211, 180.5), (10358, 181.9),
        (10504, 179.0), (10659, 269.8), (10812, 270.4), (10960, 272.0), (11108, 269.5),
        (11252, 270.9), (11400, 269.1), (11556, 271.4), (11704, 272.8), (11850, 269.7),
        (12004, 271.3), (12150, 2.2), (12300, 0.7), (12462, -1.5), (12599, 1.0),
        (12752, -0.9), (12905, 0.8), (13050, -1.1), (13210, 1.3), (13354, -0.6),
        (13500, 0.5),
    ]
    
    # Initialize tracker and visualizer
    tracker = OdometryTracker(wheel_radius=0.033, ticks_per_rev=1000, gear_ratio=1.0)
    visualizer = OdometryVisualizer(title='Odometry Path (Single Encoder + Gyro)')
    
    print("Starting odometry simulation...")
    time.sleep(1)
    
    try:
        for ticks, gyro_angle in odometry_data:
            # Update tracker with new data
            tracker.update(ticks, gyro_angle)
            
            # Get current position
            x, y, theta = tracker.get_position()
            print(f"Position: x={x:.3f}m, y={y:.3f}m, θ={math.degrees(theta):.1f}°")
            
            # Update visualization
            visualizer.update_plot(tracker.get_position_history())
            
            time.sleep(0.5)
        
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