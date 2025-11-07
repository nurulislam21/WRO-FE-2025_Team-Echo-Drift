import time
import matplotlib.pyplot as plt
import math

# Wheel and encoder parameters
WHEEL_RADIUS = 0.033  # in meters (e.g., 33 mm)
TICKS_PER_REV = 1000  # encoder ticks per revolution
GEAR_RATIO = 1        # adjust if there's a gearbox

# Initialize position and orientation
x, y, theta = 0.0, 0.0, 0.0
positions = [(x, y)]
odometry_data = [
    (12, -0.7),
    (155, 1.5),
    (303, -1.2),
    (462, 0.3),
    (612, -0.9),
    (757, 2.1),
    (907, 1.8),
    (1050, -0.4),
    (1203, -1.8),
    (1358, 0.9),
    (1504, 89.3),
    (1658, 88.2),
    (1807, 90.5),
    (1960, 91.7),
    (2102, 89.4),
    (2261, 92.6),
    (2413, 88.8),
    (2555, 91.1),
    (2710, 89.0),
    (2863, 88.3),
    (3006, 178.1),
    (3152, 181.2),
    (3308, 179.0),
    (3451, 180.7),
    (3610, 179.4),
    (3765, 182.3),
    (3908, 179.1),
    (4060, 181.7),
    (4215, 178.8),
    (4358, 182.0),
    (4503, 272.5),
    (4655, 269.7),
    (4808, 270.8),
    (4962, 268.1),
    (5103, 271.6),
    (5260, 269.2),
    (5412, 273.0),
    (5566, 269.3),
    (5707, 271.1),
    (5859, 269.4),
    (6009, 359.6),
    (6161, 358.3),
    (6304, 0.5),
    (6462, -1.4),
    (6610, 0.9),
    (6761, -0.5),
    (6918, 1.1),
    (7060, -0.8),
    (7214, 0.3),
    (7362, -1.6),
    (7510, 0.4),
    (7653, 89.9),
    (7805, 90.7),
    (7961, 92.1),
    (8102, 88.2),
    (8260, 90.8),
    (8412, 91.6),
    (8565, 88.9),
    (8718, 90.1),
    (8862, 91.3),
    (9014, 89.5),
    (9155, 179.3),
    (9302, 180.6),
    (9457, 181.4),
    (9612, 179.9),
    (9758, 182.1),
    (9914, 180.7),
    (10060, 178.4),
    (10211, 180.5),
    (10358, 181.9),
    (10504, 179.0),
    (10659, 269.8),
    (10812, 270.4),
    (10960, 272.0),
    (11108, 269.5),
    (11252, 270.9),
    (11400, 269.1),
    (11556, 271.4),
    (11704, 272.8),
    (11850, 269.7),
    (12004, 271.3),
    (12150, 2.2),
    (12300, 0.7),
    (12462, -1.5),
    (12599, 1.0),
    (12752, -0.9),
    (12905, 0.8),
    (13050, -1.1),
    (13210, 1.3),
    (13354, -0.6),
    (13500, 0.5),
]

time.sleep(2)  # Wait for Arduino to reset (if needed)

# Function to calculate distance from ticks
def get_distance_from_ticks(ticks):
    return 2 * math.pi * WHEEL_RADIUS * (ticks / TICKS_PER_REV) / GEAR_RATIO

try:
    prev_ticks = 0
    i = 0

    while True:
        # Simulate reading from serial
        if i < len(odometry_data):
            line = f"{odometry_data[i][0]},{odometry_data[i][1]}\n"
            i += 1
        else:
            break
        line = line.strip()
        if not line:
            continue
        ticks_str, gyro_str = line.split(',')
        ticks = int(ticks_str)
        gyro = float(gyro_str)

        # Calculate distance traveled since last reading
        delta_ticks = ticks - prev_ticks
        distance = get_distance_from_ticks(delta_ticks)
        prev_ticks = ticks

        # Update pose
        theta = math.radians(gyro)
        x += distance * math.cos(theta)
        y += distance * math.sin(theta)

        positions.append((x, y))

        # Live plot
        plt.clf()
        xs, ys = zip(*positions)
        plt.plot(xs, ys, marker='.')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Odometry Path (Based on Single Encoder + Gyro)')
        plt.axis('equal')
        plt.pause(0.01)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Odometry stopped by user.")