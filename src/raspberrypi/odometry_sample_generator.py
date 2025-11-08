import random

# Function to add Gaussian noise
def noisy(value, noise_level):
    return value + random.gauss(0, noise_level)

# Simulated noisy odometry data
odometry_data = []

# Track segments: 3m, 2m, 3m, 2m rectangle
distances = [3.0, 2.0, 3.0, 2.0]
ticks_per_meter = 1540  # approx (1m / (2*pi*0.033)) * 1000

tick_sum = 0
angle = 0

for i, dist in enumerate(distances * 2):  # two laps
    segment_ticks = int(dist * ticks_per_meter)

    for step in range(100):
        # Linear progress on segment
        delta = segment_ticks / 100
        tick_sum += delta

        # Noisy ticks and gyro angle
        noisy_ticks = int(noisy(tick_sum, 15))  # 15 tick noise
        noisy_angle = noisy(angle, 2)  # 2 deg gyro noise

        odometry_data.append((noisy_ticks, noisy_angle))

    # At segment end, turn 90 degrees
    angle += 90
    if angle >= 360:
        angle -= 360

# Show first few entries
for i in range(100):
    print(odometry_data[i] + ',')
