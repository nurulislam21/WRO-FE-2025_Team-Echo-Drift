import serial
import time
import math
from odometry import OdometryTracker, OdometryVisualizer

arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, dsrdtr=True)
time.sleep(2)
arduino.write(b"0,-1,95\n")

instructions = [
    (5, 2220, 95)
]

# init odometry tracker and visualizer
tracker = OdometryTracker(wheel_radius=0.046, ticks_per_rev=2100, gear_ratio=1.0)
visualizer = OdometryVisualizer(title='Odometry Path (Single Encoder + Gyro)')

while True:
    print("Waiting for start...")
    # dont start until start button pressed
    if arduino.in_waiting > 0:
        line = arduino.readline().decode("utf-8").rstrip()
        print(f"Arduino: {line}")
        if line == "START":
            break

for instr in instructions:
    arduino.write(f"{instr[0]},{instr[1]},{instr[2]}\n".encode())
    time.sleep(1)

    if arduino.in_waiting > 0:
        print(arduino.readline().decode("utf-8").rstrip())

ticks = 0
gyro_angle = 0.0
prev_ticks = 0
prev_gyro_angle = 0.0

def clamp_angle(totalAngle, threshold=5):
    """Clamp totalAngle to nearest multiple of 90 if within the threshold."""
    # Find the nearest multiple of 90
    nearest = round(totalAngle / 90) * 90
    
    # If the difference is within the threshold, return the clamped value
    if abs(totalAngle - nearest) <= threshold:
        return nearest
    
    # Otherwise return original
    return totalAngle


while True:
    if arduino.in_waiting > 0:
        line = arduino.readline().decode('utf-8').rstrip()
        parts = line.split(',')
        print(f"Arduino: {line}")
        if len(parts) != 2:
            continue
        try:
            ticks = -int(parts[0])
            gyro_angle = float(parts[1])
            # print(f"Ticks: {ticks}, Gyro Angle: {gyro_angle}")
        except ValueError:
            continue

    now = time.time()
    if not hasattr(arduino, "_last_send_time"):
        arduino._last_send_time = 0.0
        arduino._last_plot_time = 0.0        
    if now - arduino._last_send_time >= 0.3:
        arduino.write(b"0,0,95\n")
        arduino._last_send_time = now
    if (now - arduino._last_plot_time) >= 0.5 and (abs(ticks - prev_ticks) > 100):
        gyro_angle = clamp_angle(gyro_angle, threshold=20)
        # Update odometry tracker         
        tracker.update(ticks, gyro_angle)
        # Get current position
        x, y, theta = tracker.get_position()
        print(f"Position: x={x:.3f}m, y={y:.3f}m, θ={math.degrees(theta):.1f}°")     
        # Update visualization
        visualizer.update_plot(tracker.get_position_history())
        arduino._last_plot_time = now

        # update prev values
        prev_ticks = ticks
        prev_gyro_angle = gyro_angle        
    # # Update odometry tracker         
    # tracker.update(ticks, gyro_angle)
    # # Get current position
    # x, y, theta = tracker.get_position()
    # print(f"Position: x={x:.3f}m, y={y:.3f}m, θ={math.degrees(theta):.1f}°")     
    # # Update visualization
    # visualizer.update_plot(tracker.get_position_history())