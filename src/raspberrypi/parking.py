import cv2
from serial import Serial
from contour_workers import ContourResult
from simple_pid import PID

class Parking:
    def __init__(self, arduino: Serial):
        self.arduino = arduino

    def process_parking(self, parking_result: ContourResult, pid: PID):
        parking_walls = []
        parking_wall_count = 0

        if parking_result and parking_result.area > 800:
            for contour in parking_result.contours:
                x, y, w, h = cv2.boundingRect(contour)
                area = w * h                

                if area > 1000:
                    parking_wall_count += 1
                    parking_walls.append((x, y, w, h))
        
        return parking_walls, parking_wall_count
