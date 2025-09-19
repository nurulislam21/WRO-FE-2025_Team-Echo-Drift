import cv2
from serial import Serial
from contour_workers import ContourResult

class Parking:
    def __init__(self, arduino: Serial):
        self.arduino = arduino

    def process_parking(self, parking_result: ContourResult):
        parking_walls = []

        if parking_result and parking_result.area > 500:
            parking_wall_count = 0

            for contour in parking_result.contours:
                x, y, w, h = cv2.boundingRect(contour)
                parking_walls.append((x, y, w, h))
        
        return parking_walls
