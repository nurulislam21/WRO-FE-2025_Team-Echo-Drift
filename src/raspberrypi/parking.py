from serial import Serial


class Parking:
    def __init__(self, arduino: Serial):
        self.arduino = arduino

    def process_parking(self, frame_result):
        ...
