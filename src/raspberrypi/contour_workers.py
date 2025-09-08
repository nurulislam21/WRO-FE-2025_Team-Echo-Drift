from img_processing_functions import find_contours, max_contour
import threading
from queue import Queue, Empty
import cv2
import numpy as np


class ContourResult:
    def __init__(self, area=0, contours=None):
        self.area = area
        self.contours = contours or []


class ContourWorkers:
    def __init__(
        self,
        lower_blue: np.ndarray,
        upper_blue: np.ndarray,
        lower_black: np.ndarray,
        upper_black: np.ndarray,
        lower_orange: np.ndarray,
        upper_orange: np.ndarray,
        roi1: list,
        roi2: list,
        roi3: list,
    ):
        # colors
        self.LOWER_BLUE = lower_blue
        self.UPPER_BLUE = upper_blue
        self.LOWER_BLACK = lower_black
        self.UPPER_BLACK = upper_black
        self.LOWER_ORANGE = lower_orange
        self.UPPER_ORANGE = upper_orange
        # regions of interest
        self.ROI1 = roi1
        self.ROI2 = roi2
        self.ROI3 = roi3
        # queues
        self.frame_queue_left = Queue(maxsize=2)
        self.frame_queue_right = Queue(maxsize=2)
        self.frame_queue_orange = Queue(maxsize=2)
        self.frame_queue_blue = Queue(maxsize=2)
        # result queues
        self.result_queue_left = Queue(maxsize=2)
        self.result_queue_right = Queue(maxsize=2)
        self.result_queue_orange = Queue(maxsize=2)
        self.result_queue_blue = Queue(maxsize=2)

        self.stop_processing = threading.Event()

    def blue_contour_worker(self):
        """Worker thread for blue marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_blue.get(timeout=0.1)
                contours = find_contours(
                    frame, self.LOWER_BLUE, self.UPPER_BLUE, self.ROI3
                )
                area, _ = max_contour(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_blue.put_nowait(result)
                except:
                    try:
                        self.result_queue_blue.get_nowait()
                        self.result_queue_blue.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_blue.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Blue processing error: {e}")
                continue

    def left_contour_worker(self):
        """Worker thread for left ROI black line detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_left.get(timeout=0.1)
                contours = find_contours(
                    frame, self.LOWER_BLACK, self.UPPER_BLACK, self.ROI1
                )
                area, _ = max_contour(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_left.put_nowait(result)
                except:
                    try:
                        self.result_queue_left.get_nowait()
                        self.result_queue_left.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_left.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Left processing error: {e}")
                continue

    def right_contour_worker(self):
        """Worker thread for right ROI black line detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_right.get(timeout=0.1)
                contours = find_contours(
                    frame, self.LOWER_BLACK, self.UPPER_BLACK, self.ROI2
                )
                area, _ = max_contour(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_right.put_nowait(result)
                except:
                    try:
                        self.result_queue_right.get_nowait()
                        self.result_queue_right.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_right.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Right processing error: {e}")
                continue

    def orange_contour_worker(self):
        """Worker thread for orange marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_orange.get(timeout=0.1)
                contours = find_contours(
                    frame, self.LOWER_ORANGE, self.UPPER_ORANGE, self.ROI3
                )
                area, _ = max_contour(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_orange.put_nowait(result)
                except:
                    try:
                        self.result_queue_orange.get_nowait()
                        self.result_queue_orange.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_orange.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Orange processing error: {e}")
                continue

    def blue_contour_worker(self):
        """Worker thread for blue marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_blue.get(timeout=0.1)
                contours = find_contours(
                    frame, self.LOWER_BLUE, self.UPPER_BLUE, self.ROI3
                )
                area, _ = max_contour(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_blue.put_nowait(result)
                except:
                    try:
                        self.result_queue_blue.get_nowait()
                        self.result_queue_blue.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_blue.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Blue processing error: {e}")
                continue
