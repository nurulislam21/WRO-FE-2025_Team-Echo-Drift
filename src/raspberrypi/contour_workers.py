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
        lower_red: np.ndarray,
        upper_red: np.ndarray,
        lower_green: np.ndarray,
        upper_green: np.ndarray,
        roi1: list,
        roi2: list,
        roi3: list,
        roi4: list,
    ):
        # colors
        self.LOWER_BLUE = lower_blue
        self.UPPER_BLUE = upper_blue
        self.LOWER_BLACK = lower_black
        self.UPPER_BLACK = upper_black
        self.LOWER_ORANGE = lower_orange
        self.UPPER_ORANGE = upper_orange
        self.LOWER_RED = lower_red
        self.UPPER_RED = upper_red
        self.LOWER_GREEN = lower_green
        self.UPPER_GREEN = upper_green
        # regions of interest
        self.ROI1 = roi1
        self.ROI2 = roi2
        self.ROI3 = roi3
        self.ROI4 = roi4
        # queues
        self.frame_queue_left = Queue(maxsize=2)
        self.frame_queue_right = Queue(maxsize=2)
        self.frame_queue_orange = Queue(maxsize=2)
        self.frame_queue_blue = Queue(maxsize=2)
        self.frame_queue_green = Queue(maxsize=2)
        self.frame_queue_red = Queue(maxsize=2)
        # result queues
        self.result_queue_left = Queue(maxsize=2)
        self.result_queue_right = Queue(maxsize=2)
        self.result_queue_orange = Queue(maxsize=2)
        self.result_queue_blue = Queue(maxsize=2)
        self.result_queue_green = Queue(maxsize=2)
        self.result_queue_red = Queue(maxsize=2)
        # control event
        self.stop_processing = threading.Event()

        # results
        self.left_result = ContourResult()
        self.right_result = ContourResult()
        self.orange_result = ContourResult()
        self.blue_result = ContourResult()
        self.green_result = ContourResult()
        self.red_result = ContourResult()
    
    def put_frames_in_queues(self, frame_copy):
        try:
            self.frame_queue_left.put_nowait(frame_copy)
        except:
            pass  # Skip if queue full

        try:
            self.frame_queue_right.put_nowait(frame_copy)
        except:
            pass

        try:
            self.frame_queue_orange.put_nowait(frame_copy)
        except:
            pass

        try:
            self.frame_queue_blue.put_nowait(frame_copy)
        except:
            pass

        try:
            self.frame_queue_green.put_nowait(frame_copy)
        except:
            pass

        try:
            self.frame_queue_red.put_nowait(frame_copy)
        except:
            pass
    
    def collect_results(self):
        try:
            while not self.result_queue_left.empty():
                self.left_result = self.result_queue_left.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_right.empty():
                self.right_result = self.result_queue_right.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_orange.empty():
                self.orange_result = self.result_queue_orange.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_blue.empty():
                self.blue_result = self.result_queue_blue.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_green.empty():
                self.green_result = self.result_queue_green.get_nowait()
        except Empty:
            pass

        try:
            while not self.result_queue_red.empty():
                self.red_result = self.result_queue_red.get_nowait()
        except Empty:
            pass

        return (
            self.left_result,
            self.right_result,
            self.orange_result,
            self.blue_result,
            self.green_result,
            self.red_result,
        )

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

    def green_contour_worker(self):
        """Worker thread for green marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_green.get(timeout=0.1)
                contours = find_contours(
                    frame, self.LOWER_GREEN, self.UPPER_GREEN, self.ROI4
                )
                area, _ = max_contour(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_green.put_nowait(result)
                except:
                    try:
                        self.result_queue_green.get_nowait()
                        self.result_queue_green.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_green.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Green processing error: {e}")
                continue

    def red_contour_worker(self):
        """Worker thread for red marker detection"""
        while not self.stop_processing.is_set():
            try:
                frame = self.frame_queue_red.get(timeout=0.1)
                contours = find_contours(
                    frame, self.LOWER_RED, self.UPPER_RED, self.ROI4
                )
                area, _ = max_contour(contours)
                result = ContourResult(area, contours)

                try:
                    self.result_queue_red.put_nowait(result)
                except:
                    try:
                        self.result_queue_red.get_nowait()
                        self.result_queue_red.put_nowait(result)
                    except Empty:
                        pass

                self.frame_queue_red.task_done()
            except Empty:
                continue
            except Exception as e:
                print(f"Red processing error: {e}")
                continue
