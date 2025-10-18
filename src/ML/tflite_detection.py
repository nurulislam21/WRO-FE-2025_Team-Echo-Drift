import cv2
import numpy as np
import time
from pathlib import Path
from collections import deque
import multiprocessing as mp
from threading import Thread
from queue import Queue, Empty
import os

# Try to import LiteRT (new), fallback to TFLite (old)
try:
    from ai_edge_litert.interpreter import Interpreter
    print("Using LiteRT (ai_edge_litert)")
except ImportError:
    try:
        import tensorflow as tf
        Interpreter = tf.lite.Interpreter
        print("Using TensorFlow Lite (deprecated)")
    except ImportError:
        raise ImportError("Neither ai_edge_litert nor tensorflow.lite is available")

# Set OpenCV optimization flags
cv2.setUseOptimized(True)
cv2.setNumThreads(mp.cpu_count())

# Set CPU affinity for maximum performance (Linux only)
try:
    os.sched_setaffinity(0, range(mp.cpu_count()))
except:
    pass


class VideoStreamThread:
    """Threaded video capture for non-blocking frame reading"""
    def __init__(self, src, queue_size=2):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.stopped = False
        self.queue = Queue(maxsize=queue_size)
        
    def start(self):
        Thread(target=self.update, daemon=True).start()
        return self
    
    def update(self):
        while not self.stopped:
            if not self.queue.full():
                ret, frame = self.stream.read()
                if not ret:
                    self.stopped = True
                    return
                
                # Drop old frames if queue is full
                if self.queue.full():
                    try:
                        self.queue.get_nowait()
                    except Empty:
                        pass
                
                self.queue.put(frame)
    
    def read(self):
        return self.queue.get()
    
    def more(self):
        return not self.stopped or not self.queue.empty()
    
    def stop(self):
        self.stopped = True
        self.stream.release()
    
    def get(self, prop):
        return self.stream.get(prop)


class YOLOv11TFLiteDetectorUltraFast:
    def __init__(self, model_path, conf_threshold=0.25, num_threads=None, 
                 nms_threshold=0.45, max_det=300):
        """
        Ultra-optimized YOLOv11 TFLite detector
        
        Args:
            model_path: Path to .tflite model
            conf_threshold: Confidence threshold
            num_threads: CPU threads (None = all cores)
            nms_threshold: NMS IoU threshold (lower = faster, fewer boxes)
            max_det: Maximum detections to keep (lower = faster)
        """
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        self.max_det = max_det
        
        if num_threads is None:
            num_threads = mp.cpu_count()
        
        model_path = Path(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        print(f"Loading model: {model_path.name} ({model_path.stat().st_size / 1024 / 1024:.2f} MB)")
        
        # Load with XNNPack delegate for ARM/x86 optimization
        try:
            self.interpreter = Interpreter(
                model_path=str(model_path),
                num_threads=num_threads,
                experimental_delegates=None  # XNNPack is default
            )
        except:
            self.interpreter = Interpreter(
                model_path=str(model_path),
                num_threads=num_threads
            )
        
        self.interpreter.allocate_tensors()
        
        # Cache everything
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_index = self.input_details[0]['index']
        self.output_index = self.output_details[0]['index']
        self.input_shape = self.input_details[0]['shape']
        self.img_height = self.input_shape[1]
        self.img_width = self.input_shape[2]
        
        # Pre-allocate arrays
        self.input_array = np.zeros(self.input_shape, dtype=np.float32)
        self.resize_buffer = np.zeros((self.img_height, self.img_width, 3), dtype=np.uint8)
        
        # Pre-generate colors
        np.random.seed(42)
        self._colors = [(int(c[0]), int(c[1]), int(c[2])) 
                       for c in np.random.randint(0, 255, size=(100, 3))]
        
        print(f"Loaded: {self.input_shape} | Threads: {num_threads} | Max Det: {max_det}")
    
    def preprocess_turbo(self, frame):
        """Ultra-fast preprocessing with minimal copies"""
        orig_h, orig_w = frame.shape[:2]
        
        # Resize directly to buffer
        cv2.resize(frame, (self.img_width, self.img_height), 
                  dst=self.resize_buffer, interpolation=cv2.INTER_LINEAR)
        
        # Convert and normalize in one shot
        cv2.cvtColor(self.resize_buffer, cv2.COLOR_BGR2RGB, self.resize_buffer)
        np.multiply(self.resize_buffer, 1.0/255.0, out=self.input_array[0], 
                   casting='unsafe', dtype=np.float32)
        
        return orig_w, orig_h
    
    def nms_fast(self, boxes, scores, iou_threshold=0.45):
        """Fast NMS using cv2.dnn.NMSBoxes"""
        if len(boxes) == 0:
            return []
        
        # Convert to x, y, w, h format
        boxes_xywh = np.column_stack([
            boxes[:, 0],
            boxes[:, 1],
            boxes[:, 2] - boxes[:, 0],
            boxes[:, 3] - boxes[:, 1]
        ])
        
        # Use OpenCV's optimized NMS
        indices = cv2.dnn.NMSBoxes(
            boxes_xywh.tolist(),
            scores.tolist(),
            score_threshold=0.0,  # Already filtered
            nms_threshold=iou_threshold,
            top_k=self.max_det
        )
        
        return indices.flatten() if len(indices) > 0 else []
    
    def detect_turbo(self, frame):
        """Ultra-fast detection with aggressive optimization"""
        orig_w, orig_h = self.preprocess_turbo(frame)
        
        # Inference
        self.interpreter.set_tensor(self.input_index, self.input_array)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_index)[0]
        
        # Fast confidence filtering
        conf_mask = output[:, 4] >= self.conf_threshold
        filtered = output[conf_mask]
        
        if len(filtered) == 0:
            return []
        
        # Limit max detections early for speed
        if len(filtered) > self.max_det * 2:
            top_indices = np.argpartition(filtered[:, 4], -self.max_det * 2)[-self.max_det * 2:]
            filtered = filtered[top_indices]
        
        # Vectorized coordinate scaling
        coords = filtered[:, :4]
        coords[:, [0, 2]] *= orig_w
        coords[:, [1, 3]] *= orig_h
        
        # Clip and validate (vectorized)
        coords = np.clip(coords, 0, [orig_w, orig_h, orig_w, orig_h])
        valid = (coords[:, 2] > coords[:, 0] + 1) & (coords[:, 3] > coords[:, 1] + 1)
        
        coords = coords[valid].astype(np.int32)
        scores = filtered[valid, 4]
        classes = filtered[valid, 5].astype(np.int32)
        
        # Apply NMS
        if len(coords) > 1:
            keep_indices = self.nms_fast(coords, scores, self.nms_threshold)
            if len(keep_indices) > 0:
                coords = coords[keep_indices]
                scores = scores[keep_indices]
                classes = classes[keep_indices]
        
        # Build detections (keep top N only)
        detections = []
        for i in range(min(len(coords), self.max_det)):
            detections.append([
                coords[i, 0], coords[i, 1], coords[i, 2], coords[i, 3],
                float(scores[i]), int(classes[i])
            ])
        
        return detections
    
    def draw_turbo(self, frame, detections, class_names=None, show_labels=True):
        """Ultra-fast drawing with minimal overhead"""
        if not detections or not show_labels:
            # Just boxes, no text (fastest)
            for det in detections:
                x1, y1, x2, y2, _, cls = det
                cv2.rectangle(frame, (x1, y1), (x2, y2), 
                            self._colors[cls % len(self._colors)], 2)
            return
        
        # With labels (still optimized)
        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            color = self._colors[cls % len(self._colors)]
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Minimal label
            label = f"{class_names[cls] if class_names and cls < len(class_names) else cls}:{conf:.1f}"
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            cv2.rectangle(frame, (x1, y1 - lh - 6), (x1 + lw, y1), color, -1)
            cv2.putText(frame, label, (x1, y1 - 3),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)


class FPSCounter:
    def __init__(self, window_size=30):
        self.times = deque(maxlen=window_size)
        self.last = time.perf_counter()
    
    def update(self):
        now = time.perf_counter()
        self.times.append(now - self.last)
        self.last = now
        return len(self.times) / sum(self.times) if self.times else 0


def process_video_turbo(model_path, video_path, conf_threshold=0.25, 
                        class_names=None, num_threads=None, roi=None,
                        skip_frames=0, display_scale=0.5, show_labels=True,
                        nms_threshold=0.45, max_det=100):
    """
    Maximum performance video processing with all optimizations
    
    Args:
        model_path: Path to TFLite model
        video_path: Path to video
        conf_threshold: Confidence threshold (higher = faster)
        class_names: Class names list
        num_threads: CPU threads (None = all)
        roi: Region of interest (x1, y1, x2, y2) or None
        skip_frames: Process every N+1 frames (0=all, 1=half, 2=third)
        display_scale: Display resize scale (0.5 = half size, faster)
        show_labels: Show class labels (False = boxes only, faster)
        nms_threshold: NMS IoU threshold (lower = faster, fewer overlaps)
        max_det: Max detections (lower = faster)
    """
    # Initialize detector
    detector = YOLOv11TFLiteDetectorUltraFast(
        model_path, conf_threshold, num_threads, nms_threshold, max_det
    )
    
    # Threaded video capture
    print("Starting threaded video capture...")
    vs = VideoStreamThread(str(video_path), queue_size=2).start()
    time.sleep(1.0)  # Warm up
    
    # Video properties
    width = int(vs.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(vs.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(vs.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # Calculate display size
    display_width = int(width * display_scale)
    display_height = int(height * display_scale)
    
    print(f"\n{'='*60}")
    print(f"VIDEO: {width}x{height} | DISPLAY: {display_width}x{display_height}")
    print(f"SKIP: {skip_frames} | ROI: {'Yes' if roi else 'No'} | LABELS: {show_labels}")
    print(f"CONF: {conf_threshold} | NMS: {nms_threshold} | MAX_DET: {max_det}")
    print(f"{'='*60}\n")
    print("Controls: 'q'=quit | 'p'=pause | 's'=toggle skip | 'l'=toggle labels")
    
    fps_counter = FPSCounter(window_size=30)
    frame_count = 0
    processed_count = 0
    paused = False
    last_detections = []
    
    # Pre-compute ROI slice if provided
    roi_slice = None
    roi_offset = (0, 0)
    if roi:
        x1, y1, x2, y2 = roi
        roi_slice = (slice(y1, y2), slice(x1, x2))
        roi_offset = (x1, y1)
    
    # Create display window
    cv2.namedWindow('YOLO', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('YOLO', display_width, display_height)
    
    try:
        while vs.more():
            if not paused:
                frame = vs.read()
                
                # Frame skip logic
                process_this_frame = (skip_frames == 0) or (frame_count % (skip_frames + 1) == 0)
                
                if process_this_frame:
                    # ROI processing
                    if roi_slice:
                        roi_frame = frame[roi_slice]
                        detections = detector.detect_turbo(roi_frame)
                        
                        # Adjust coordinates
                        for det in detections:
                            det[0] += roi_offset[0]
                            det[1] += roi_offset[1]
                            det[2] += roi_offset[0]
                            det[3] += roi_offset[1]
                        
                        # Draw ROI box
                        cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), 
                                    (0, 255, 255), 2)
                    else:
                        detections = detector.detect_turbo(frame)
                    
                    last_detections = detections
                    processed_count += 1
                else:
                    detections = last_detections
                
                # Draw detections
                detector.draw_turbo(frame, detections, class_names, show_labels)
                
                # FPS calculation
                fps = fps_counter.update()
                
                # Minimal overlay
                cv2.putText(frame, f"{fps:.0f}", (10, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                cv2.putText(frame, f"{len(detections)}", (10, 85),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                frame_count += 1
            
            # Display (resize for performance)
            if display_scale != 1.0:
                display_frame = cv2.resize(frame, (display_width, display_height),
                                          interpolation=cv2.INTER_NEAREST)
            else:
                display_frame = frame
            
            cv2.imshow('YOLO', display_frame)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('p'):
                paused = not paused
            elif key == ord('s'):
                skip_frames = (skip_frames + 1) % 3
                print(f"Skip mode: {skip_frames}")
            elif key == ord('l'):
                show_labels = not show_labels
                print(f"Labels: {show_labels}")
    
    finally:
        vs.stop()
        cv2.destroyAllWindows()
        
        print(f"\n{'='*60}")
        print(f"COMPLETE: {processed_count}/{frame_count} frames | Avg FPS: {fps:.1f}")
        print(f"{'='*60}")


if __name__ == "__main__":
    # ==================== CONFIGURATION ====================
    MODEL_PATH = "best_saved_model/best_int8.tflite"
    # MODEL_PATH = "best_saved_model/best_float16.tflite"
    # MODEL_PATH = "best_saved_model/best_integer_quant.tflite"
    VIDEO_PATH = "video.mp4"
    
    CLASS_NAMES = ["class_0", "class_1", "class_2"]
    
    # ROI (Region of Interest) - set to None to disable
    ROI = (200, 70, 700, 250)  # (x1, y1, x2, y2) or None
    
    # ==================== SPEED TUNING ====================
    # Priority 1: Confidence (higher = faster, fewer false positives)
    CONF_THRESHOLD = 0.6  # Try 0.5-0.7
    
    # Priority 2: Frame skipping (0=all, 1=half, 2=third)
    SKIP_FRAMES = 0  # Start with 0, increase if needed
    
    # Priority 3: Display scale (0.5 = 4x faster rendering)
    DISPLAY_SCALE = 1  # 0.5 = half size, 1.0 = full size
    
    # Priority 4: Max detections (lower = faster)
    MAX_DETECTIONS = 10  # Try 30-100
    
    # Priority 5: NMS threshold (lower = faster, fewer boxes)
    NMS_THRESHOLD = 0.4  # Try 0.3-0.5
    
    # Priority 6: Show labels (False = boxes only, ~10% faster)
    SHOW_LABELS = True
    
    # Priority 7: CPU threads (None = all cores)
    NUM_THREADS = None
    
    # =====================================================
    
    try:
        process_video_turbo(
            model_path=MODEL_PATH,
            video_path=VIDEO_PATH,
            conf_threshold=CONF_THRESHOLD,
            class_names=CLASS_NAMES,
            num_threads=NUM_THREADS,
            roi=ROI,
            skip_frames=SKIP_FRAMES,
            display_scale=DISPLAY_SCALE,
            show_labels=SHOW_LABELS,
            nms_threshold=NMS_THRESHOLD,
            max_det=MAX_DETECTIONS
        )
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()