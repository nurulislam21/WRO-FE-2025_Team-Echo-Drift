# - canny
# - color classyfier


"""
Real-time Canny edge detection (OpenCV)

Usage examples:
  - Use webcam (default):
      python canny_realtime.py
  - Use a video file:
      python canny_realtime.py --video path/to/video.mp4
  - Save output to file:
      python canny_realtime.py --video input.mp4 --out out.mp4

Features:
  - Real-time display of original & edge frames
  - Adjustable low/high thresholds via trackbars
  - Toggle Gaussian blur and dilation
  - FPS overlay and keyboard controls
    q: quit
    s: toggle show edges only / side-by-side
    g: toggle gaussian blur
    d: toggle dilation
    p: pause / resume
"""

import cv2
import argparse
import time


def nothing(x):
    pass


def make_trackbar_window(win_name):
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    # Initial values
    cv2.createTrackbar('Low', win_name, 50, 500, nothing)
    cv2.createTrackbar('High', win_name, 150, 500, nothing)
    cv2.createTrackbar('Aperture', win_name, 3, 7, nothing)  # must be odd: 3,5,7
    cv2.createTrackbar('L2grad', win_name, 0, 1, nothing)


def get_trackbar_values(win_name):
    low = cv2.getTrackbarPos('Low', win_name)
    high = cv2.getTrackbarPos('High', win_name)
    aperture = cv2.getTrackbarPos('Aperture', win_name)
    if aperture % 2 == 0:
        aperture += 1
        if aperture > 7:
            aperture = 7
    l2grad = bool(cv2.getTrackbarPos('L2grad', win_name))
    return low, high, aperture, l2grad


def main():
    parser = argparse.ArgumentParser(description='Realtime Canny edge detection')
    parser.add_argument('--video', '-v', help='Path to video file. If omitted, webcam is used.', default=None)
    parser.add_argument('--out', '-o', help='Optional path to save processed output (MP4).', default=None)
    parser.add_argument('--width', '-w', type=int, help='Resize width (preserve aspect ratio).', default=None)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.video if args.video else 0)
    if not cap.isOpened():
        print('Error: cannot open video source')
        return

    # Prepare writer if needed
    writer = None
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    win = 'Canny Controls'
    make_trackbar_window(win)

    show_edges_only = False
    use_blur = True
    use_dilate = False
    paused = False

    prev_time = time.time()
    fps = 0.0

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                # If video file ended, break; if webcam lost frame, continue
                if args.video:
                    print('End of video file reached')
                    break
                else:
                    time.sleep(0.01)
                    continue

            # Optional resize
            if args.width:
                h, w = frame.shape[:2]
                if w != args.width:
                    scale = args.width / float(w)
                    frame = cv2.resize(frame, (args.width, int(h * scale)), interpolation=cv2.INTER_AREA)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            low, high, aperture, l2grad = get_trackbar_values(win)
            # Apply optional blur
            if use_blur:
                # kernel size chosen based on aperture value
                k = 5 if aperture < 5 else 7
                gray_proc = cv2.GaussianBlur(gray, (k, k), 0)
            else:
                gray_proc = gray

            # Canny needs thresholds; if low==high we offset
            if low >= high:
                high = low + 1

            edges = cv2.Canny(gray_proc, low, high, apertureSize=aperture, L2gradient=l2grad)

            if use_dilate:
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                edges = cv2.dilate(edges, kernel, iterations=1)

            # Create colour edges for overlay
            edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

            # Overlay edges in red on original
            overlay = frame.copy()
            overlay[edges != 0] = (0, 0, 255)
            blended = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)

            # Compose final display
            if show_edges_only:
                display = edges_color
            else:
                # side-by-side: original | edges | overlay
                h, w = frame.shape[:2]
                small = cv2.resize(edges_color, (w // 3, h))
                orig_small = cv2.resize(frame, (w // 3, h))
                overlay_small = cv2.resize(blended, (w // 3, h))
                display = cv2.hconcat([orig_small, small, overlay_small])

            # FPS calculation
            curr_time = time.time()
            dt = curr_time - prev_time
            prev_time = curr_time
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else (1.0 / dt)

            # Put FPS text
            cv2.putText(display, f'FPS: {fps:.1f}', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, f'Blur: {"On" if use_blur else "Off"} | Dilate: {"On" if use_dilate else "Off"} | Mode: {"Edges" if show_edges_only else "Side-by-side"}',
                        (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow('Canny Realtime', display)

            # Initialize writer after knowing frame size
            if args.out and writer is None:
                h_out, w_out = display.shape[:2]
                writer = cv2.VideoWriter(args.out, fourcc, 20.0, (w_out, h_out))

            if writer is not None:
                # Writer expects BGR frames with 3 channels
                writer.write(display)

        # Keyboard controls (waitKey delayed so trackbars are responsive)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            show_edges_only = not show_edges_only
        elif key == ord('g'):
            use_blur = not use_blur
        elif key == ord('d'):
            use_dilate = not use_dilate
        elif key == ord('p'):
            paused = not paused

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
