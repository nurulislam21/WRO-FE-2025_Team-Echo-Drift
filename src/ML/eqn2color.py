import cv2
import numpy as np

# --- Your fitted function (OpenCV Lab units) ---
def fitted_b(a):    
    return (-0.0541 * a ** 2) + (19.5786 * a) + -1611.0335
    

def make_and_show_lab_image(width=1024, height=300, L_value=128):
    """
    Generate an OpenCV Lab image (constant L) where b = fitted_b(a),
    with horizontal 'a' value labels drawn below.
    """

    # --- 1. Generate range of a values ---
    a_vals = np.linspace(0, 255, width, dtype=np.float32)
    A = np.tile(a_vals, (height, 1))

    # --- 2. Compute corresponding b values ---
    B = fitted_b(A)
    B = np.clip(B, 0, 255).astype(np.uint8)

    # --- 3. Constant L across the whole image ---
    L = np.full_like(A, L_value, dtype=np.uint8)

    # --- 4. Combine into Lab image ---
    lab_img = np.dstack((L, A.astype(np.uint8), B))

    # --- 5. Convert to BGR for visualization ---
    bgr_img = cv2.cvtColor(lab_img, cv2.COLOR_Lab2BGR)

    # --- 6. Create a label bar below the image ---
    label_height = 60
    bar = np.full((label_height, width, 3), 255, np.uint8)

    # --- 7. Draw tick marks and a values ---
    step = 50  # spacing between ticks
    for x in range(0, width, step):
        a_val = int((x / width) * 255)
        cv2.line(bar, (x, 0), (x, 10), (0, 0, 0), 2)
        cv2.putText(bar, str(a_val), (x - 10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    # --- 8. Stack the main image and label bar vertically ---
    final_img = np.vstack((bgr_img, bar))

    # --- 9. Show using OpenCV ---
    cv2.imshow(f"b = fitted_b(a) | L={L_value}", final_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    make_and_show_lab_image(width=1024, height=300, L_value=128)