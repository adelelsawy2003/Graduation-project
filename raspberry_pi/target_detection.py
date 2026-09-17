# ==============================================================================
# AUTONOMOUS TARGET DETECTION AND LASER POINTING SYSTEM - CENTRAL INTEGRATION
# Hardware: Raspberry Pi 4, IMX219 Camera, Arduino (Serial)
# Frameworks: NCNN, OpenCV, Picamera2
# ==============================================================================

import ncnn
import numpy as np
import cv2
import time
import serial
import argparse
from picamera2 import Picamera2


def verify_pink_color(crop_img):
    '''
    Secondary Deterministic Verification:
    Converts the cropped bounding box to HSV color space and checks if the
    ratio of pink pixels exceeds the required safety threshold (15%).
    Eliminates False-Positives reliably.
    '''
    if crop_img.size == 0:
        return False

    # Convert BGR to HSV for illumination-invariant color detection
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV if crop_img.shape[2] == 3 else cv2.COLOR_RGB2HSV)

    # Strict HSV bounds for the Target Pink Box
    lower_pink = np.array([140, 50, 50])
    upper_pink = np.array([175, 255, 255])

    # Generate binary mask and count non-zero pixels
    mask = cv2.inRange(hsv, lower_pink, upper_pink)
    pink_pixels = cv2.countNonZero(mask)
    total_pixels = crop_img.shape[0] * crop_img.shape[1]

    # Return True only if the box is genuinely pink
    return (pink_pixels / total_pixels) > 0.15


def detect_pink_box(img, net, target_size=512, conf_threshold=0.5, nms_threshold=0.4):
    '''
    Core Inference Pipeline:
    Processes the raw image through the NCNN YOLO11n network, decodes the raw
    (5, 5376) tensor, applies Non-Maximum Suppression (NMS), and calculates
    the absolute geometric center of the target.
    '''
    h_orig, w_orig = img.shape[:2]

    # Resize and normalize image for neural network consumption
    mat_in = ncnn.Mat.from_pixels_resize(img, ncnn.Mat.PixelType.PIXEL_BGR2RGB, w_orig, h_orig, target_size, target_size)
    mean_vals = []
    norm_vals = [1 / 255.0, 1 / 255.0, 1 / 255.0]
    mat_in.substract_mean_normalize(mean_vals, norm_vals)

    # Execute NCNN Forward Propagation
    ex = net.create_extractor()
    ex.input("in0", mat_in)
    ret, mat_out = ex.extract("out0")
    if ret != 0:
        return []

    # Decode the massive candidate tensor
    out_array = np.array(mat_out)
    preds = out_array.T
    boxes_cxcywh = preds[:, :4]
    scores = preds[:, 4]

    # Apply Confidence Threshold Mask
    mask = scores > conf_threshold
    filtered_boxes = boxes_cxcywh[mask]
    filtered_scores = scores[mask]

    if len(filtered_scores) == 0:
        return []

    # Convert center coordinates to Top-Left representation for OpenCV NMS
    boxes_nms = []
    for box in filtered_boxes:
        cx, cy, w, h = box
        boxes_nms.append([int(cx - w / 2.0), int(cy - h / 2.0), int(w), int(h)])

    # Apply Intersection over Union (IoU) NMS filtering
    indices = cv2.dnn.NMSBoxes(boxes_nms, filtered_scores.tolist(), conf_threshold, nms_threshold)

    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            x_min, y_min, bw, bh = boxes_nms[i]
            score = filtered_scores[i]

            # Scale mathematical bounding box back to original hardware frame resolution
            x_scale = w_orig / target_size
            y_scale = h_orig / target_size
            final_w = int(bw * x_scale)
            final_h = int(bh * y_scale)
            final_x = int(x_min * x_scale)
            final_y = int(y_min * y_scale)

            # Boundary protections
            final_x = max(0, final_x)
            final_y = max(0, final_y)

            # Trigger Secondary HSV Protocol
            crop = img[final_y:final_y + final_h, final_x:final_x + final_w]
            if not verify_pink_color(crop):
                continue

            # Calculate final physical center coordinate
            center_x = final_x + (final_w // 2)
            center_y = final_y + (final_h // 2)
            results.append({"center": (center_x, center_y), "confidence": score})

    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--conf', type=float, default=0.75, help='Deep Learning Confidence Threshold')
    args = parser.parse_args()

    # -----------------------------------------------------------------
    # Hardware Initialization Phase
    # -----------------------------------------------------------------
    print("[INFO] Initializing UART Serial Communication to Arduino Kinetic Controller...")
    try:
        arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=0.1)
        time.sleep(2)
        print("[INFO] Arduino Actuators Connected Successfully! 🤖")
    except Exception as e:
        print("[WARNING] Arduino Hardware not detected. Proceeding in AI Evaluation Mode.")
        arduino = None

    print("[INFO] Loading compiled NCNN weights into physical memory...")
    net = ncnn.Net()
    net.load_param("model.ncnn.param")
    net.load_model("model.ncnn.bin")

    print("[INFO] Waking up IMX219 Optical Sensor...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"}))
    picam2.start()

    # State Machine Variables
    detection_counter = 0
    REQUIRED_FRAMES = 3  # Temporal Smoothing to prevent jitter
    target_was_visible = False

    # -----------------------------------------------------------------
    # Main Closed-Loop Execution System
    # -----------------------------------------------------------------
    try:
        while True:
            start_time = time.time()
            img = picam2.capture_array()

            # Calculate Optical Frame Center (Physical Laser Resting Point)
            h_frame, w_frame = img.shape[:2]
            screen_center_x = w_frame // 2
            screen_center_y = h_frame // 2

            detections = detect_pink_box(img, net, conf_threshold=args.conf)
            fps = 1.0 / (time.time() - start_time)

            if len(detections) > 0:
                detection_counter += 1

                # Enforce temporal stability logic
                if detection_counter >= REQUIRED_FRAMES:
                    det = detections[0]
                    cx, cy = det['center']
                    conf = det['confidence']

                    # Calculate Proportional Error Vectors (Visual Servoing)
                    error_x = cx - screen_center_x
                    error_y = cy - screen_center_y

                    print(f"[FPS: {fps:.1f}] 🎯 TARGET LOCKED -> Spatial Error X: {error_x}, Y: {error_y}")

                    # Transmit correction vectors to hardware muscles
                    if arduino is not None:
                        arduino.write(f"{error_x},{error_y}\n".encode('utf-8'))

                    target_was_visible = True
                else:
                    print(f"[FPS: {fps:.1f}] ⏳ Filtering temporal noise... ({detection_counter}/{REQUIRED_FRAMES})")
            else:
                detection_counter = 0
                print(f"[FPS: {fps:.1f}] ⏳ Scanning environment for target payload...")

                # Failsafe Mechanical Halt Protocol
                if target_was_visible:
                    if arduino is not None:
                        arduino.write("STOP\n".encode('utf-8'))
                        print("[CRITICAL] Target lost. Issued Emergency STOP to actuators.")
                    target_was_visible = False

    except KeyboardInterrupt:
        print("\n[INFO] Gracefully shutting down optic sensors...")
        picam2.stop()
        if arduino is not None:
            arduino.write("STOP\n".encode('utf-8'))
            arduino.close()
        print("[INFO] Operations Terminated.")
