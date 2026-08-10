import cv2
import numpy as np
import subprocess
import time
import os

ESP32_CAM_URL = os.getenv('ESP32_CAM_URL', "http://http://YOUR_ESP32_CAM_IP//stream")

# --- Tuning for low-end CPU / 4GB RAM (i5 laptop) ---
# Reduce CPU load by running inference less often.
# If inference gets slow, the code auto-increases the skip for smoother UI.
INITIAL_INFER_SKIP = 3  # 1 = best smoothness & accuracy, higher = faster
MIN_INFER_SKIP = 1
MAX_INFER_SKIP = 6
TARGET_INFER_MS = 130  # aim for ~130ms per YOLO forward pass

# 416 = default accuracy; 320 = faster (recommended for i5/4GB)
BLOB_W = BLOB_H = 320

# Display size (reduce to make UI lighter if needed)
DISPLAY_W, DISPLAY_H = 640, 480

# Detection tuning
CONF_THRESH = 0.35
NMS_CONF = 0.35
NMS_IOU = 0.4
MAX_DETECTIONS = 5  # limit boxes drawn/speech candidates per frame

# If not running inference this frame, optionally discard a couple frames
# to keep things "live" (helps MJPEG streams on slow inference).
SKIP_GRABS = 1  # set 0 to disable

# Load COCO classes
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Load YOLOv4-tiny (CPU; OpenCL/GPU rarely helps on integrated graphics)
net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")
# Try OpenVINO first (sometimes faster), but fall back to plain OpenCV DNN
# if the OpenVINO plugin isn't actually available at runtime.
backend_set = False
if hasattr(cv2.dnn, "DNN_BACKEND_INFERENCE_ENGINE"):
    try:
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_INFERENCE_ENGINE)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        backend_set = True
    except cv2.error:
        backend_set = False

if not backend_set:
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Limit OpenCV thread oversubscription (helps on some i5 laptops)
try:
    cv2.setNumThreads(2)
except Exception:
    pass

layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

candidate_urls = [ESP32_CAM_URL]
# Many ESP32-CAM streams are on port :81 (for the webserver), so try it as a fallback.
try:
    if ESP32_CAM_URL.startswith("http://") and "/stream" in ESP32_CAM_URL:
        host = ESP32_CAM_URL[len("http://") :].split("/stream", 1)[0]
        if ":" not in host:  # only if no port is already specified
            candidate_urls.append(f"http://{host}:81/stream")
except Exception:
    pass

candidate_urls = list(dict.fromkeys(candidate_urls))  # preserve order, remove duplicates

cap = None
for url in candidate_urls:
    print(f"Connecting to camera URL: {url}", flush=True)
    # Prefer FFMPEG backend for network streams.
    try:
        cap_try = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    except Exception:
        cap_try = cv2.VideoCapture(url)

    # Drop backlog so you see recent frames (helps HTTP/MJPEG lag on slow inference)
    try:
        cap_try.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    # Reduce how long OpenCV waits during connect/read (supported in some builds).
    for prop, value in [
        ("CAP_PROP_OPEN_TIMEOUT_MSEC", 4000),
        ("CAP_PROP_READ_TIMEOUT_MSEC", 4000),
    ]:
        if hasattr(cv2, prop):
            try:
                cap_try.set(getattr(cv2, prop), value)
            except Exception:
                pass

    if cap_try.isOpened():
        cap = cap_try
        break
    try:
        cap_try.release()
    except Exception:
        pass

if cap is None or not cap.isOpened():
    print("Camera not opened (all candidate URLs failed).", flush=True)
    print("Make sure the URL works in your browser/VLC, and that you're on the right port/path.", flush=True)
    raise SystemExit(1)

# Global speech timer
last_speech_time = 0
SPEAK_COOLDOWN = 2.0  # global seconds between speeches

def speak(text):
    global speech_proc

    # If another speech process is still running, don't start a new one.
    if speech_proc is not None:
        try:
            if speech_proc.poll() is None:
                # Fallback: if PowerShell hangs for more than 5 seconds, kill it
                # so the speech system remains responsive.
                if getattr(speech_proc, '_start_time', time.time()) < time.time() - 5.0:
                    try: speech_proc.kill()
                    except: pass
                    speech_proc = None
                else:
                    return False
            else:
                speech_proc = None
        except Exception:
            speech_proc = None

    # Start a new speech process.
    # We pipe stdout/stderr to DEVNULL and properly Dispose the synthesizer
    # to prevent PowerShell from permanently hanging in the background.
    speech_proc = subprocess.Popen([
        "powershell",
        "-NoProfile",
        "-WindowStyle",
        "Hidden",
        "-Command",
        f"Add-Type -AssemblyName System.Speech; "
        f"$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
        f"$s.Speak('{text}'); "
        f"$s.Dispose()"
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    speech_proc._start_time = time.time()
    return True


print("Running. Press Q to quit.")

# Each item: (label, x, y, bw, bh) in display coordinates
last_detections = []
speech_proc = None
infer_skip = int(INITIAL_INFER_SKIP)
frame_to_wait = 0
infer_ms_ema = None

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Release finished speech process
    if speech_proc is not None and speech_proc.poll() is not None:
        speech_proc = None

    frame = cv2.resize(frame, (DISPLAY_W, DISPLAY_H))
    h, w = frame.shape[:2]

    do_infer = frame_to_wait <= 0
    if do_infer:
        start = time.perf_counter()
        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (BLOB_W, BLOB_H), swapRB=True, crop=False
        )
        net.setInput(blob)
        outs = net.forward(output_layers)
        infer_ms = (time.perf_counter() - start) * 1000.0

        boxes, confidences, class_ids = [], [], []

        for out in outs:
            for det in out:
                scores = det[5:]
                cid = int(np.argmax(scores))
                conf = float(scores[cid])
                if conf > CONF_THRESH:
                    cx = int(det[0] * w)
                    cy = int(det[1] * h)
                    bw = int(det[2] * w)
                    bh = int(det[3] * h)
                    x = int(cx - bw / 2)
                    y = int(cy - bh / 2)

                    boxes.append([x, y, bw, bh])
                    confidences.append(conf)
                    class_ids.append(cid)

        # Update auto-skip (smooth UI if YOLO gets slow)
        if infer_ms_ema is None:
            infer_ms_ema = infer_ms
        else:
            infer_ms_ema = 0.8 * infer_ms_ema + 0.2 * infer_ms

        if infer_ms_ema > TARGET_INFER_MS and infer_skip < MAX_INFER_SKIP:
            infer_skip += 1
        elif infer_ms_ema < TARGET_INFER_MS * 0.6 and infer_skip > MIN_INFER_SKIP:
            infer_skip -= 1

        frame_to_wait = infer_skip - 1

        last_detections = []
        if boxes:
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, NMS_CONF, NMS_IOU)
        else:
            indexes = []

        if len(indexes) > 0:
            idx = np.array(indexes).flatten().tolist()
            idx.sort(key=lambda i: confidences[int(i)], reverse=True)
            idx = idx[:MAX_DETECTIONS]
            for i in idx:
                ii = int(i)
                label = classes[class_ids[ii]]
                x, y, bw, bh = boxes[ii]
                last_detections.append((label, x, y, bw, bh))
    else:
        # Count down until next inference
        frame_to_wait -= 1
        # Optionally discard a few frames to stay "live"
        if SKIP_GRABS > 0:
            try:
                for _ in range(SKIP_GRABS):
                    cap.grab()
            except Exception:
                pass

    now = time.time()
    
    for label, x, y, bw, bh in last_detections:
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 1)
        cv2.putText(
            frame,
            label,
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        if now - last_speech_time >= SPEAK_COOLDOWN:
            if speak(label):
                last_speech_time = now

    cv2.imshow("ESP32 YOLO PC SPEAK (REAL)", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
