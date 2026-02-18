import cv2
import json
import socket
import threading
import csv
import time
import math
from datetime import datetime
import requests
from ultralytics import YOLO

# ================= YOLO MODEL =================
model = YOLO(r"C:\Users\TG68\OneDrive\Desktop\road_defect_system\edge_device\best.pt")   # keep best.pt in same folder

# ================= CAMERA =====================
cap = cv2.VideoCapture(0)

# ================= GPS CONFIG =================
GPS_IP = "192.168.2.103"
GPS_PORT = 8080

latitude = None
longitude = None
date_str = None
time_str = None

# ================= CLOUD API ==================
API_URL = "https://road-defect-system-1.onrender.com/upload"
BASE_URL = "https://road-defect-system-1.onrender.com"

# ================= CSV ========================
CSV_FILE = "laptop_test_gps.csv"

with open(CSV_FILE, "a", newline="") as f:
    writer = csv.writer(f)
    if f.tell() == 0:
        writer.writerow(["date","time","latitude","longitude","class","confidence"])

# ================= FILTER SETTINGS =================
DISTANCE_THRESHOLD = 8      # meters
TIME_COOLDOWN = 10          # seconds

last_saved = {}

# ================= DISTANCE FUNCTION =================
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# ================= CLOUD FUNCTION =================
def send_to_cloud(data):
    try:
        # wake up render (first request slow)
        requests.get(BASE_URL, timeout=20)

        response = requests.post(API_URL, json=data, timeout=40)

        print("📡 CLOUD STATUS:", response.status_code)
        print("📡 RESPONSE:", response.text)

    except Exception as e:
        print("❌ CLOUD UPLOAD FAILED:", e)

# ================= GPS THREAD =================
def gps_loop():
    global latitude, longitude, date_str, time_str

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((GPS_IP, GPS_PORT))
        print("📡 Connected to Mobile GPS")

        buffer = ""

        while True:
            data = sock.recv(1024).decode(errors="ignore")
            buffer += data

            while "{" in buffer and "}" in buffer:
                start = buffer.find("{")
                end = buffer.find("}") + 1
                chunk = buffer[start:end]
                buffer = buffer[end:]

                try:
                    gps = json.loads(chunk)

                    latitude = gps.get("latitude")
                    longitude = gps.get("longitude")
                    ts_ms = gps.get("timestamp")

                    if latitude and longitude and ts_ms:
                        ts = datetime.fromtimestamp(ts_ms / 1000)
                        date_str = ts.strftime("%Y-%m-%d")
                        time_str = ts.strftime("%H:%M:%S")

                except:
                    pass

    except Exception as e:
        print("❌ GPS Error:", e)

# ================= DUPLICATE CHECK =================
def is_new_defect(cls_name, lat, lon):
    global last_saved

    now = time.time()

    if cls_name not in last_saved:
        last_saved[cls_name] = {"lat": lat, "lon": lon, "time": now}
        return True

    prev = last_saved[cls_name]

    dist = haversine(lat, lon, prev["lat"], prev["lon"])
    time_diff = now - prev["time"]

    if dist > DISTANCE_THRESHOLD and time_diff > TIME_COOLDOWN:
        last_saved[cls_name] = {"lat": lat, "lon": lon, "time": now}
        return True

    return False

# ================= MAIN LOOP =================
def main():
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, conf=0.4, verbose=False)
        annotated = results[0].plot()

        if results[0].boxes is not None and latitude and longitude:

            for box in results[0].boxes:

                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf = float(box.conf[0])

                if is_new_defect(cls_name, latitude, longitude):

                    print(f"✅ NEW DEFECT → {cls_name}")

                    # SAVE CSV
                    with open(CSV_FILE, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            date_str,
                            time_str,
                            round(latitude, 6),
                            round(longitude, 6),
                            cls_name,
                            round(conf, 2)
                        ])

                    # SEND TO CLOUD
                    data = {
                        "defect_type": cls_name,
                        "latitude": latitude,
                        "longitude": longitude,
                        "confidence": conf,
                        "date": date_str,
                        "time": time_str
                    }

                    send_to_cloud(data)

        cv2.imshow("YOLO + GPS + CLOUD", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

# ================= RUN =================
if __name__ == "__main__":
    threading.Thread(target=gps_loop, daemon=True).start()
    main()