import cv2
import serial
import time
import numpy as np
from picamera2 import Picamera2

# ---------------- SERIAL ----------------
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
time.sleep(2)                 # Arduino reset
ser.write(b"HELLO\n")         # handshake
time.sleep(1)

# ---------------- TIMING ----------------
start_time = time.time()

DESCEND_TIME = 10      # seconds
FORWARD_TIME = 15      # seconds
MISSION_END_TIME = 45  # pitch + stop time
PUMP_DURATION = 15     # seconds

descend_sent = False
forward_sent = False
pitch_done = False

ball_sent = False
pump_start_time = 0

# ---------------- CAMERA SETUP ----------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "BGR888"}
)
picam2.configure(config)
picam2.start()

print("Camera started...")

# ---------------- MAIN LOOP ----------------
while True:

    frame = picam2.capture_array()

    # -------- BALL DETECTION --------
    ball_detected = False

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 1.5)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=100,
        param2=30,
        minRadius=15,
        maxRadius=200
    )

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for (x, y, r) in circles[0, :]:
            ball_detected = True
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.putText(
                frame,
                "BALL DETECTED",
                (x - 40, y - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

    cv2.imshow("Ball Detection", frame)

    # -------- TIME BASED MISSION (ALWAYS RUNS) --------
    now = time.time()
    elapsed = now - start_time

    # DESCEND
    if elapsed > DESCEND_TIME and not descend_sent:
        ser.write(b"DESCEND\n")
        print("DESCEND")
        descend_sent = True

    # FORWARD
    if elapsed > FORWARD_TIME and not forward_sent:
        ser.write(b"FORWARD\n")
        print("FORWARD")
        forward_sent = True

    # -------- BALL → PUMP (OPTIONAL) --------
    if ball_detected and not ball_sent:
        ser.write(b"BALL\n")
        print("BALL DETECTED → PUMP ON")
        ball_sent = True
        pump_start_time = now

    # STOP PUMP after 15s (only if started)
    if ball_sent and (now - pump_start_time) > PUMP_DURATION:
        ser.write(b"STOP_PUMP\n")
        print("STOP PUMP")
        ball_sent = False   # prevent repeat

    # -------- PITCH & STOP (ALWAYS HAPPENS) --------
    if elapsed > MISSION_END_TIME and not pitch_done:
        ser.write(b"PITCH_UP\n")
        print("PITCH UP")
        time.sleep(2)

        ser.write(b"PITCH_DOWN\n")
        print("PITCH DOWN")
        time.sleep(2)

        ser.write(b"STOP\n")
        print("MISSION COMPLETE")

        pitch_done = True
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ---------------- CLEANUP ----------------
cv2.destroyAllWindows()
ser.close()

