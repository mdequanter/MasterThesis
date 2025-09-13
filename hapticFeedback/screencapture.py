# pip install mss opencv-python numpy
import cv2, numpy as np, mss

# Optioneel: kies een regio (x, y, width, height)
region = {'top': 100, 'left': 100, 'width': 100, 'height': 100}

with mss.mss() as sct:
    # Wil je full-screen: region = sct.monitors[1]  # [1] is primaire monitor
    while True:
        frame = np.array(sct.grab(region))         # BGRA
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        # Doe hier je OpenCV bewerkingen, b.v. edge detectie:
        edges = cv2.Canny(frame, 100, 200)

        cv2.imshow("Screen", frame)
        cv2.imshow("Edges", edges)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

cv2.destroyAllWindows()
