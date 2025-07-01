from picamera2 import Picamera2
import cv2

# Initialiseer camera
picam2 = Picamera2()

# Start de preview configuratie (voor RGB output)
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()

while True:
    # Haal frame op als numpy array
    frame = picam2.capture_array()
    
    # Toon met OpenCV
    cv2.imshow("Raspicam Live", frame)
    
    # Stoppen met 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
