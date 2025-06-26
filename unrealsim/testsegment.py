import cv2
import numpy as np
from ultralytics import YOLO
import sys

# 🔧 Parameters
MODEL_PATH = "unrealsim/models/blindnavUnreal.pt"
IMAGE_PATH = "images/test.png"
CONFIDENCE = 0.85
SCAN_HEIGHTS = [0.1,0.2,0.3,0.4, 0.5, 0.6 , 0.7]  # procentuele hoogte van scanlijnen

# ✅ Command-line parsing
for arg in sys.argv[1:]:
    if arg.startswith("MODEL="):
        MODEL_PATH = arg.split("=")[1]
    elif arg.startswith("IMAGE="):
        IMAGE_PATH = arg.split("=")[1]
    elif arg.startswith("CONFIDENCE="):
        CONFIDENCE = float(arg.split("=")[1])

# 📥 Laad beeld
image = cv2.imread(IMAGE_PATH)
if image is None:
    print("❌ Kan afbeelding niet laden.")
    sys.exit(1)

height, width = image.shape[:2]

# 📦 Laad model
model = YOLO(MODEL_PATH)

# 🔍 Run segmentatie
results = model(image, conf=CONFIDENCE, verbose=False)

# 🧠 Extract mask
overlay = image.copy()
midpoints = []

for result in results:
    if result.masks is not None:
        mask = result.masks.data[0].cpu().numpy()
        mask = (mask * 255).astype(np.uint8)

        # Teken overlay
        overlay[mask > 0] = cv2.addWeighted(image, 0.3, np.full_like(image, (0, 255, 0)), 0.7, 0)[mask > 0]

        # 🎯 Zoek middens op scanlijnen
        for r in SCAN_HEIGHTS:
            y = int(height * r)
            scan_row = mask[y, :]
            indices = np.where(scan_row > 0)[0]
            if len(indices) > 0:
                midpoint_x = int(np.mean(indices))
                midpoints.append((midpoint_x, y))
                cv2.circle(overlay, (midpoint_x, y), 5, (255, 0, 0), -1)
            cv2.line(overlay, (0, y), (width, y), (150, 150, 150), 1)

# ➡️ Teken richting
if midpoints:
    avg_x = int(np.mean([pt[0] for pt in midpoints]))
    target_point = (avg_x, min(pt[1] for pt in midpoints))
    start_point = (width // 2, height)
    cv2.arrowedLine(overlay, start_point, target_point, (0, 0, 255), 5, tipLength=0.2)

    dx = avg_x - start_point[0]
    dy = start_point[1] - target_point[1]
    angle_rad = np.arctan2(dy, dx)
    angle_deg = np.degrees(angle_rad)
    cv2.putText(overlay, f"Direction: {angle_deg:.2f} deg", (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    print(f"📐 Richtingshoek: {angle_deg:.2f}°")
else:
    print("⚠️ Geen segmentatie op de scanlijnen gevonden.")

# 🖼️ Toon resultaat
cv2.imshow("Segmentatie met richtingspijl", overlay)
cv2.waitKey(0)
cv2.destroyAllWindows()
