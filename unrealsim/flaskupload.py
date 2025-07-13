from flask import Flask, render_template, request, send_file
import cv2
import numpy as np
import io
from ultralytics import YOLO
from PIL import Image

app = Flask(__name__)

# Laad het YOLO-model
MODEL_PATH = 'unrealsim/models/unrealsim.pt'   # Pas aan indien nodig
model = YOLO(MODEL_PATH, verbose=True)

# Scanhoogtes zoals in je originele code
SCAN_HEIGHTS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]

@app.route("/", methods=["GET", "POST"])
def upload_predict():
    if request.method == "POST":
        if "file" not in request.files:
            return "No file part"
        file = request.files["file"]
        if file.filename == "":
            return "No selected file"
        if file:
            # Lees afbeelding in numpy array
            in_memory_file = io.BytesIO()
            file.save(in_memory_file)
            data = np.frombuffer(in_memory_file.getvalue(), dtype=np.uint8)
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

            # Run inference
            DETECTION_CONFIDENCE = 0.85
            results = model(frame, conf=DETECTION_CONFIDENCE, verbose=False)

            # Overlay zoals in je script
            overlay = frame.copy()
            height, width = frame.shape[:2]
            midpoints = []

            for result in results:
                if result.masks is not None:
                    mask = result.masks.data[0].cpu().numpy()
                    mask = (mask * 255).astype(np.uint8)
                    mask_resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)

                    green_overlay = np.full_like(frame, (0, 255, 0))
                    blended = cv2.addWeighted(frame, 0.3, green_overlay, 0.7, 0)
                    overlay[mask_resized > 0] = blended[mask_resized > 0]

                    # Scanhoogtes en middenpunten
                    for r in SCAN_HEIGHTS:
                        y = int(height * r)
                        if y >= height:
                            continue
                        scan_row = mask_resized[y, :]
                        indices = np.where(scan_row > 0)[0]
                        if len(indices) > 0:
                            midpoint_x = int(np.mean(indices))
                            midpoints.append((midpoint_x, y))
                            cv2.circle(overlay, (midpoint_x, y), 5, (255, 0, 0), -1)
                        cv2.line(overlay, (0, y), (width, y), (150, 150, 150), 1)

            # Richtingspijl
            if midpoints:
                avg_x = int(np.mean([pt[0] for pt in midpoints]))
                target_point = (avg_x, min([pt[1] for pt in midpoints]))
                start_point = (width // 2, height)
                cv2.arrowedLine(overlay, start_point, target_point, (0, 0, 255), 5, tipLength=0.2)

            # Converteer BGR naar RGB
            overlay_rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(overlay_rgb)

            # Opslaan in buffer om terug te geven
            img_io = io.BytesIO()
            pil_img.save(img_io, 'PNG')
            img_io.seek(0)
            return send_file(img_io, mimetype='image/png')

    return render_template("upload.html")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
