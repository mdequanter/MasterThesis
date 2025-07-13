from flask import Flask, render_template_string, request, jsonify
import cv2
import numpy as np
import base64
from ultralytics import YOLO

app = Flask(__name__)

MODEL_PATH = 'unrealsim/models/unrealsim.pt'
model = YOLO(MODEL_PATH)

SCAN_HEIGHTS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]

HTML_PAGE = """
<!DOCTYPE html>
<body>
  <h2>Live Inference</h2>
  <button id="switchButton">Switch Camera</button><br>
  <canvas id="canvas"></canvas>

  <script>
    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');
    const switchButton = document.getElementById('switchButton');

    // Hidden video element (not displayed)
    const video = document.createElement('video');
    video.setAttribute('playsinline', ''); // Important for iOS
    video.style.display = 'none';
    document.body.appendChild(video);

    let currentFacingMode = "user";
    let currentStream = null;

    function startCamera(facingMode) {
      if (currentStream) {
        currentStream.getTracks().forEach(track => track.stop());
      }
      navigator.mediaDevices.getUserMedia({
        video: { facingMode: facingMode }
      }).then(stream => {
        currentStream = stream;
        video.srcObject = stream;
        video.play();
      }).catch(err => {
        console.error("Error accessing camera:", err);
      });
    }

    // Start initial camera
    startCamera(currentFacingMode);

    // Handle switching cameras
    switchButton.addEventListener('click', () => {
      currentFacingMode = currentFacingMode === "user" ? "environment" : "user";
      startCamera(currentFacingMode);
    });

    video.addEventListener('play', () => {
      canvas.width = video.videoWidth;
      canvas.height = video.videoHeight;
      setInterval(() => {
        ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
        const dataURL = canvas.toDataURL('image/jpeg', 0.5);
        fetch('/process_frame', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ image: dataURL })
        })
        .then(response => response.json())
        .then(data => {
          const img = new Image();
          img.src = 'data:image/png;base64,' + data.image;
          img.onload = () => {
            ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
          };
        });
      }, 300);
    });
  </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/process_frame", methods=["POST"])
def process_frame():
    content = request.json
    data_url = content["image"]
    header, encoded = data_url.split(",", 1)
    img_data = base64.b64decode(encoded)

    nparr = np.frombuffer(img_data, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    height, width = frame.shape[:2]
    overlay = frame.copy()
    midpoints = []

    results = model(frame, conf=0.85, verbose=False)

    for result in results:
        if result.masks is not None:
            mask = result.masks.data[0].cpu().numpy()
            mask = (mask * 255).astype(np.uint8)
            mask_resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)

            green_overlay = np.full_like(frame, (0, 255, 0))
            blended = cv2.addWeighted(frame, 0.3, green_overlay, 0.7, 0)
            overlay[mask_resized > 0] = blended[mask_resized > 0]

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

    if midpoints:
        avg_x = int(np.mean([pt[0] for pt in midpoints]))
        target_point = (avg_x, min([pt[1] for pt in midpoints]))
        start_point = (width // 2, height)
        cv2.arrowedLine(overlay, start_point, target_point, (0, 0, 255), 5, tipLength=0.2)

    # Encode processed frame to PNG
    _, buffer = cv2.imencode('.png', overlay)
    encoded_result = base64.b64encode(buffer).decode('utf-8')

    return jsonify({'image': encoded_result})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=9005, debug=True)
