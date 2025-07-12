from flask import Flask, request, jsonify, render_template_string
import random

app = Flask(__name__)

@app.route("/")
def index():
    return render_template_string("""
<!DOCTYPE html>
<html>
<head>
    <title>Upload Image and Get Arrow</title>
    <style>
        #arrowCanvas { border:1px solid black; margin-top:20px; }
    </style>
</head>
<body>
    <h2>Upload Image</h2>
    <input type="file" id="fileInput">
    <button onclick="uploadImage()">Upload</button>

    <canvas id="arrowCanvas" width="400" height="400"></canvas>

    <script>
        const canvas = document.getElementById("arrowCanvas");
        const ctx = canvas.getContext("2d");
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;

        function drawArrow(angle) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.beginPath();
            const length = 100;
            const rad = angle * Math.PI / 180;
            const x = centerX + length * Math.cos(rad);
            const y = centerY - length * Math.sin(rad);
            ctx.moveTo(centerX, centerY);
            ctx.lineTo(x, y);
            ctx.strokeStyle = "red";
            ctx.lineWidth = 5;
            ctx.stroke();
        }

        function uploadImage() {
            const fileInput = document.getElementById("fileInput");
            const file = fileInput.files[0];
            if (!file) {
                alert("Please select a file.");
                return;
            }

            const reader = new FileReader();
            reader.onload = function() {
                const base64 = reader.result.split(",")[1];

                fetch("/upload", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ frame: base64 })
                })
                .then(response => response.json())
                .then(data => {
                    if (data.direction !== null) {
                        drawArrow(data.direction);
                    }
                })
                .catch(err => {
                    console.error(err);
                });
            };
            reader.readAsDataURL(file);
        }
    </script>
</body>
</html>
""")

@app.route("/upload", methods=["POST"])
def upload():
    data = request.json
    frame_data = data.get("frame")

    # Here you would process the frame with your model.
    # For this demo, we'll simulate with a random angle.
    simulated_angle = random.randint(0, 359)
    print(f"Received image of size {len(frame_data)} bytes. Returning angle {simulated_angle}°")

    return jsonify({"direction": simulated_angle})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
