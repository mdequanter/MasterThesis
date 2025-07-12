import threading
import time
import json
import asyncio
import websockets
from flask import Flask, request, jsonify, render_template_string
import base64
import cv2
import numpy as np

app = Flask(__name__)

# Global to hold the latest direction
latest_direction = None

# WebSocket inference server
SIGNALING_SERVER = "ws://127.0.0.1:9000"

# Frame queue
frame_queue = asyncio.Queue()

# Background thread flag
running = True


async def inference_loop():
    global latest_direction, running
    async with websockets.connect(SIGNALING_SERVER) as websocket:
        print("✅ Connected to inference server")
        while running:
            # Wait for a frame from the browser
            encoded_frame = await frame_queue.get()

            # Send to inference server
            await websocket.send(json.dumps({
                "frame": encoded_frame
            }))

            # Receive result
            msg = await websocket.recv()
            data = json.loads(msg)
            latest_direction = data.get("direction_angle")

            # Small delay to avoid overloading
            await asyncio.sleep(0.05)


def start_inference_thread():
    def run_loop():
        asyncio.run(inference_loop())
    t = threading.Thread(target=run_loop, daemon=True)
    t.start()


@app.route("/")
def index():
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Webcam Arrow Inference</title>
        <style>
            video, canvas { border:1px solid black; }
        </style>
    </head>
    <body>
        <h2>Webcam Arrow Inference</h2>
        <video id="video" width="320" height="240" autoplay></video>
        <canvas id="arrowCanvas" width="400" height="400"></canvas>
        <script>
            const video = document.getElementById("video");
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

            async function pollDirection() {
                try {
                    const resp = await fetch("/direction");
                    const data = await resp.json();
                    if (data.direction !== null) {
                        drawArrow(data.direction);
                    }
                } catch (e) {
                    console.error(e);
                }
            }

            setInterval(pollDirection, 300);

            // Start webcam
            navigator.mediaDevices.getUserMedia({ video: true })
                .then((stream) => {
                    video.srcObject = stream;
                })
                .catch((err) => {
                    console.error("Error accessing webcam:", err);
                });

            // Capture and send frames periodically
            const hiddenCanvas = document.createElement("canvas");
            hiddenCanvas.width = 320;
            hiddenCanvas.height = 240;
            const hiddenCtx = hiddenCanvas.getContext("2d");

            async function captureAndSend() {
                hiddenCtx.drawImage(video, 0, 0, hiddenCanvas.width, hiddenCanvas.height);
                const dataURL = hiddenCanvas.toDataURL("image/jpeg", 0.6);
                const base64Data = dataURL.split(",")[1];

                try {
                    await fetch("/upload", {
                        method: "POST",
                        headers: { "Content-Type": "application/json" },
                        body: JSON.stringify({ frame: base64Data })
                    });
                } catch (e) {
                    console.error(e);
                }
            }

            setInterval(captureAndSend, 200);
        </script>
    </body>
    </html>
    """)


@app.route("/upload", methods=["POST"])
def upload():
    data = request.json
    frame_data = data["frame"]
    # Put into async queue
    asyncio.run(frame_queue.put(frame_data))
    return jsonify({"status": "ok"})


@app.route("/direction")
def get_direction():
    return jsonify({"direction": latest_direction})


if __name__ == "__main__":
    start_inference_thread()
    app.run(host="0.0.0.0", port=5000)
