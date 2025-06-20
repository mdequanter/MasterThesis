import cv2
import numpy as np
import os
import time  # <-- Nieuw toegevoegd
from ultralytics import YOLO

# --- Model laden ---
model = YOLO('unrealsim\\models\\blindnavUnreal.pt', verbose=True)

# --- Videobestand openen ---
video_path = r'unrealsim\\videos\\UnrealParkRecording.mp4'
cap = cv2.VideoCapture(video_path)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("Fout bij het openen van de video")
    exit()

# --- Video-eigenschappen ophalen ---
fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# --- Outputpad opbouwen ---
dirname, basename = os.path.split(video_path)
name, ext = os.path.splitext(basename)
output_path = os.path.join(dirname, f"{name}_result.mp4")

# --- VideoWriter instellen ---
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Gebruik 'XVID' voor AVI
out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

print(f"Resultaatvideo wordt opgeslagen als: {output_path}")

# --- Voor FPS-berekening ---
prev_time = time.time()  # <-- Nieuw toegevoegd

while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    ret, frame = cap.read()
    if not ret or frame is None:
        print("Frame niet beschikbaar, probeer volgende frame")
        continue

    # --- Start tijd voor inferentie ---
    start_time = time.time()

    # Voer objectdetectie uit
    results = model(frame, conf=0.1, verbose=False)

    # --- Eind tijd voor inferentie ---
    end_time = time.time()

    # Bereken de inferentie tijd
    inference_time = end_time - start_time

    overlay = frame.copy()
    for result in results:
        if result.masks is not None:
            for mask in result.masks.xy:
                points = np.array(mask, dtype=np.int32)
                cv2.fillPoly(overlay, [points], color=(0, 255, 0))
                cv2.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=2)

    alpha = 0.3
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

    # --- FPS berekenen en weergeven ---
    current_time = time.time()
    fps_text = 1 / (current_time - prev_time)
    prev_time = current_time

    # --- Toon FPS en Inference Time op het frame ---
    cv2.putText(frame, f"FPS: {fps_text:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)  # FPS tekst

    inference_time_text = f"Inference Time: {inference_time * 1000:.2f} ms"  # Convert to milliseconds
    cv2.putText(frame, inference_time_text, (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)  # Inference time tekst

    # Toon het frame met FPS en Inference Time
    cv2.imshow("YOLOv8 Segmentatie", frame)
    out.write(frame)  # Frame wegschrijven naar outputbestand

# --- Alles afsluiten ---
cap.release()
out.release()
cv2.destroyAllWindows()

print("Video verwerking voltooid.")
