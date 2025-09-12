# app.py
#!/usr/bin/env python3
import os
import argparse
from flask import Flask, request, redirect, url_for, render_template_string, flash
import requests

# ============ Config ============
DEFAULT_BASE_URL = "http://192.168.0.61:8000"
MAX_CONTENT_LENGTH = 8 * 1024 * 1024  # 8 MB uploadlimiet

HTML_INDEX = """
<!doctype html>
<html lang="nl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>CompreFace – Wie is het?</title>
  <style>
    body { font-family: system-ui, Arial, sans-serif; margin: 24px; }
    .card { max-width: 520px; margin: 0 auto; border: 1px solid #ddd; border-radius: 12px; padding: 20px; }
    h1 { margin-top: 0; }
    .hint { color: #666; font-size: .95rem; }
    .row { margin: 10px 0; }
    input[type=file] { width: 100%; }
    button { padding: 10px 14px; border-radius: 8px; border: 0; background: #111827; color: white; font-weight: 600; }
    .result { border-top: 1px solid #eee; margin-top: 18px; padding-top: 14px; }
    .faces { border-collapse: collapse; width: 100%; }
    .faces th, .faces td { border: 1px solid #eee; padding: 8px; text-align: left; }
    .ok { color: #065f46; font-weight: 700; }
    .warn { color: #92400e; font-weight: 700; }
    .err { color: #991b1b; }
  </style>
</head>
<body>
  <div class="card">
    <h1>Wie is het? (CompreFace)</h1>
    <p class="hint">Maak of kies een foto. Op smartphone opent de camera automatisch.</p>

    {% with messages = get_flashed_messages() %}
      {% if messages %}
        <div class="err">
          {% for m in messages %}<p>{{ m|safe }}</p>{% endfor %}
        </div>
      {% endif %}
    {% endwith %}

    <form class="row" action="{{ url_for('recognize') }}" method="post" enctype="multipart/form-data">
      <input type="file" name="file" accept="image/*" capture="environment" required>
      <div class="row">
        <label for="threshold">Drempel (min. similarity %) </label>
        <input type="number" id="threshold" name="threshold" value="{{ threshold|int }}" min="0" max="100" step="1" style="width:90px">
      </div>
      <button type="submit">Analyseer</button>
    </form>

    {% if result %}
      <div class="result">
        <h3>Resultaat</h3>
        {% if best %}
          <p>Beste gok: <span class="{{ 'ok' if best.similarity*100>=threshold else 'warn' }}">
            {{ best.subject }} ({{ (best.similarity*100)|round(1) }}%)
          </span></p>
        {% else %}
          <p class="warn">Geen match boven drempel of geen gezichten gevonden.</p>
        {% endif %}

        {% if faces and faces|length > 0 %}
          <h4>Gevonden gezichten</h4>
          <table class="faces">
            <thead><tr><th>#</th><th>Subject</th><th>Similarity (%)</th></tr></thead>
            <tbody>
            {% for f in faces %}
              <tr>
                <td>{{ loop.index }}</td>
                <td>{{ f.subject or 'Unknown' }}</td>
                <td>{{ (f.similarity*100)|round(2) }}</td>
              </tr>
            {% endfor %}
            </tbody>
          </table>
        {% endif %}
      </div>
    {% endif %}

    <p class="hint">API host: {{ base_url }} — prediction_count={{ prediction_count }} — plugins={{ face_plugins }}</p>
  </div>
</body>
</html>
"""

app = Flask(__name__)
app.secret_key = os.environ.get("FLASK_SECRET_KEY", "dev-secret-not-for-production")
app.config["MAX_CONTENT_LENGTH"] = MAX_CONTENT_LENGTH

# Worden gezet in main()
APP_STATE = {
    "API_KEY": None,
    "BASE_URL": DEFAULT_BASE_URL,
    "ENDPOINT": None,
    "FACE_PLUGINS": "none",       # bv. "age,gender"
    "PREDICTION_COUNT": 1,
    "DEFAULT_THRESHOLD": 60,      # %
    "TIMEOUT_S": 8,
}

def call_compreface(jpeg_bytes: bytes):
    """
    Stuur een image naar CompreFace /recognition/recognize en geef JSON terug.
    """
    headers = {"x-api-key": APP_STATE["API_KEY"]}
    params = {
        "prediction_count": APP_STATE["PREDICTION_COUNT"],
        "det_prob_threshold": 0.6,
        "face_plugins": APP_STATE["FACE_PLUGINS"],
        "status": "false",
        "detect_faces": "true",
    }
    files = {"file": ("upload.jpg", jpeg_bytes, "image/jpeg")}
    r = requests.post(APP_STATE["ENDPOINT"], headers=headers, params=params, files=files, timeout=APP_STATE["TIMEOUT_S"])
    r.raise_for_status()
    return r.json()

@app.route("/", methods=["GET"])
def index():
    return render_template_string(
        HTML_INDEX,
        result=None,
        best=None,
        faces=None,
        threshold=APP_STATE["DEFAULT_THRESHOLD"],
        base_url=APP_STATE["BASE_URL"],
        prediction_count=APP_STATE["PREDICTION_COUNT"],
        face_plugins=APP_STATE["FACE_PLUGINS"],
    )

@app.route("/recognize", methods=["POST"])
def recognize():
    file = request.files.get("file")
    try:
        threshold = float(request.form.get("threshold", APP_STATE["DEFAULT_THRESHOLD"]))
    except ValueError:
        threshold = APP_STATE["DEFAULT_THRESHOLD"]

    if not file:
        flash("Geen bestand ontvangen.")
        return redirect(url_for("index"))

    # We sturen bytes ongewijzigd door; CompreFace accepteert diverse formaten.
    img_bytes = file.read()
    if not img_bytes:
        flash("Leeg bestand.")
        return redirect(url_for("index"))

    try:
        data = call_compreface(img_bytes)
    except requests.RequestException as e:
        flash(f"Fout bij CompreFace: {e}")
        return redirect(url_for("index"))

    # Parseer resultaten: neem per face de beste subject (indien aanwezig)
    faces = []
    for item in (data.get("result") or []):
        subjects = item.get("subjects") or []
        if subjects:
            best = subjects[0]  # hoogste similarity eerst
            faces.append({
                "subject": best.get("subject"),
                "similarity": float(best.get("similarity", 0.0)),
            })
        else:
            faces.append({"subject": "Unknown", "similarity": 0.0})

    # Kies totaal beste
    best = None
    if faces:
        best = max(faces, key=lambda x: x["similarity"])

    # Render
    # Zet kleine objects zodat Jinja makkelijk kan lezen
    class FaceObj: pass
    faces_obj = []
    for f in faces:
        o = FaceObj()
        o.subject = f["subject"]
        o.similarity = f["similarity"]
        faces_obj.append(o)
    best_obj = None
    if best:
        best_obj = FaceObj()
        best_obj.subject = best["subject"]
        best_obj.similarity = best["similarity"]

    return render_template_string(
        HTML_INDEX,
        result=True,
        best=best_obj,
        faces=faces_obj,
        threshold=threshold,
        base_url=APP_STATE["BASE_URL"],
        prediction_count=APP_STATE["PREDICTION_COUNT"],
        face_plugins=APP_STATE["FACE_PLUGINS"],
    )

def parse_args():
    p = argparse.ArgumentParser(description="Flask UI voor CompreFace 'Wie is het?'")
    p.add_argument("--api-key", default=os.getenv("CFX_API_KEY"), help="CompreFace Recognition service API key (of via env CFX_API_KEY)")
    p.add_argument("--url", default=os.getenv("CFX_URL", DEFAULT_BASE_URL), help="Base URL van CompreFace (default: http://localhost:8000)")
    p.add_argument("--plugins", default=os.getenv("CFX_PLUGINS", "none"), help="Face plugins, bv. 'age,gender' of 'none'")
    p.add_argument("--topk", type=int, default=int(os.getenv("CFX_TOPK", "1")), help="prediction_count (top-K onderwerpen per face)")
    p.add_argument("--threshold", type=float, default=float(os.getenv("CFX_THRESHOLD", "60")), help="Standaard similarity-drempel in %")
    p.add_argument("--host", default="0.0.0.0", help="Bind host (0.0.0.0 om bereikbaar te zijn op LAN)")
    p.add_argument("--port", type=int, default=5000, help="Poort")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()
    if not args.api_key:
        raise SystemExit("Geef een API key via --api-key of zet env var CFX_API_KEY")

    APP_STATE["API_KEY"] = args.api_key
    APP_STATE["BASE_URL"] = args.url.rstrip("/")
    APP_STATE["ENDPOINT"] = f"{APP_STATE['BASE_URL']}/api/v1/recognition/recognize"
    APP_STATE["FACE_PLUGINS"] = args.plugins
    APP_STATE["PREDICTION_COUNT"] = max(1, int(args.topk))
    APP_STATE["DEFAULT_THRESHOLD"] = float(args.threshold)

    print(f"==> CompreFace endpoint: {APP_STATE['ENDPOINT']}")
    print(f"==> Plugins: {APP_STATE['FACE_PLUGINS']} | topK: {APP_STATE['PREDICTION_COUNT']}")
    print(f"==> Start server op http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, debug=False)
