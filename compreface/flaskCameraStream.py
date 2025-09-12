# stream_app.py
#!/usr/bin/env python3
import os
import argparse
from flask import Flask, request, render_template_string, jsonify
import requests

HTML_PAGE = """
<!doctype html>
<html lang="nl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Live herkenning – CompreFace</title>
  <style>
    :root { --w: min(92vw, 680px); }
    body { font-family: system-ui, Arial, sans-serif; margin: 18px; color:#111; }
    .card { max-width: var(--w); margin: 0 auto; padding: 14px; border: 1px solid #e5e7eb; border-radius: 12px; }
    h1 { margin: 4px 0 12px 0; font-size: 1.2rem; }
    .row { display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
    .hint { color:#6b7280; font-size:.95rem; }
    .video-wrap { position:relative; width: var(--w); }
    video, canvas { width: 100%; height: auto; border-radius: 10px; }
    #overlay { position:absolute; top:0; left:0; }
    .pill { display:inline-block; padding:6px 10px; border-radius:9999px; background:#eef2ff; color:#3730a3; font-weight:600; }
    .warn { color:#9a3412; }
    .ok { color:#065f46; }
    .err { color:#991b1b; }
    .grid { display:grid; grid-template-columns: 1fr 1fr; gap:8px; }
    input[type=number]{ width:90px; }
    button { padding:8px 12px; border:0; border-radius:8px; background:#111827; color:#fff; font-weight:600; }
  </style>
</head>
<body>
  <div class="card">
    <h1>Live herkenning – CompreFace</h1>
    <p class="hint">Deze pagina gebruikt je smartphonecamera en stuurt elke {{interval_ms}} ms een frame naar de server voor herkenning.</p>

    <div class="grid" style="margin:8px 0 12px;">
      <label>Drempel (min. similarity %)
        <input id="thresh" type="number" min="0" max="100" step="1" value="{{threshold}}">
      </label>
      <label>Resolutie
        <select id="res">
          <option value="640x480">640×480</option>
          <option value="960x540">960×540</option>
          <option value="1280x720" selected>1280×720</option>
        </select>
      </label>
      <label>Interval (ms)
        <input id="interval" type="number" min="100" step="50" value="{{interval_ms}}">
      </label>
      <label>Top-K
        <input id="topk" type="number" min="1" step="1" value="{{topk}}">
      </label>
    </div>

    <div class="row" style="margin-bottom:10px;">
      <button id="btnStart">Start</button>
      <button id="btnStop" disabled>Stop</button>
      <span id="status" class="pill">idle</span>
    </div>

    <div class="video-wrap">
      <video id="video" playsinline autoplay muted></video>
      <canvas id="overlay"></canvas>
      <canvas id="grab" style="display:none;"></canvas>
    </div>

    <div style="margin-top:10px;">
      <div id="best" class="hint">Beste gok: —</div>
      <div id="meta" class="hint">fps: — | latency: — ms | faces: —</div>
      <div class="hint">API: {{base_url}} — plugins={{plugins}} — prediction_count={{topk}}</div>
      <div id="err" class="err"></div>
    </div>
  </div>

<script>
const video = document.getElementById('video');
const overlay = document.getElementById('overlay');
const grab = document.getElementById('grab');
const ctx = overlay.getContext('2d');
const gtx = grab.getContext('2d');

const statusEl = document.getElementById('status');
const bestEl = document.getElementById('best');
const metaEl = document.getElementById('meta');
const errEl  = document.getElementById('err');

const btnStart = document.getElementById('btnStart');
const btnStop  = document.getElementById('btnStop');

const threshEl = document.getElementById('thresh');
const resEl    = document.getElementById('res');
const intEl    = document.getElementById('interval');
const topkEl   = document.getElementById('topk');

let timer = null;
let running = false;
let lastCallAt = 0;

async function start() {
  errEl.textContent = "";
  if (running) return;

  // camera resolutie
  const [w,h] = resEl.value.split('x').map(Number);
  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: { facingMode: 'environment', width: { ideal: w }, height: { ideal: h } },
      audio: false
    });
    video.srcObject = stream;
    await video.play();
  } catch (e) {
    errEl.textContent = "Camera toegang geweigerd of niet beschikbaar: " + e;
    return;
  }

  // canvas dimensies = videodimensies
  await new Promise(r => setTimeout(r, 100)); // kleine delay
  const vw = video.videoWidth || w;
  const vh = video.videoHeight || h;
  overlay.width = vw; overlay.height = vh;
  grab.width = vw; grab.height = vh;

  running = true;
  btnStart.disabled = true;
  btnStop.disabled = false;
  statusEl.textContent = "running";

  const loop = async () => {
    if (!running) return;

    const intervalMs = Math.max(100, parseInt(intEl.value||{{interval_ms}}));
    const threshold  = Math.max(0, Math.min(100, parseFloat(threshEl.value||{{threshold}})));
    const topk       = Math.max(1, parseInt(topkEl.value||{{topk}}));

    // teken huidig frame in hidden canvas
    gtx.drawImage(video, 0, 0, grab.width, grab.height);
    const t0 = performance.now();
    const blob = await new Promise(res => grab.toBlob(res, 'image/jpeg', 0.7));
    try {
      const form = new FormData();
      form.append('file', blob, 'frame.jpg');
      form.append('threshold', String(threshold));
      form.append('topk', String(topk));

      const resp = await fetch('/api/recognize_frame', { method: 'POST', body: form });
      if (!resp.ok) throw new Error("HTTP " + resp.status);
      const data = await resp.json();
      const t1 = performance.now();

      drawOverlay(data, grab.width, grab.height);
      const best = data.best;
      const faces = data.faces || [];
      if (best) {
        const pct = (best.similarity*100).toFixed(1);
        bestEl.innerHTML = `Beste gok: <span class="${pct >= threshold ? 'ok' : 'warn'}">${best.subject} (${pct}%)</span>`;
      } else {
        bestEl.textContent = "Beste gok: — (geen match boven drempel)";
      }
      const latency = Math.round(t1 - t0);
      metaEl.textContent = `fps: ~${Math.round(1000/intervalMs)} | latency: ${latency} ms | faces: ${faces.length}`;
      errEl.textContent = "";
    } catch (e) {
      errEl.textContent = "Fout bij upload/herkenning: " + e;
    } finally {
      lastCallAt = performance.now();
      timer = setTimeout(loop, intervalMs);
    }
  };
  loop();
}

function stop() {
  running = false;
  btnStart.disabled = false;
  btnStop.disabled = true;
  statusEl.textContent = "stopped";
  if (timer) { clearTimeout(timer); timer = null; }
  if (video.srcObject) {
    for (const track of video.srcObject.getTracks()) track.stop();
    video.srcObject = null;
  }
  ctx.clearRect(0,0,overlay.width, overlay.height);
}

function drawOverlay(data, w, h) {
  ctx.clearRect(0,0,w,h);
  const faces = data.faces || [];
  ctx.lineWidth = 2;
  ctx.font = "16px system-ui, Arial";
  for (const f of faces) {
    const b = f.box || {};
    const x = b.x_min||0, y=b.y_min||0, X=b.x_max||0, Y=b.y_max||0;
    ctx.strokeStyle = "lime";
    ctx.strokeRect(x, y, X - x, Y - y);
    const label = `${f.subject||'Unknown'} ${(f.similarity*100).toFixed(1)}%`;
    const tw = ctx.measureText(label).width + 8;
    ctx.fillStyle = "rgba(0,255,0,0.85)";
    ctx.fillRect(x, Math.max(0, y - 22), tw, 20);
    ctx.fillStyle = "#000";
    ctx.fillText(label, x+4, Math.max(14, y - 6));
  }
}

btnStart.addEventListener('click', start);
btnStop.addEventListener('click', stop);

// auto-start op pagina-load? => uit; iOS kan een user gesture vereisen.
</script>
</body>
</html>
"""

def create_app(api_key: str, base_url: str, plugins: str, topk: int, det_prob: float, timeout_s: int):
    app = Flask(__name__)
    app.config["API_KEY"] = api_key
    app.config["BASE_URL"] = base_url.rstrip("/")
    app.config["RECOGNIZE_URL"] = f"{app.config['BASE_URL']}/api/v1/recognition/recognize"
    app.config["PLUGINS"] = plugins
    app.config["TOPK"] = max(1, int(topk))
    app.config["DET_PROB"] = float(det_prob)
    app.config["TIMEOUT"] = int(timeout_s)
    app.config["DEFAULT_THRESHOLD"] = 60
    app.config["INTERVAL_MS"] = 500

    @app.get("/")
    def root():
        return render_template_string(
            HTML_PAGE,
            base_url=app.config["BASE_URL"],
            plugins=app.config["PLUGINS"],
            topk=app.config["TOPK"],
            threshold=app.config["DEFAULT_THRESHOLD"],
            interval_ms=app.config["INTERVAL_MS"],
        )

    @app.post("/api/recognize_frame")
    def recognize_frame():
        # Ontvang snapshot van de browser
        f = request.files.get("file")
        if not f:
            return jsonify({"error": "no file"}), 400

        # Client-instellingen
        try:
            threshold = float(request.form.get("threshold", app.config["DEFAULT_THRESHOLD"]))
        except ValueError:
            threshold = app.config["DEFAULT_THRESHOLD"]
        try:
            topk = int(request.form.get("topk", app.config["TOPK"]))
        except ValueError:
            topk = app.config["TOPK"]

        files = {"file": (f.filename or "frame.jpg", f.read(), "image/jpeg")}
        headers = {"x-api-key": app.config["API_KEY"]}
        params = {
            "prediction_count": max(1, topk),
            "det_prob_threshold": app.config["DET_PROB"],
            "face_plugins": app.config["PLUGINS"],
            "status": "false",
            "detect_faces": "true",
        }
        try:
            r = requests.post(
                app.config["RECOGNIZE_URL"],
                headers=headers,
                params=params,
                files=files,
                timeout=app.config["TIMEOUT"],
            )
            r.raise_for_status()
            data = r.json()
        except requests.RequestException as e:
            return jsonify({"error": str(e)}), 502

        # Bouw een compacter antwoord terug voor de browser
        faces = []
        for item in (data.get("result") or []):
            subj = "Unknown"
            sim = 0.0
            subs = item.get("subjects") or []
            if subs:
                best = subs[0]
                subj = best.get("subject") or "Unknown"
                try:
                    sim = float(best.get("similarity", 0.0))
                except Exception:
                    sim = 0.0
            box = item.get("box") or {}
            faces.append({
                "subject": subj,
                "similarity": sim,
                "box": {
                    "x_min": int(box.get("x_min", 0)),
                    "y_min": int(box.get("y_min", 0)),
                    "x_max": int(box.get("x_max", 0)),
                    "y_max": int(box.get("y_max", 0)),
                }
            })

        best = max(faces, key=lambda x: x["similarity"]) if faces else None
        # Filter onder drempel niet weg; UI toont kleur (ok/warn). Desgewenst kun je ze hier ook filteren.

        return jsonify({
            "faces": faces,
            "best": best if best else None,
            "threshold": threshold,
        })

    return app

def parse_args():
    p = argparse.ArgumentParser(description="Flask live herkenning vanaf smartphonecamera via CompreFace")
    p.add_argument("--api-key", default=os.getenv("CFX_API_KEY"), required=False,
                   help="CompreFace Recognition service API key (of env CFX_API_KEY)")
    p.add_argument("--url", default=os.getenv("CFX_URL", "http://localhost:8000"),
                   help="Base URL van CompreFace (default: http://localhost:8000)")
    p.add_argument("--plugins", default=os.getenv("CFX_PLUGINS", "none"),
                   help="Face plugins (bv. 'age,gender' of 'none')")
    p.add_argument("--topk", type=int, default=int(os.getenv("CFX_TOPK", "1")),
                   help="prediction_count (top-K onderwerpen per face)")
    p.add_argument("--det-prob", type=float, default=float(os.getenv("CFX_DET_PROB", "0.6")),
                   help="Detector probability threshold (0..1)")
    p.add_argument("--timeout", type=int, default=int(os.getenv("CFX_TIMEOUT", "8")),
                   help="HTTP timeout naar CompreFace (s)")
    p.add_argument("--host", default="0.0.0.0", help="Bind host (0.0.0.0 om bereikbaar te zijn op LAN)")
    p.add_argument("--port", type=int, default=5000, help="Poort")
    p.add_argument("--https", action="store_true", help="Start met self-signed HTTPS (handig voor iOS camera)")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()
    if not args.api_key:
        raise SystemExit("Geef een API key via --api-key of zet env CFX_API_KEY")

    app = create_app(
        api_key=args.api_key,
        base_url=args.url,
        plugins=args.plugins,
        topk=args.topk,
        det_prob=args.det_prob,
        timeout_s=args.timeout,
    )

    print(f"==> CompreFace: {app.config['RECOGNIZE_URL']}")
    print(f"==> Plugins: {app.config['PLUGINS']} | topK: {app.config['TOPK']} | det_prob: {app.config['DET_PROB']}")
    print(f"==> Open: http{'s' if args.https else ''}://{args.host}:{args.port}")
    if args.https:
        # Zelfondertekend certificaat; werkt meestal op iOS/Safari (klik 'doorgaan' op waarschuwing).
        # Vereist cryptography; installeer zo nodig: pip install cryptography
        app.run(host=args.host, port=args.port, debug=False, ssl_context="adhoc")
    else:
        app.run(host=args.host, port=args.port, debug=False)
