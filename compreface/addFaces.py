#!/usr/bin/env python3
"""
Upload alle beelden uit ./faces naar CompreFace en maak per bestand een subject
met als naam de bestandsnaam (zonder extensie).

Voorbeeld:
  faces/
    alice.jpg   -> subject "alice"
    bob.png     -> subject "bob"

Gebruik:
  python add_faces.py --api-key YOUR_KEY [--dir faces] [--url http://localhost:8000]
"""
import argparse
import sys
from pathlib import Path
import requests

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

def upload_face(endpoint: str, api_key: str, img_path: Path, subject: str, timeout: int = 10):
    headers = {"x-api-key": api_key}
    params = {"subject": subject}
    files = {"file": (img_path.name, img_path.read_bytes(), "application/octet-stream")}
    r = requests.post(endpoint, headers=headers, params=params, files=files, timeout=timeout)
    return r

def main():
    ap = argparse.ArgumentParser(description="Upload faces to CompreFace using filename as subject.")
    ap.add_argument("--api-key", required=True, help="CompreFace Recognition service API key")
    ap.add_argument("--url", default="http://localhost:8000", help="Base URL van CompreFace (default: http://localhost:8000)")
    ap.add_argument("--dir", default="faces", help="Map met gezichten (default: faces)")
    ap.add_argument("--dry-run", action="store_true", help="Toon wat er zou gebeuren zonder te uploaden")
    args = ap.parse_args()

    base_url = args.url.rstrip("/")
    endpoint = f"{base_url}/api/v1/recognition/faces"

    faces_dir = Path(args.dir)
    if not faces_dir.exists() or not faces_dir.is_dir():
        print(f"[FOUT] Map niet gevonden: {faces_dir.resolve()}", file=sys.stderr)
        sys.exit(1)

    imgs = sorted([p for p in faces_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS and p.is_file()])
    if not imgs:
        print(f"[INFO] Geen afbeeldingsbestanden gevonden in {faces_dir.resolve()}. Extensies: {sorted(IMAGE_EXTS)}")
        sys.exit(0)

    ok, fail = 0, 0
    for p in imgs:
        subject = p.stem  # bestandsnaam zonder extensie
        if args.dry_run:
            print(f"[DRY-RUN] Zou uploaden: {p.name} -> subject '{subject}'")
            continue

        try:
            resp = upload_face(endpoint, args.api_key, p, subject)
            if resp.ok:
                # Succes
                ok += 1
                print(f"[OK] {p.name} -> '{subject}' (status {resp.status_code})")
            else:
                fail += 1
                try:
                    err = resp.json()
                except Exception:
                    err = resp.text[:300]
                print(f"[FOUT] {p.name} -> '{subject}' (status {resp.status_code})\n      {err}", file=sys.stderr)
        except requests.RequestException as e:
            fail += 1
            print(f"[FOUT] {p.name} -> '{subject}' netwerk/HTTP: {e}", file=sys.stderr)

    print(f"\nKlaar. Succes: {ok} | Fouten: {fail} | Totaal: {ok+fail}")

if __name__ == "__main__":
    main()
