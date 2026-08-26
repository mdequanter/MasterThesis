#!/usr/bin/env python3
import argparse, requests, os, subprocess, json, shutil, time, signal, shlex
from datetime import datetime

URL = "http://192.168.1.1/api/model.json"

# ---------- Router helpers ----------
def fetch_status(url=URL, timeout=3):
    r = requests.get(url, timeout=timeout)
    r.raise_for_status()
    return r.json()

def classify_rat(wwan: dict) -> str:
    ps  = str(wwan.get("currentPSserviceType", "")).lower()
    nw  = str(wwan.get("currentNWserviceType", "")).lower()
    txt = str(wwan.get("connectionText", "")).lower()
    diag = (wwan.get("diagInfo") or [{}])[0]
    lte_att = bool(diag.get("lteAttached"))
    nr_att  = bool(diag.get("nr5gAttached"))
    endc    = bool(diag.get("endcEnabledConfig"))
    is5g = any(s and ("5g" in s or "nr" in s) for s in (ps, nw, txt)) or nr_att or bool(wwan.get("nr5gBandInfo"))
    is4g = any(s and ("lte" in s or "4g" in s) for s in (ps, nw, txt)) or lte_att or bool(wwan.get("lteBandInfo"))
    if is5g:
        if (nr_att and lte_att) or endc: return "Connected via 5G (NR-NSA)"
        elif nr_att and not lte_att:     return "Connected via 5G (NR-SA)"
        else:                            return "Connected via 5G (NR)"
    elif is4g:
        return "Connected via 4G (LTE)"
    else:
        return "Radio tech: unknown (keys not found)"

def to_float(x):
    if x is None: return None
    try:
        if isinstance(x, (int, float)): return float(x)
        return float(str(x).split()[0])
    except Exception:
        return None

def first_carrier(info_list):
    if not info_list: return {}
    for c in info_list:
        if isinstance(c, dict) and c.get("isPcc") is True:
            return c
    return info_list[0] if isinstance(info_list[0], dict) else {}

# ---------- Ookla integration ----------
def run_ookla(single=True, server_id=None, timeout=180):
    if not shutil.which("speedtest"):
        return {"error": "Ookla speedtest not installed"}

    base = ["speedtest", "-f", "json", "--accept-license", "--accept-gdpr", "--progress=no"]
    if server_id:
        base += ["-s", str(server_id)]
    cmds = []
    if single:
        cmds.append(base + ["--single"])
    cmds.append(base)

    last_err = None
    for cmd in cmds:
        try:
            out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, timeout=timeout)
            data = json.loads(out.decode("utf-8", "ignore"))

            def pick_bandwidth(d):
                bw = (d or {}).get("bandwidth")
                if bw is None:
                    bw = (d or {}).get("bytes", 0)
                return float(bw or 0)

            def to_mbps(val):
                v = float(val)
                return (v / 1e6) if v > 1e6 else (v * 8 / 1e6)

            dl_mbps = round(to_mbps(pick_bandwidth(data.get("download"))), 2)
            ul_mbps = round(to_mbps(pick_bandwidth(data.get("upload"))),   2)
            ping_ms = round(((data.get("ping") or {}).get("latency") or 0.0), 2)
            jitter  = round(((data.get("ping") or {}).get("jitter")  or 0.0), 2)
            loss    = data.get("packetLoss")
            isp     = data.get("isp") or data.get("ispName") or ""
            srv     = (data.get("server") or {})
            srv_name = srv.get("name") or ""
            srv_loc  = srv.get("location") or ""

            return {
                "download_Mbps": dl_mbps,
                "upload_Mbps": ul_mbps,
                "latency_ms": ping_ms,
                "jitter_ms": jitter,
                "packetLoss_pct": loss,
                "isp": isp,
                "server_name": srv_name,
                "server_loc": srv_loc
            }
        except Exception as e:
            last_err = e
            continue
    return {"error": str(last_err) if last_err else "speedtest failed"}

# ---------- GPS (USB NMEA) ----------
def nmea_to_deg(val_str, hemi):
    try:
        if not val_str:
            return None
        v = float(val_str)
        deg = int(v // 100)
        minutes = v - deg * 100
        dec = deg + minutes / 60.0
        if hemi in ("S", "W"):
            dec = -dec
        return dec
    except Exception:
        return None

def read_gps_once(port="/dev/ttyUSB0", baud=9600, timeout_s=1.5, max_lines=120):
    try:
        import serial
    except ImportError:
        return {"error": "pyserial not installed", "lat": None, "lon": None, "alt_m": None, "fix_quality": None, "sats": None}

    lat = lon = alt_m = None
    fixq = None
    sats = None
    got_pos = got_alt = False

    try:
        with serial.Serial(port, baudrate=baud, timeout=timeout_s) as ser:
            lines_read = 0
            start = time.time()
            while lines_read < max_lines and (time.time() - start) < timeout_s:
                raw = ser.readline().decode("ascii", "ignore").strip()
                if not raw or raw[0] != "$":
                    continue
                lines_read += 1
                parts = raw.split(",")
                # RMC: $GPRMC/$GNRMC
                if parts[0].endswith("RMC") and len(parts) >= 7:
                    status = parts[2] if len(parts) > 2 else "V"
                    if status == "A":
                        la = nmea_to_deg(parts[3], parts[4] if len(parts) > 4 else "")
                        lo = nmea_to_deg(parts[5], parts[6] if len(parts) > 6 else "")
                        if la is not None and lo is not None:
                            lat, lon = la, lo
                            got_pos = True
                # GGA: $GPGGA/$GNGGA
                if parts[0].endswith("GGA") and len(parts) >= 11:
                    la = nmea_to_deg(parts[2], parts[3] if len(parts) > 3 else "")
                    lo = nmea_to_deg(parts[4], parts[5] if len(parts) > 5 else "")
                    try:    fq   = int(parts[6])
                    except: fq   = None
                    try:    satn = int(parts[7])
                    except: satn = None
                    try:    alt  = float(parts[9])
                    except: alt  = None
                    if la is not None and lo is not None:
                        lat, lon = la, lo
                        got_pos = True
                    if alt is not None:
                        alt_m = alt
                        got_alt = True
                    if fq is not None:
                        fixq = fq
                    if satn is not None:
                        sats = satn
                if got_pos and got_alt:
                    break
    except Exception as e:
        return {"error": str(e), "lat": None, "lon": None, "alt_m": None, "fix_quality": None, "sats": None}

    return {"lat": lat, "lon": lon, "alt_m": alt_m, "fix_quality": fixq, "sats": sats}

# ---------- Speech (piper TTS) ----------
PIPER_MODEL = "/home/pi/faceassist/voices/nl_BE-nathalie-medium.onnx"

def _nl_num(x, decimals=0):
    """Format a number for Dutch TTS (decimal comma)."""
    if x is None:
        return None
    try:
        txt = f"{float(x):.{decimals}f}"
    except Exception:
        return None
    return txt.replace(".", ",")

def build_speech(prefix, ookla, gps):
    """Build the spoken summary: latency, upload/download and GPS availability."""
    parts = [prefix.rstrip(".")]

    if not ookla:
        parts.append("geen snelheidstest")
    elif ookla.get("error"):
        parts.append("snelheidstest mislukt")
    else:
        lat_ms = _nl_num(ookla.get("latency_ms"))
        dl     = _nl_num(ookla.get("download_Mbps"))
        ul     = _nl_num(ookla.get("upload_Mbps"))
        if lat_ms is not None:
            parts.append(f"latentie {lat_ms} milliseconden")
        if dl is not None:
            parts.append(f"download {dl} megabit per seconde")
        if ul is not None:
            parts.append(f"upload {ul} megabit per seconde")

    gps = gps or {}
    if gps.get("lat") is not None and gps.get("lon") is not None:
        parts.append("G P S beschikbaar")
    else:
        parts.append("geen G P S")

    return ". ".join(parts) + "."

def speak(text="Meting uitgevoerd", model=PIPER_MODEL, timeout=30):
    """Speak `text` through piper -> aplay. Never raises; returns True on success."""
    if not shutil.which("piper") or not shutil.which("aplay"):
        return False
    cmd = (
        f"echo {shlex.quote(text)} | piper --model {shlex.quote(model)} --output_raw "
        f"| aplay -r 22050 -f S16_LE -c 1 -t raw"
    )
    try:
        subprocess.run(cmd, shell=True, timeout=timeout,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception as e:
        print(f"TTS error: {e}")
        return False

# ---------- CSV ----------
def write_csv(path, header, row):
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", encoding="utf-8") as f:
        if new:
            f.write(",".join(header) + "\n")
        esc = lambda s: ("" if s is None else str(s).replace("\n"," ").replace(",",";"))
        f.write(",".join(esc(x) for x in row) + "\n")

# ---------- Main (repeating) ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", help="path to CSV file to append rows", default="")
    ap.add_argument("--interval", type=float, default=120.0, help="repeat interval in seconds (default 120)")
    ap.add_argument("--runs", type=int, default=0, help="number of iterations (0 = run forever)")
    ap.add_argument("--ookla", action="store_true", help="run one Ookla speedtest each iteration")
    ap.add_argument("--ookla-server-id", type=int, default=None, help="optional Ookla server id")
    ap.add_argument("--no-single", action="store_true", help="do NOT use --single (use multi-stream test)")
    ap.add_argument("--gps-port", type=str, default="", help="GPS serial port (e.g., /dev/ttyUSB0). Leave empty to skip.")
    ap.add_argument("--gps-baud", type=int, default=9600, help="GPS baud rate (default 9600)")
    ap.add_argument("--no-speak", action="store_true", help="do NOT speak after each measurement")
    ap.add_argument("--speak-text", type=str, default="Meting uitgevoerd", help="spoken prefix, followed by latency/speeds/GPS status")
    ap.add_argument("--piper-model", type=str, default=PIPER_MODEL, help="path to the piper .onnx voice model")
    args = ap.parse_args()

    stop = {"flag": False}
    def _sigint(_s, _f):
        stop["flag"] = True
        print("\nStopping…")
    signal.signal(signal.SIGINT, _sigint)

    header = [
        "ts","verdict",
        "lte_band","lte_channel","lte_dl_bw","lte_rsrp_dbm","lte_rsrq_db","lte_sinr_db",
        "nr_band","nr_channel","nr_dl_bw","nr_rsrp_dbm","nr_rsrq_db","nr_sinr_db",
        "gps_lat","gps_lon","gps_alt_m","gps_fix_quality","gps_sats",
        "lte_line","nr_line",
        "ookla_download_Mbps","ookla_upload_Mbps","ookla_latency_ms","ookla_jitter_ms",
        "ookla_packetLoss_pct","ookla_server","ookla_server_loc","ookla_isp","ookla_error"
    ]

    iteration = 0
    next_t = time.monotonic()

    while not stop["flag"]:
        iteration += 1
        ts = datetime.now().isoformat(timespec="seconds")
        print(f"\n=== [{ts}] Iteration {iteration} ===")

        # Router status
        try:
            data = fetch_status()
        except Exception as e:
            print(f"Router fetch error: {e}")
            data = {}

        wwan = data.get("wwan", {})
        verdict = classify_rat(wwan)
        print(verdict)

        # Bands / carriers
        lte_c = first_carrier(wwan.get("lteBandInfo") or [])
        nr_c  = first_carrier(wwan.get("nr5gBandInfo") or [])
        lte_band = f"B{lte_c.get('band')}" if lte_c.get("band") is not None else "n/a"
        lte_chan = lte_c.get("channel", "n/a")
        lte_bw   = lte_c.get("dlBandwidth", "n/a")
        nr_band  = f"n{nr_c.get('band')}" if nr_c.get("band") is not None else "n/a"
        nr_chan  = nr_c.get("channel", "n/a")
        nr_bw    = nr_c.get("dlBandwidth", "n/a")

        # Signal strength
        sig = wwan.get("signalStrength", {})
        lte_rsrp = to_float(sig.get("rsrp"))
        lte_rsrq = to_float(sig.get("rsrq"))
        lte_sinr = to_float(sig.get("sinr"))
        nr_rsrp  = to_float(sig.get("nr5gRsrp"))
        nr_rsrq  = to_float(sig.get("nr5gRsrq"))
        nr_sinr  = to_float(sig.get("nr5gSinr"))

        # Console lines
        lte_line = (f"LTE anchor: {lte_band} (ch {lte_chan}, DL {lte_bw}) | "
                    f"RSRP {lte_rsrp} dBm | RSRQ {lte_rsrq} dB | SINR {lte_sinr} dB")
        print(lte_line)
        if wwan.get("nr5gBandInfo"):
            nr_line = (f"NR carrier:  {nr_band} (ch {nr_chan}, DL {nr_bw})  | "
                       f"NR RSRP {nr_rsrp} dBm | NR RSRQ {nr_rsrq} dB | NR SINR {nr_sinr} dB")
        else:
            nr_line = "NR carrier:  n/a"
        print(nr_line)

        # GPS (optional)
        gps = {"lat": None, "lon": None, "alt_m": None, "fix_quality": None, "sats": None}
        if args.gps_port:
            gps = read_gps_once(port=args.gps_port, baud=args.gps_baud)
            if "error" in gps and gps["error"]:
                print(f"GPS: {gps['error']}")
            else:
                print(f"GPS lat/lon/alt: {gps['lat']}, {gps['lon']}, {gps['alt_m']} m "
                      f"(fix={gps.get('fix_quality')}, sats={gps.get('sats')})")

        # Ookla (optional) – note this may take time
        ookla = {}
        if args.ookla:
            print("Running Ookla speedtest…")
            ookla = run_ookla(single=not args.no_single, server_id=args.ookla_server_id)
            if "error" in ookla:
                print(f"Ookla: {ookla['error']}")
            else:
                print(f"Ookla DL/UL: {ookla['download_Mbps']} / {ookla['upload_Mbps']} Mbps  "
                      f"Ping: {ookla['latency_ms']} ms  Jitter: {ookla['jitter_ms']} ms  "
                      f"Loss: {ookla.get('packetLoss_pct')}%  "
                      f"Server: {ookla.get('server_name')} ({ookla.get('server_loc')})")

        # CSV write


        if args.csv:

            keys = [
                "ts", "verdict",
                "lte_band", "lte_chan", "lte_bw", "lte_rsrp", "lte_rsrq", "lte_sinr",
                "nr_band", "nr_chan", "nr_bw", "nr_rsrp", "nr_rsrq", "nr_sinr",
                "gps_lat", "gps_lon", "gps_alt_m", "gps_fix_quality", "gps_sats",
                "lte_line", "nr_line",
                "download_Mbps", "upload_Mbps", "latency_ms", "jitter_ms",
                "packetLoss_pct", "server_name", "server_loc", "isp", "error"
    ]

            row = [
                ts, verdict,
                lte_band, lte_chan, lte_bw, lte_rsrp, lte_rsrq, lte_sinr,
                nr_band, nr_chan, nr_bw, nr_rsrp, nr_rsrq, nr_sinr,
                gps.get("lat"), gps.get("lon"), gps.get("alt_m"), gps.get("fix_quality"), gps.get("sats"),
                lte_line, nr_line,
                (ookla.get("download_Mbps") if ookla else None),
                (ookla.get("upload_Mbps") if ookla else None),
                (ookla.get("latency_ms") if ookla else None),
                (ookla.get("jitter_ms") if ookla else None),
                (ookla.get("packetLoss_pct") if ookla else None),
                (ookla.get("server_name") if ookla else None),
                (ookla.get("server_loc") if ookla else None),
                (ookla.get("isp") if ookla else None),
                (ookla.get("error") if "error" in ookla else None),
            ]
        print("Row preview:")
        for k, v in zip(keys, row):
            print(f"{k}: {v}")  

        write_csv(args.csv, header, row)

        # Announce the measurement result out loud
        if not args.no_speak:
            phrase = build_speech(args.speak_text, ookla, gps)
            print(f"Speaking: {phrase}")
            speak(phrase, model=args.piper_model)

        # Stop after N runs if requested
        if args.runs and iteration >= args.runs:
            print("Reached requested number of runs. Exiting.")
            break

        # Schedule next run (monotonic, minimal drift)
        next_t += args.interval
        now = time.monotonic()
        # If we overran the interval (e.g., because Ookla took long), catch up
        if now > next_t:
            next_t = now
        sleep_s = max(0.0, next_t - now)
        if sleep_s > 0:
            print(f"Next run in {sleep_s:.1f} s (Ctrl+C to stop)")
            try:
                time.sleep(sleep_s)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    main()
