#!/usr/bin/env python3
import argparse, requests, os, subprocess, json, shutil
from datetime import datetime

URL = "http://192.168.1.1/api/model.json"

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

# ---- Ookla speedtest integration ----
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
            # Some versions expose 'bandwidth' (bytes/s), some bits/s; be robust.
            def pick_bandwidth(d):
                bw = (d or {}).get("bandwidth")
                if bw is None:
                    # older schema fallback — rough
                    bw = (d or {}).get("bytes", 0)
                return float(bw or 0)

            def to_mbps(val):
                # Heuristic: if > 1e6 assume bits/s; else bytes/s
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

def write_csv(path, header, row):
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", encoding="utf-8") as f:
        if new:
            f.write(",".join(header) + "\n")
        esc = lambda s: ("" if s is None else str(s).replace("\n"," ").replace(",",";"))
        f.write(",".join(esc(x) for x in row) + "\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", help="path to CSV file to append one row", default="")
    ap.add_argument("--ookla", action="store_true", help="run one Ookla speedtest and include its metrics in CSV/print")
    ap.add_argument("--ookla-server-id", type=int, default=None, help="optional Ookla server id")
    ap.add_argument("--no-single", action="store_true", help="do NOT use --single (use multi-stream test)")
    args = ap.parse_args()

    data = fetch_status()
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

    # Print lines exactly like your console
    lte_line = (f"LTE anchor: {lte_band} (ch {lte_chan}, DL {lte_bw}) | "
                f"RSRP {lte_rsrp} dBm | RSRQ {lte_rsrq} dB | SINR {lte_sinr} dB")
    print(lte_line)

    if wwan.get("nr5gBandInfo"):
        nr_line = (f"NR carrier:  {nr_band} (ch {nr_chan}, DL {nr_bw})  | "
                   f"NR RSRP {nr_rsrp} dBm | NR RSRQ {nr_rsrq} dB | NR SINR {nr_sinr} dB")
    else:
        nr_line = "NR carrier:  n/a"
    print(nr_line)

    # Optionally run Ookla
    ookla = {}
    if args.ookla:
        print("Running Ookla speedtest… (this may take ~30–90s)")
        ookla = run_ookla(single=not args.no_single, server_id=args.ookla_server_id)
        if "error" in ookla:
            print(f"Ookla: {ookla['error']}")
        else:
            print(f"Ookla DL/UL: {ookla['download_Mbps']} / {ookla['upload_Mbps']} Mbps  "
                  f"Ping: {ookla['latency_ms']} ms  Jitter: {ookla['jitter_ms']} ms  "
                  f"Loss: {ookla.get('packetLoss_pct')}%  "
                  f"Server: {ookla.get('server_name')} ({ookla.get('server_loc')})")

    # CSV append (one row)
    if args.csv:
        header = [
            "ts","verdict",
            "lte_band","lte_channel","lte_dl_bw","lte_rsrp_dbm","lte_rsrq_db","lte_sinr_db",
            "nr_band","nr_channel","nr_dl_bw","nr_rsrp_dbm","nr_rsrq_db","nr_sinr_db",
            "lte_line","nr_line",
            "ookla_download_Mbps","ookla_upload_Mbps","ookla_latency_ms","ookla_jitter_ms",
            "ookla_packetLoss_pct","ookla_server","ookla_server_loc","ookla_isp","ookla_error"
        ]
        row = [
            datetime.now().isoformat(timespec="seconds"), verdict,
            lte_band, lte_chan, lte_bw, lte_rsrp, lte_rsrq, lte_sinr,
            nr_band, nr_chan, nr_bw, nr_rsrp, nr_rsrq, nr_sinr,
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
        write_csv(args.csv, header, row)

if __name__ == "__main__":
    main()
