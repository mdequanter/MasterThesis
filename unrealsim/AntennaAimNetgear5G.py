#!/usr/bin/env python3
import argparse, requests, time, sys, signal, json, math, os
from datetime import datetime

URL = "http://192.168.1.1/api/model.json"

# ---------- Helpers ----------
def fetch_status(url=URL, timeout=2.0):
    r = requests.get(url, timeout=timeout)
    r.raise_for_status()
    return r.json()

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
        return "Radio tech: unknown"

# ---------- Quality + score ----------
def bar(val, vmin, vmax, width=30):
    if val is None: return "[" + " " * width + "]"
    clamped = max(vmin, min(vmax, val))
    frac = (clamped - vmin) / (vmax - vmin) if vmax > vmin else 0
    filled = max(0, min(width, int(round(frac * width))))
    return "[" + "#" * filled + "-" * (width - filled) + "]"

def lte_quality_bins(rsrp, rsrq, sinr):
    def lab_rsrp(v): return "Excellent" if v > -80 else "Good" if v > -90 else "Fair" if v > -100 else "Poor"
    def lab_rsrq(v): return "Good" if v > -10 else "Fair" if v > -15 else "Poor"
    def lab_sinr(v): return "Excellent" if v > 20 else "Good" if v >= 13 else "Fair" if v >= 0 else "Poor"
    return (lab_rsrp(rsrp) if rsrp is not None else "n/a",
            lab_rsrq(rsrq) if rsrq is not None else "n/a",
            lab_sinr(sinr) if sinr is not None else "n/a")

def nr_quality_bins(rsrp, rsrq, sinr):
    def lab_rsrp(v): return "Excellent" if v > -80 else "Good" if v > -90 else "Fair" if v > -100 else "Poor"
    def lab_rsrq(v): return "Good" if v > -10 else "Fair" if v > -15 else "Poor"
    def lab_sinr(v): return "Excellent" if v > 20 else "Good" if v >= 10 else "Fair" if v >= 0 else "Poor"
    return (lab_rsrp(rsrp) if rsrp is not None else "n/a",
            lab_rsrq(rsrq) if rsrq is not None else "n/a",
            lab_sinr(sinr) if sinr is not None else "n/a")

def aim_score(nr_sinr, nr_rsrq, nr_rsrp, lte_sinr=None, lte_rsrq=None):
    """
    Heuristic 0..100. We prioritize NR SINR, then NR RSRQ, then NR RSRP.
    Small bonus for better LTE (uplink on NSA).
    """
    score = 0.0
    # Normalize ranges
    if nr_sinr is not None:  # map 0..30+ -> 0..70
        score += max(0.0, min(30.0, nr_sinr)) / 30.0 * 70.0
    if nr_rsrq is not None:  # -20..-5 -> 0..20 (less negative is better)
        score += max(0.0, min(15.0, 20.0 + nr_rsrq)) / 15.0 * 20.0
    if nr_rsrp is not None:  # -120..-80 -> 0..10
        score += max(0.0, min(40.0, 120.0 + nr_rsrp)) / 40.0 * 10.0
    # LTE bonus (uplink stability in NSA)
    if lte_sinr is not None:
        score += max(0.0, min(20.0, lte_sinr)) / 20.0 * 5.0
    if lte_rsrq is not None:
        score += max(0.0, min(10.0, 10.0 + lte_rsrq)) / 10.0 * 5.0
    return round(score, 1)

# ---------- EMA ----------
class EMA:
    def __init__(self, alpha=0.4):
        self.a = alpha
        self.v = None
    def update(self, x):
        if x is None: return self.v
        self.v = x if self.v is None else (self.a * x + (1 - self.a) * self.v)
        return self.v

# ---------- Main loop ----------
def main():
    ap = argparse.ArgumentParser(description="Live 5G aiming meter (Netgear M6 JSON)")
    ap.add_argument("--interval", type=float, default=0.5, help="poll interval seconds (default 0.5)")
    ap.add_argument("--csv", type=str, default="", help="optional CSV path to append samples")
    ap.add_argument("--no-color", action="store_true", help="disable ANSI colors")
    args = ap.parse_args()

    use_color = sys.stdout.isatty() and not args.no_color
    C = lambda code: f"\033[{code}m" if use_color else ""
    CLR = C("0")
    BOLD = C("1")
    GREEN = C("32")
    YELLOW = C("33")
    RED = C("31")
    CYAN = C("36")

    # EMAs to smooth readouts a bit for aiming
    ema_nr_sinr, ema_nr_rsrq, ema_nr_rsrp = EMA(0.35), EMA(0.35), EMA(0.35)
    ema_lte_sinr, ema_lte_rsrq = EMA(0.35), EMA(0.35)

    best = {"score": -1, "ts": None, "vals": {}}

    if args.csv and not os.path.exists(args.csv):
        with open(args.csv, "w") as f:
            f.write("ts,verdict,lte_band,lte_chan,lte_bw,nr_band,nr_chan,nr_bw,"
                    "lte_rsrp,lte_rsrq,lte_sinr,nr_rsrp,nr_rsrq,nr_sinr,score\n")

    def sigint(_sig, _frm):
        print("\nBye.")
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint)

    while True:
        try:
            J = fetch_status()
            wwan = J.get("wwan", {})
            verdict = classify_rat(wwan)

            lte = first_carrier(wwan.get("lteBandInfo") or [])
            nr  = first_carrier(wwan.get("nr5gBandInfo") or [])

            lte_band = f"B{lte.get('band')}" if lte.get("band") is not None else "n/a"
            lte_chan = lte.get("channel", "n/a")
            lte_bw   = lte.get("dlBandwidth", "n/a")
            nr_band  = f"n{nr.get('band')}" if nr.get("band") is not None else "n/a"
            nr_chan  = nr.get("channel", "n/a")
            nr_bw    = nr.get("dlBandwidth", "n/a")

            sig = wwan.get("signalStrength", {})
            lte_rsrp = to_float(sig.get("rsrp"))
            lte_rsrq = to_float(sig.get("rsrq"))
            lte_sinr = to_float(sig.get("sinr"))
            nr_rsrp  = to_float(sig.get("nr5gRsrp"))
            nr_rsrq  = to_float(sig.get("nr5gRsrq"))
            nr_sinr  = to_float(sig.get("nr5gSinr"))

            # Smooth for aiming
            lte_rsrp_s, lte_rsrq_s = lte_rsrp, ema_lte_rsrq.update(lte_rsrq)
            lte_sinr_s = ema_lte_sinr.update(lte_sinr)
            nr_rsrp_s  = ema_nr_rsrp.update(nr_rsrp)
            nr_rsrq_s  = ema_nr_rsrq.update(nr_rsrq)
            nr_sinr_s  = ema_nr_sinr.update(nr_sinr)

            score = aim_score(nr_sinr_s, nr_rsrq_s, nr_rsrp_s, lte_sinr_s, lte_rsrq_s)

            # Best tracker
            if score > best["score"]:
                best["score"] = score
                best["ts"] = datetime.now().isoformat(timespec="seconds")
                best["vals"] = dict(nr_sinr=nr_sinr_s, nr_rsrq=nr_rsrq_s, nr_rsrp=nr_rsrp_s,
                                    lte_sinr=lte_sinr_s, lte_rsrq=lte_rsrq_s)
                # beep on improvement
                print("\a", end="", flush=True)

            # Visuals
            os.system("clear")
            print(f"{BOLD}{verdict}{CLR}")
            print(f"{CYAN}LTE anchor:{CLR} {lte_band} (ch {lte_chan}, DL {lte_bw}) | "
                  f"RSRP {lte_rsrp} dBm | RSRQ {lte_rsrq} dB | SINR {lte_sinr} dB")
            print(f"{CYAN}NR carrier: {CLR} {nr_band} (ch {nr_chan}, DL {nr_bw})  | "
                  f"NR RSRP {nr_rsrp} dBm | NR RSRQ {nr_rsrq} dB | NR SINR {nr_sinr} dB")
            print()

            # Bars (aim using NR first)
            print(f"NR SINR {bar(nr_sinr_s, 0, 30)} {nr_sinr_s:.1f} dB  (target ↑)")
            print(f"NR RSRQ {bar(nr_rsrq_s, -20, -5)} {nr_rsrq_s:.1f} dB (less negative is better)")
            print(f"NR RSRP {bar(nr_rsrp_s, -120, -80)} {nr_rsrp_s:.1f} dBm")
            print(f"LTE SINR {bar(lte_sinr_s, 0, 20)} {lte_sinr_s:.1f} dB  (uplink stability)")
            print(f"LTE RSRQ {bar(lte_rsrq_s, -20, -5)} {lte_rsrq_s:.1f} dB")
            print()

            color = GREEN if score >= 75 else YELLOW if score >= 55 else RED
            print(f"{BOLD}Aim score:{CLR} {color}{score:5.1f}{CLR}    "
                  f"{BOLD}Best:{CLR} {best['score']:5.1f} @ {best['ts']}")

            # CSV logging
            if args.csv:
                with open(args.csv, "a") as f:
                    ts = datetime.now().isoformat()
                    f.write(",".join([
                        ts, verdict, str(lte_band), str(lte_chan), str(lte_bw),
                        str(nr_band), str(nr_chan), str(nr_bw),
                        str(lte_rsrp), str(lte_rsrq), str(lte_sinr),
                        str(nr_rsrp), str(nr_rsrq), str(nr_sinr),
                        str(score)
                    ]) + "\n")

            time.sleep(max(0.05, args.interval))

        except KeyboardInterrupt:
            print("\nBye.")
            sys.exit(0)
        except Exception as e:
            print(f"\nError: {e}", file=sys.stderr)
            time.sleep(1.0)

if __name__ == "__main__":
    main()
