"""Holding-pattern entries and wind correction: the model behind the holding-entries video.

Every rule is from FAA AIM 5-3-8 (read 2026-10-05); the script's Model section quotes them.
- Entry by heading on arrival at the fix: parallel, teardrop or direct, split by the 70-degree
  line (AIM FIG 5-3-8, "Holding Pattern Entry Procedures").
- Teardrop: "a 30 degree teardrop entry within the pattern (on the holding side) for a period
  of one minute". Parallel: outbound on the nonholding side for one minute, then turn toward
  the holding side through more than 180 degrees.
- Legs: 1 minute at or below 14,000 ft MSL, 1 1/2 above. Speeds: 200 KIAS (MHA-6,000 ft),
  265 KIAS (14,001 ft and above).
- Turns: 3 deg/s or 30 deg bank, whichever needs less bank (no flight director).
- Outbound correction: "triple the inbound drift correction".

My choices, which the AIM does not fix:
- Indicated airspeed is taken as calibrated airspeed (no position or instrument error) and
  converted to true airspeed for a stated altitude on a standard day (see ktas_for). The
  low case is flown at sea level, where 200 KCAS is 200 KTAS.
- Outbound timing starts over the fix for teardrop and parallel entries; for direct entries
  and in the wind test, at abeam the fix or at the end of the outbound turn, whichever is later.
- The return to the inbound course is a 45-degree-capped intercept (direct, teardrop) or
  homing on the fix (parallel). Real pilots fly these by eye and needle.

Frame: fix at the origin, inbound course 360 (flying north to the fix), right turns, so the
holding side is east (x > 0). Distances in nautical miles, time in seconds.

Usage: python3 holding.py [out.ppm]. Every run also writes out/holding.json, the only source of
the numbers and tracks the video's scenes show.
"""
import json
import math
import os
import statistics
import sys

G = 9.80665                 # m/s^2
KT = 1852 / 3600            # m/s per knot
DT = 0.25
GAMMA = 1.4                 # NASA Glenn, Isentropic Flow Equations (air)
FT = 0.3048                 # m per ft


def atmosphere(alt_ft):
    """NASA Glenn 'Earth Atmosphere Model - Metric Units', troposphere curve fit (h < 11,000 m):
    T = 15.04 - .00649 h (deg C), p = 101.29 [(T + 273.1)/288.08]^5.256 (kPa),
    r = p / [.2869 (T + 273.1)] (kg/m^3). Returns (p in Pa, T in K)."""
    h = alt_ft * FT
    assert h < 11000
    t = 15.04 - 0.00649 * h
    p = 101.29 * ((t + 273.1) / 288.08) ** 5.256
    return p * 1000, t + 273.1


def sound(temp_k):
    """a = sqrt(gam R T) (NASA Glenn, Eq #2), R = 286.9 J/(kg K) from the same model's .2869."""
    return math.sqrt(GAMMA * 286.9 * temp_k)


def ktas_for(kcas, alt_ft):
    """14 CFR 1.1: indicated airspeed is 'calibrated to reflect standard atmosphere adiabatic
    compressible flow at sea level', and calibrated airspeed 'is equal to true airspeed in
    standard atmosphere at sea level'. So the impact pressure (pt - p) is the one this speed
    makes at sea level, and the Mach number aloft is whatever gives that same impact pressure
    there, both through p/pt = [1 + M^2 (gam-1)/2]^-[gam/(gam-1)] (NASA Glenn, Eq #6)."""
    e = GAMMA / (GAMMA - 1)
    p0, t0 = atmosphere(0)
    m0 = kcas * KT / sound(t0)
    qc = p0 * ((1 + (GAMMA - 1) / 2 * m0 ** 2) ** e - 1)
    p, t = atmosphere(alt_ft)
    m = math.sqrt(2 / (GAMMA - 1) * ((qc / p + 1) ** (1 / e) - 1))
    return m * sound(t) / KT


def wrap(a):
    return (a + 180) % 360 - 180


def turn_rate(ktas):
    """Degrees per second: 3 deg/s or 30 deg bank, whichever requires the least bank."""
    bank30 = math.degrees(G * math.tan(math.radians(30)) / (ktas * KT))
    return min(3.0, bank30)


def bank_for(ktas, rate):
    return math.degrees(math.atan(ktas * KT * math.radians(rate) / G))


def entry_for(heading):
    """AIM FIG 5-3-8, with the inbound course at 000: the 70-degree line meets the holding side
    at 70 degrees from the outbound course, so direct 290-110, teardrop 110-180, parallel 180-290
    (heading at the fix)."""
    d = heading % 360
    if d >= 290 or d < 110:
        return "direct"
    if d < 180:
        return "teardrop"
    return "parallel"


class Plane:
    def __init__(self, ktas, hdg, wind=(0.0, 0.0)):
        self.v = ktas / 3600            # nm/s, true airspeed
        self.rate = turn_rate(ktas)
        self.hdg = hdg % 360
        self.x = self.y = 0.0
        self.t = 0.0
        self.wind = (wind[0] / 3600, wind[1] / 3600)
        self.path = [(0.0, 0.0)]

    def step(self, turn=0.0):
        self.hdg = (self.hdg + turn * self.rate * DT) % 360
        b = math.radians(self.hdg)
        self.x += (self.v * math.sin(b) + self.wind[0]) * DT
        self.y += (self.v * math.cos(b) + self.wind[1]) * DT
        self.t += DT
        self.path.append((self.x, self.y))

    def turn_to(self, target, direction):
        while abs(wrap(target - self.hdg)) > self.rate * DT:
            self.step(direction)
        self.hdg = target % 360

    def fly(self, seconds):
        for _ in range(round(seconds / DT)):
            self.step()

    def fly_outbound_timed(self, seconds, outbound_hdg=180.0):
        """Timing starts at abeam the fix (y <= 0) or now, whichever is later."""
        while self.y > 0:
            self.step()
        self.fly(seconds)

    def intercept_inbound(self, direction, limit=900):
        """Keep turning in `direction` until the 45-degree intercept heading, then track x = 0."""
        start = self.t
        committed = True
        while self.t - start < limit:
            want = -max(-45.0, min(45.0, 90.0 * self.x))
            err = wrap(want - self.hdg)
            if committed and abs(err) > self.rate * DT * 2:
                self.step(direction)
                continue
            committed = False
            self.step(max(-1.0, min(1.0, err / (self.rate * DT))))
            if self.y >= 0 and self.path[-2][1] < 0:
                return True
        return False

    def home_on_fix(self, direction, limit=900):
        start = self.t
        committed = True
        while self.t - start < limit:
            if math.hypot(self.x, self.y) < 0.05:
                return True
            want = math.degrees(math.atan2(-self.x, -self.y)) % 360
            err = wrap(want - self.hdg)
            if committed and abs(err) > 10:
                self.step(direction)
                continue
            committed = False
            self.step(max(-1.0, min(1.0, err / (self.rate * DT))))
        return False


def fly_entry(heading, ktas, leg):
    """Fly one entry from the moment the aircraft crosses the fix on `heading`."""
    p = Plane(ktas, heading)
    kind = entry_for(heading)
    if kind == "direct":
        p.turn_to(180, +1)                       # turn to follow the holding pattern
        p.fly_outbound_timed(leg)
        ok = p.intercept_inbound(+1)
    elif kind == "teardrop":
        p.turn_to(150, +1 if wrap(150 - p.hdg) > 0 else -1)   # 30 degrees into the holding side
        p.fly(leg - (p.t))                       # one minute from over the fix
        ok = p.intercept_inbound(+1)
    else:
        p.turn_to(180, +1 if wrap(180 - p.hdg) > 0 else -1)   # parallel, nonholding side
        p.fly(leg - p.t)
        ok = p.home_on_fix(-1)                   # toward the holding side, through > 180 deg
    return kind, p, ok


def entries(ktas, leg):
    rows = {}
    for h in range(360):
        kind, p, ok = fly_entry(h, ktas, leg)
        r = rows.setdefault(kind, {"n": 0, "times": [], "west": 0.0, "south": 0.0, "failed": 0})
        r["n"] += 1
        r["failed"] += not ok
        r["times"].append(p.t)
        r["west"] = max(r["west"], -min(x for x, _ in p.path))
        r["south"] = max(r["south"], -min(y for _, y in p.path))
    return rows


def racetrack(ktas, leg):
    rate = turn_rate(ktas)
    radius = (ktas / 3600) / math.radians(rate)
    return rate, bank_for(ktas, rate), radius, ktas / 3600 * leg


def wind_rollout(ktas, leg, cross_kt, k, track=False):
    """One circuit in a wind blowing from the west (toward the holding side). Inbound is
    crabbed onto the course; outbound applies k times that correction. Returns the
    cross-track distance (nm, + = holding side) when the inbound turn ends."""
    wca = math.degrees(math.asin(cross_kt / ktas))
    p = Plane(ktas, -wca, wind=(cross_kt, 0.0))
    p.turn_to(180 + k * wca, +1)
    p.fly_outbound_timed(leg)
    p.turn_to(-wca, +1)
    if track:
        return wca, p.x, p
    return wca, p.x


# (label, KIAS, altitude ft, leg s). The narrated cases come first; the rest show how much the
# altitude moves each number.
CASES = (("200 KIAS, sea level, 1 min", 200, 0, 60),
         ("200 KIAS, 6,000 ft, 1 min", 200, 6000, 60),
         ("265 KIAS, 15,000 ft, 1.5 min", 265, 15000, 90),
         ("265 KIAS, 20,000 ft, 1.5 min", 265, 20000, 90),
         ("265 KIAS, 25,000 ft, 1.5 min", 265, 25000, 90))
LOW, HIGH = CASES[0], CASES[2]


def best_k(ktas, leg, cross_kt=30):
    res = [(k, *wind_rollout(ktas, leg, cross_kt, k)) for k in (0, 1, 2, 2.5, 3, 3.5, 4)]
    best = None
    for (k0, _, x0), (k1, _, x1) in zip(res, res[1:]):
        if x0 > 0 >= x1:
            best = k0 + (k1 - k0) * x0 / (x0 - x1)
    return res, best


def main(out=None):
    print("== airspeed: KIAS taken as KCAS, standard day ==")
    for label, kias, alt, leg in CASES:
        print(f"{label}: {ktas_for(kias, alt):.1f} KTAS")
    print("sensitivity, 265 KIAS at 15,000 ft if KIAS is 5 kt off KCAS: "
          f"{ktas_for(260, 15000):.1f} / {ktas_for(270, 15000):.1f} KTAS")

    print("\n== racetrack geometry (no wind) ==")
    for label, kias, alt, leg in CASES:
        v = ktas_for(kias, alt)
        rate, bank, r, legnm = racetrack(v, leg)
        print(f"{label}: {v:.0f} KTAS, turn {rate:.2f} deg/s at {bank:.1f} deg bank, "
              f"radius {r:.2f} nm, width {2 * r:.2f} nm, straight legs {legnm:.2f} nm, "
              f"length {legnm + 2 * r:.2f} nm, one 180 turn {180 / rate:.0f} s, "
              f"one circuit {2 * leg + 360 / rate:.0f} s")

    for label, kias, alt, leg in (LOW, HIGH):
        v = ktas_for(kias, alt)
        print(f"\n== 360 entries, one per degree, {label} ({v:.0f} KTAS) ==")
        rows = entries(v, leg)
        for kind in ("direct", "teardrop", "parallel"):
            r = rows[kind]
            t = r["times"]
            print(f"{kind:9s} {r['n']:3d} aircraft  fix to fix {min(t):.0f}-{max(t):.0f} s "
                  f"(median {statistics.median(t):.0f})  farthest onto nonholding side "
                  f"{r['west']:.2f} nm  farthest past the fix along outbound {r['south']:.2f} nm"
                  f"  failed {r['failed']}")

    print(f"\n== the boundary: 5 degrees either side of each line ({LOW[0]}) ==")
    for h in (100, 105, 110, 115, 175, 180, 185, 285, 290, 295, 360):
        kind, p, _ = fly_entry(h, ktas_for(LOW[1], LOW[2]), LOW[3])
        print(f"heading {h:3d}: {kind:9s} fix to fix {p.t:.0f} s, "
              f"farthest onto nonholding side {-min(x for x, _ in p.path):.2f} nm")

    print("\n== wind: 30 kt from the west; rollout error after one circuit ==")
    for label, kias, alt, leg in CASES:
        v = ktas_for(kias, alt)
        res, best = best_k(v, leg)
        cells = "  ".join(f"k={k:g}: {x:+.2f}" for k, _, x in res)
        print(f"{label} ({v:.0f} KTAS), inbound correction {res[0][1]:.1f} deg: {cells}  "
              f"zero error at k = {best:.2f}")

    print("\n== the same, at other crosswinds: zero-error k at 15 / 30 / 60 kt ==")
    for label, kias, alt, leg in CASES:
        v = ktas_for(kias, alt)
        print(f"{label}: " + " / ".join(f"{best_k(v, leg, w)[1]:.2f}" for w in (15, 30, 60)))

    write_json(os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "holding.json"))
    if out:
        draw(out)


def mmss(seconds):
    s = round(seconds)
    return f"{s // 60}:{s % 60:02d}"


def track(p, every=4):
    """A path sampled once a second (every 4th 0.25 s step): [x, y] in nm."""
    return [[round(x, 4), round(y, 4)] for x, y in p.path[::every]]


def write_json(path):
    """Everything the scenes put on screen, from the same code that printed the run above.
    Times are seconds, distances nm, angles degrees; *_text keys are the exact strings shown."""
    data = {"source": "sim/holding.py", "dt": DT, "track_step_s": DT * 4, "cases": {}}
    for label, kias, alt, leg in CASES:
        v = ktas_for(kias, alt)
        rate, bank, r, legnm = racetrack(v, leg)
        res, best = best_k(v, leg)
        data["cases"][label] = {
            "kias": kias, "alt_ft": alt, "leg_s": leg, "ktas": v, "rate": rate, "bank": bank,
            "radius": r, "width": 2 * r, "straight": legnm, "length": legnm + 2 * r,
            "turn180_s": 180 / rate, "circuit_s": 2 * leg + 360 / rate,
            "wind30": {"wca": res[0][1], "rollout": {f"{k:g}": x for k, _, x in res},
                       "best_k": best},
            "best_k_by_wind": {str(w): best_k(v, leg, w)[1] for w in (15, 30, 60)}}
    low = data["cases"][LOW[0]]
    data["low"], data["high"] = LOW[0], HIGH[0]

    v = low["ktas"]
    arrivals, summary = [], {}
    for h in range(360):
        kind, p, ok = fly_entry(h, v, LOW[3])
        arrivals.append({"heading": h, "kind": kind, "t": p.t, "ok": ok,
                         "west": -min(x for x, _ in p.path), "track": track(p)})
        r = summary.setdefault(kind, {"n": 0, "times": []})
        r["n"] += 1
        r["times"].append(p.t)
    for kind, r in summary.items():
        t = r.pop("times")
        r.update(tmin=min(t), tmax=max(t), median=statistics.median(t),
                 range_text=f"{mmss(min(t))}–{mmss(max(t))}", median_text=mmss(statistics.median(t)))
    data["entries_low"] = {"case": LOW[0], "arrival_nm": 4.0, "arrival_s": 4.0 / (v / 3600),
                           "summary": summary, "tmax_all": max(a["t"] for a in arrivals),
                           "arrivals": arrivals}

    wv = ktas_for(LOW[1], LOW[2])
    ghosts = []
    for k in (0, 1, 2, 3, 4):
        wca, x, p = wind_rollout(wv, LOW[3], 30, k, track=True)
        ghosts.append({"k": k, "rollout": x, "t": p.t, "track": track(p)})
    data["wind_low"] = {"case": LOW[0], "wind_from": 270, "wind_kt": 30,
                        "wca": wca, "ghosts": ghosts}

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".tmp", "w") as f:
        json.dump(data, f, separators=(",", ":"))
    os.replace(path + ".tmp", path)


def draw(out):
    from raster import Canvas
    colours = {"direct": (70, 150, 255), "teardrop": (255, 170, 40), "parallel": (80, 220, 120)}
    size, span = 1080, 12.0
    scale = size / span
    cx, cy = size * 0.5, size * 0.42
    c = Canvas(size, size, bg=(12, 20, 40))
    for h in range(360):
        kind, p, _ = fly_entry(h, ktas_for(LOW[1], LOW[2]), LOW[3])
        b = math.radians(h)
        pts = [(-4 * math.sin(b), -4 * math.cos(b))] + p.path
        for (x0, y0), (x1, y1) in zip(pts[::4], pts[4::4]):
            c.line(cx + x0 * scale, cy - y0 * scale, cx + x1 * scale, cy - y1 * scale, colours[kind])
    c.line(cx, cy, cx, cy + 5 * scale, (230, 230, 230), 1)
    for bearing in (110, 290):                                 # the 70-degree line through the fix
        b = math.radians(bearing)
        c.line(cx, cy, cx + 5.5 * math.sin(b) * scale, cy - 5.5 * math.cos(b) * scale, (230, 230, 230))
    c.dot(cx, cy, (255, 60, 60), 6)
    c.save_ppm(out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
