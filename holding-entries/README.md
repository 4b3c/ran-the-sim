# holding-entries

The model behind the Ran the Sim video on holding-pattern entries: 360 aircraft arrive at a
holding fix, one per degree of heading, and each flies the FAA's recommended entry (direct,
teardrop or parallel). A second test flies one circuit in a crosswind to see how much outbound
correction brings the aircraft back onto the inbound course.

**This is a teaching model, not flight-planning software. Not for navigation.**

## What the model is

A point-mass aircraft flying in a flat plane at constant true airspeed, stepped every 0.25 s.
It turns at a fixed rate, flies each entry exactly as written, and makes no early corrections.
The frame is fixed: the holding fix at the origin, inbound course 360 (flying north to the fix),
right turns, so the holding side is east (x > 0). Distances are nautical miles, times seconds,
angles degrees.

Every number the video marks "model" comes from one run of `holding.py`.

## Parameters and their sources

The holding rules are from the FAA Aeronautical Information Manual (AIM) 5-3-8, Holding,
read 5 October 2026:
https://www.faa.gov/air_traffic/publications/atpubs/aim_html/chap5_section_3.html

| Rule / parameter | Value in the model | Source |
|---|---|---|
| Entry choice | By heading at the fix, relative to the inbound course: direct 290°–110°, teardrop 110°–180°, parallel 180°–290° | AIM figure "Holding Pattern Entry Procedures" ([image](https://www.faa.gov/air_traffic/publications/atpubs/aim_html/images/aim0503_At_Anchor2.png)): the 70° line through the fix. The sector sizes 180° / 110° / 70° are our arithmetic from the figure's angles. |
| Direct entry | Turn right to the outbound heading (180°), fly the outbound leg, turn right and intercept the inbound course | AIM 5-3-8, Direct Entry Procedure |
| Teardrop entry | Turn to 150° (30° into the holding side), fly until one minute after the fix, turn right and intercept | AIM 5-3-8, Teardrop Procedure |
| Parallel entry | Turn to 180° on the nonholding side, fly until one minute after the fix, turn left (toward the holding side) through more than 180° and home on the fix | AIM 5-3-8, Parallel Procedure |
| Leg time | 60 s at or below 14,000 ft MSL; 90 s above | AIM 5-3-8, inbound leg timing |
| Speed | 200 KIAS at sea level and 6,000 ft; 265 KIAS at 15,000, 20,000 and 25,000 ft | AIM 5-3-8, maximum holding airspeeds (200 KIAS up to 6,000 ft, 265 KIAS from 14,001 ft) |
| Turn rate | 3°/s or the rate at 30° bank, whichever needs less bank: rate = g·tan(30°)/V, V the true airspeed | AIM 5-3-8 (no RNAV guidance, no flight director) |
| Bank angle reported | tan(bank) = V·ω/g, a level coordinated turn | Standard physics; our derivation, not a quoted source |
| Wind correction | Inbound: crab angle asin(crosswind / airspeed). Outbound: k times that angle, k = 0 to 4 | AIM 5-3-8: "When outbound, triple the inbound drift correction" |
| Outbound timing | Direct entries and the wind test: from abeam the fix or the end of the outbound turn, whichever is later | AIM 5-3-8: timing "begins over/abeam the fix, whichever occurs later" |
| Atmosphere | Standard day, troposphere fit: T = 15.04 − 0.00649·h °C, p = 101.29·[(T+273.1)/288.08]^5.256 kPa, h in m | NASA Glenn, [Earth Atmosphere Model](https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html) |
| Speed of sound, γ | a = √(γ·R·T), γ = 1.4, R = 286.9 J/(kg·K) | NASA Glenn, [Isentropic Flow Equations](https://www.grc.nasa.gov/www/k-12/airplane/isentrop.html) Eq #2; R from the atmosphere model's 0.2869 |
| Indicated to true airspeed | Indicated taken as calibrated. The impact pressure is the one this speed makes at sea level; the Mach number aloft is the one giving the same impact pressure at the local pressure; TAS = M·a | [14 CFR 1.1](https://www.ecfr.gov/current/title-14/part-1/section-1.1) (calibrated airspeed equals true airspeed at sea level, standard atmosphere) and Isentropic Flow Eq #6 |
| Time step | 0.25 s | Our choice |

## Choices the AIM leaves open

- Indicated airspeed is taken as calibrated airspeed (no position or instrument error). The run
  prints the effect of a 5 kt error at 15,000 ft: about ±6 kt of true airspeed.
- Teardrop and parallel entries time their minute from over the fix.
- The return to the inbound course is a 45°-capped intercept (direct, teardrop), or homing on
  the fix (parallel; the AIM allows "return to the holding fix").
- No wind during the entries. The wind test uses one steady 30 kt crosswind from the west, and
  15 and 60 kt as checks.

## How to run it

```
python3 holding.py            # prints the run (compare with expected_output.txt)
python3 holding.py map.ppm    # also draws all 360 entries as a PPM image
```

It runs in about a second. Every run also writes `out/holding.json`: the racetrack
geometry, all 360 entry tracks (sampled once a second) and the wind-test tracks. The video's
scenes were drawn from that file.

## Limits

- Standard-day air only. On a hot day true airspeed, and so every turn radius, is larger.
- A flat, point-mass aircraft: no roll-in time, no altitude change, no pilot judgement, and no
  early correction. That is why direct entries arriving across the course overshoot it in the
  model by up to 1.41 nm at 200 KTAS (3.64 nm at 329 KTAS); a real pilot would lead the turn.
- No wind during entries, and only a steady wind in the wind test (no gusts, no wind change
  with altitude).
- Overhead fix crossing only. Some RNAV systems fly a "fly-by" turn before the fix, and many
  select the entry by ground track rather than heading; the model does neither.
- No protected-airspace template. The model says nothing about whether a track is inside the
  airspace protected for the hold.
- Standard (right-turn) patterns only. Nonstandard patterns mirror them.
