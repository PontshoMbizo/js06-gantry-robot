# Axis controllers for X, Y and Z (firmware 2.2 structure)

Companion to `controller_design.ipynb`. It derives the gains of the controller that already runs on the ESP32-C3, for each axis, from the plants identified in `report_method_analyses_new_tests.ipynb`, and checks them against CTRL-03, CTRL-04, CTRL-07, CTRL-08 and MECH-09 over the uncertainty of the plant. Every number below is taken from the executed notebook (`results_controllers/*.csv`). **Revision:** the Z axis was redesigned on the new Z recordings (`new_z/`, after the Z sensor was replaced); the earlier Z gains `[4.06, 1.633, 8, 3.34]` came from the faulty-sensor data and are obsolete. X and Y are unchanged.

## 1. Result

Paste these into the firmware configuration (`[Kp, Kd, Vlim, V0]`, the order of `gain_y` in the firmware notes):

```
gain_x = [0.35, 0.053, 6, 2.46]
gain_y = [0.637, 0.051, 8, 3.97]
gain_z = [1.7, 0.391, 5, 3.1]
```

| axis | Kp (V/mm) | Kd (V per mm/s) | Vlim (V) | V0 (V) | V0 - V_db (V) | status                                                                                                                            |
| ---- | --------- | --------------- | -------- | ------ | ------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| X    | 0.35      | 0.053           | 6        | 2.46   | 0.45          | meets CTRL-04, CTRL-07, CTRL-08 on the whole box; τ is only bounded                                                               |
| Y    | 0.637     | 0.051           | 8        | 3.97   | 0.88          | meets CTRL-04, CTRL-07, CTRL-08 and MECH-09 (27.8 s worst case, 2.2 s margin)                                                     |
| Z    | 1.7       | 0.391           | 5        | 3.1    | 0.80          | meets CTRL-07, CTRL-08; **CTRL-04 only for moves up to 7.5 mm** (see Section 6); identified on two clean voltages, limited to 5 V |

Units: $e$ in mm, $\hat v$ in mm/s, $u$ in V. $K_d$ is in V per (mm/s).

What the gains change on the bench:

* **V0 is higher than the value tried on the bench.** The identified dead band is 2.0 V (X), 3.09 V (Y) and 2.30 V (Z). The firmware note gives `gain_y = [0.3, 0.05, 7.0, 2.0]`, i.e. V0 = 2.0 V on Y, which is 1.1 V below the Y dead band. On the new Y plant that default only gets the carriage moving through the stall nudge, and in simulation 1295 of the 1944 plant and move cases take longer than 3 s (worst 50 mm move 7.2 s, against 1.85 s with the designed gains).
* **The speed term is moderate on every axis.** $K\,K_d$ is 0.5 on X and Y and 1.0 on Z, which is the damping the lag needs (X and Z have the largest τ bounds).
* **Vlim is a choice, not a result.** X 6 V (600 mm in at most 19 s); Z 5 V, the highest voltage at which the identified Z law holds (the 8 V runs are jerky and below the line). Section 7 gives the alternatives.

## 2. Plant and uncertainty (S domain)

Each axis is an affine speed law with a dead band and a first-order lag. With input $u$ (V) and carriage position $x$ (mm):

$$\tau\,\dot v+v=K\,(u-V_{db})_+,\qquad \dot x=v\quad\Rightarrow\quad G(s)=\frac{K}{s(\tau s+1)}\ \ (u>V_{db}).$$

The load is a disturbance and the identified model is uncertain (CTRL-03), so the gains are checked on a box. $K$ and $\tau$ are the edges of the identification envelope; the other edges are decisions.

| axis | K lo / nom / hi (mm/s per V) | τ lo / mid / hi (ms) | V_db lo / nom / hi (V) | count (mm) |
|---|---|---|---|---|
| X | 9.06 / 9.51 / 9.99 | 14 / 47 / 159 | 1.71 / 2.01 / 2.31 | 1.066 |
| Y | 8.31 / 9.89 / 11.86 | 15 / 28 / 54 | 2.89 / 3.09 / 3.59 | 1.066 |
| Z | 1.49 / 2.56 / 2.66 | 17 / 49 / 142 | 1.90 / 2.30 / 2.70 | 0.267 |

Also in the box (assumed, not identified): actuation delay 5, 15 and 30 ms, and an extra 0 or 0.3 V to break away from rest. The Y V_db edge is skewed upwards because Y at 3.5 V stalled in two of four runs. The Z V_db edge is ±0.4 V (the fits for the two directions agree within 0.07 V, but there are only two voltages). The Z K low edge, 1.49, is the 8 V runs, whose mean speed lies below the 3.5 and 5 V line. Above 5 V the Z plant is modelled with a knee (speed gain per volt 1.05 above 5 V), which matters only for Z gains with Vlim above 5 V. The box has 486 plants per move length.

## 3. S-domain design

With $V_0=V_{db}$ and an ideal speed signal, the law $u=K_p(D-x)-K_d\dot x$ gives

$$\tau\ddot x+(1+KK_d)\dot x+KK_p\,x=KK_p\,D,\qquad \omega_n=\sqrt{\tfrac{KK_p}{\tau}},\quad \zeta=\frac{1+KK_d}{2\sqrt{KK_p\tau}}.$$

Inverting: $K_p=\omega_n^2\tau/K$, $K_d=(2\zeta\omega_n\tau-1)/K$, with $K_d\ge 0$ because the drive is one-sided and the speed term can only slow the carriage. Two things follow from the structure:

* With $K_d=0$ the fastest response without overshoot is $\zeta=1$ at $K_p=1/(4\tau K)$. This is the proportional gain of the Lab 2 analysis (0.218, 0.825, 0.988 V/mm).
* A larger $\omega_n$ needs speed feedback, $1+KK_d=2\omega_n\tau$. The speed term is the only brake because the drive cannot reverse (CTRL-07).

CTRL-04 asks for the carriage to be inside one count of the target. For a 50 mm move the band is $\varepsilon=\ell/D$ = 2.1 %, which a critically damped step enters at $\omega_nt_s=y$ with $(1+y)e^{-y}=\varepsilon$, $y\approx5.76$. Taking $t_s$ = 2 s and never going below the $K_d=0$ critical frequency gives the start design (Figure D2):

| axis | ω_n (rad/s) | Kp (V/mm) | Kd (V per mm/s) | linear time to band |
|---|---|---|---|---|
| X | 4.16 | 0.218 | 0 | 1.39 s |
| Y | 16.3 | 0.825 | 0 | 0.35 s |
| Z (10 mm) | 5.06 | 0.988 | 0 | 1.09 s |

## 4. Z domain (5 ms)

The law runs every 5 ms with a zero-order hold. The speed term uses the measured speed (pulse timing), not a differenced position, so there is no derivative kick and no derivative to discretise: only the plant and the delay are discretised. With $a=e^{-T/\tau}$ and $d$ samples of delay,

$$\begin{bmatrix}x\\v\end{bmatrix}_{k+1}=\begin{bmatrix}1&\tau(1-a)\\0&a\end{bmatrix}\begin{bmatrix}x\\v\end{bmatrix}_k+K\begin{bmatrix}T-\tau(1-a)\\1-a\end{bmatrix}(u_{k-d}-V_{db}),\qquad u_k-V_{db}=K_p(D-x_k)-K_dv_k .$$

The plant lag pole maps to $a$ = 0.96 (X), 0.85 (Y), 0.95 (Z) at the design τ; the integrator to $z=1$. The slow poles sit within about 10 % of $z=1$, so the 5 ms sampling does not limit the design. The delay does: over the box and the 5 to 30 ms delays the first instability appears at $K_p$ = 3.2 (X), 2.9 (Y) and 12.1 V/mm (Z) for the start design, about 15, 3.5 and 12 times the start gains (Figure D3). A speed term tightens the delay limit: a large $K_d$ is unstable at the corner of high $K$, small τ and 30 ms delay (on Z, $K\,K_d$ = 3.3 gives $|z|$ = 1.11 there). The gain search therefore requires the linear loop to be stable at every corner and delay (largest $|z|$ < 0.999) before it accepts a candidate.

For the chosen gains the linear loop has real, negative poles at every corner (damping ratio at least 1.03) and the largest $|z|$ over the box and delays is 0.989 (X), 0.980 (Y), 0.992 (Z) (Figure D12).

## 5. The firmware law on the plant

The linear analysis cannot see the one-sided drive, the dead band, the saturation, the one-count quantisation or the stale speed estimate between pulses. `ctrl_sim.py` therefore simulates the law as described in the firmware notes, at 1 ms plant steps and a 5 ms controller:

| element | model |
|---|---|
| position feedback | pulse count $N$, $e=D-N\ell$; the start lies a random fraction of a count from a pulse edge |
| speed estimate | $\hat v=\ell/\max(T_{tick},\,t-t_{last})$; zero until the first pulse |
| law | $u=\mathrm{clamp}(V_0+K_pe-K_d\hat v+n,\,0,\,V_{lim})$, slew limit 50 V/s |
| stall nudge $n$ | grows at 2 V/s once 150 ms pass without a pulse, cleared by the next pulse |
| stop | brake when $e\le\ell$; the carriage then coasts with its own τ (no extra braking assumed, conservative) |
| plant | $\tau\dot v+v=K(u(t-d)-V_{db})_+$, from rest only after $u>V_{db}+\Delta V_b$ |

**These rules come from the firmware document, not from the source.** The 50 V/s slew in both directions, the nudge rule, the stop condition and the "no better than coasting" brake are assumptions to confirm against `05-software` before the gains are trusted. Removing the nudge or halving the slew limit changes few plants (Section 7), but the stop rule matters: the design relies on stopping within one count and on the coast after it.

**Metric.** Requirements are evaluated on the **feedback position** (pulse count × count length), which is what the firmware logs. The true carriage position can differ from it by up to one count, because the sensor has a single channel and a resolution of 1.07 mm, so a bench check of CTRL-07 has to use the logged counts.

## 6. Verification on the full box

The chosen (rounded) gains on 486 plants per move length, no design margin (settle 3 s, overshoot 1.1 mm, within one count):

| axis | move (mm) | worst settle (s) | median settle (s) | worst overshoot (mm) | failing plants |
|---|---|---|---|---|---|
| X | 3 / 10 / 25 / 50 | 0.90 / 1.33 / 1.84 / 2.53 | 0.52 / 0.96 / 1.42 / 2.09 | 1.05 / 1.05 / 1.06 / 1.03 | 0 |
| Y | 3 / 10 / 25 / 50 | 0.44 / 0.76 / 1.20 / 1.85 | 0.30 / 0.56 / 0.88 / 1.40 | 1.05 / 0.24 / 0 / 0 | 0 |
| Z | 3 / 5 / 10 | 1.64 / 2.18 / 3.64 | 0.99 / 1.35 / 2.17 | 0.26 / 0.25 / 0.26 | 0 / 0 / 67 late |
| Z | 25 / 50 | 7.98 / 15.26 | 4.67 / 8.75 | 0.27 / 0.27 | all late |

| requirement | X | Y | Z |
|---|---|---|---|
| CTRL-04 settle ≤ 3 s, within one count | 2.53 s at 50 mm, pass | 1.85 s at 50 mm, pass | 2.18 s at 5 mm, pass; 7.5 mm is the longest move inside 3 s |
| CTRL-07 overshoot ≤ 1.1 mm | 1.06 mm | 1.05 mm | 0.27 mm |
| CTRL-07 no reversal | u ≥ 0, speed ≥ 0, count never decreases | same | same |
| CTRL-08 timeout | worst 0.09 of the timeout | 0.08 | 0.27 |
| CTRL-08 overshoot inside the 5 mm soft-limit margin | 1.06 mm | 1.05 mm | 0.27 mm |
| MECH-09 1 m in 30 s | | 27.8 s worst, 21.7 s median, 16.8 s best | |

Long moves over the coarse box: X 600 mm in 14.4 to 19.1 s (median 16.6 s), Y 1000 mm in 16.8 to 27.8 s, Z 200 mm in 24.6 to 58.8 s (median 38.4 s).

**The no-reversal requirement is met by construction**: the law is clamped at 0 V, so the drive never reverses, and the simulation confirms it. The price is that an overshoot can never be corrected, so the gains must keep it inside the 1.1 mm. This also explains why V0 is not minimised: it sets the speed at which the carriage arrives and therefore the coast.

**Z and CTRL-04.** At 5 V the identified Z moves at most $K(V_{lim}-V_{db})$ = 6.9 mm/s, so a 50 mm move takes 7.3 s before any deceleration (15.3 s simulated on the worst plant, whose K is 1.49 and V_db 2.7 V). No gains can meet 3 s at 50 mm. On every plant of the box 3 s is met up to 7.5 mm (2.9 s, Figure D8). Either CTRL-04 for Z is restated for the moves Z actually makes (**suggested: up to 5 mm**, which gives 2.2 s worst case with margin), or the Z drive changes (a coarser lead screw; a Z limit above 5 V is not supported by the data, Section 7). Z moves of 1 mm overshoot by up to 0.33 mm (1.2 counts, inside the 1.1 mm of CTRL-07 but outside one count); moves below about 2 mm should be avoided.

## 7. Stress tests and the Vlim choice

Each assumption pushed beyond its edge in turn, on the design move and 10 mm (972 plants per case):

| case | X failing | Y failing | Z failing |
|---|---|---|---|
| reference (the box) | 0 | 0 | 0 |
| τ × 1.5 | 202 (overshoot up to 3.2 mm) | 2 | 56 (up to 0.50 mm, outside one count) |
| K × 0.8 | 1 | 0 | 0 |
| V_db + 0.3 V | 3 | 0 | 0 |
| V_db - 0.3 V | 34 | 0 | 0 |
| delay 50 ms | 13 | 5 | 0 |
| slew limit 20 V/s | 0 | 3 | 0 |
| no stall nudge | 0 | 0 | 0 |

The controllers are insensitive to the unmeasured K and delay, but **τ is the quantity that decides whether X and Z hold**: both have a τ that the data only bound (X 14 to 159 ms, Z 17 to 142 ms), and a 50 % larger τ breaks the overshoot requirement on a fifth of the X plants; on Z it pushes 6 % of the plants outside one count (overshoot at most 0.50 mm, still inside CTRL-07). Identifying τ on X and Z is therefore the most valuable bench measurement (Section 9).

Vlim options with the other gains held:

| axis | Vlim (V) | failing plants (all moves) | worst long-move time (s) | free coast at best-plant cruise (mm) |
|---|---|---|---|---|
| X | 4.5 | 109 | 31.2 | 4.4 |
| X | 5 | 0 | 25.6 | 5.2 |
| X | **6** | 0 | 19.1 | 6.8 |
| X | 8 | 0 | 12.9 | 10.0 |
| Y | 6 / 7 / **8** | 0 | 50.4 / 35.7 / 27.8 | 2.0 / 2.6 / 3.2 |
| Z | 4 / **5** / 6 / 8 | 165 / 0 / 0 / 0 | 103.1 / 58.8 / 45.4 / 31.5 | 0.8 / 1.2 / 1.5 / 2.3 |

* **Y needs 8 V.** At 7 V the worst traverse is 35.7 s, over the 30 s of MECH-09. Y at 8 V is also where the identification is best (K and τ resolved).
* **X at 5 V also passes** and costs 6 s on a 600 mm sweep. The free-coast column is the distance the carriage travels if the drive is cut at cruise speed with no braking at all (best-plant K, lowest V_db, τ at its upper edge). The soft limit is 5 mm inside the end, so at 6 V a failure of the switch path could carry the carriage past it. The firmware brakes, so the real distance is shorter; this is the argument for 5 V if the brake is weak on the bench.
* **Z at 5 V.** 4 V is not enough drive for the high end of the box. The 6 V and 8 V rows pass in the model, which represents the 8 V mean speed with a knee at 5 V, but the measured Z motion at 8 V alternates between about 6 and 14 mm/s within a run (stick-slip or a sensor artefact), so a higher limit cuts the worst 200 mm traverse from 59 s to 31 s at the cost of an operating point the identification has not validated.
* Motor noise on the sensors grows above about 5 V; the Y step tests at 8 V were usable (K and τ resolved), so 8 V is inside the identified range on Y.

## 8. Limitations (pending bench identification)

1. **The firmware rules in Section 5 are taken from the document, not from the source.** In particular the stop at $e\le\ell$, the nudge, the slew limit, the speed estimate before the first pulse, and braking no better than coasting.
2. **Z rests on two clean voltages.** K and V_db come from 3.5 and 5 V (two directions each, R² 0.997); the 8 V runs fall short of the line and are jerky, so Z is limited to 5 V and the 8 V behaviour is only a modelled knee. The new Z data replace the `new_tests` Z runs, whose sensor was faulty and undercounting. The Z gains, in particular V0 = 3.1 V, are pending a bench check, and a Z run at 6.5 V and an independent speed measurement at 8 V would show whether Z can be driven harder.
3. **τ is only bounded on X and Z.** The gains hold inside the identified τ range and fail at 1.5 times it (Section 7).
4. **The V_db range of Y is a decision** based on two stalled 3.5 V runs. A bench check should find the lowest V0 at which Y starts reliably from rest along the whole rail, and V0 = 3.97 V should be compared with it.
5. **The box edges for delay (5 to 30 ms) and breakaway (0.3 V) are assumed.**
6. **Quantisation.** A single-channel sensor of 1.07 mm resolution limits what can be verified: overshoots of one count are not distinguishable from rounding.
7. **Load.** The identified plant is taken as the loaded plant. If the Y moving mass changes, K and V_db move with it; K × 0.8 is covered, a larger change is not.

## 9. Next step on the bench (ATP-03)

1. Flash 2.2 with the three gain lines and run closed-loop moves of 3, 10, 25 and 50 mm in both directions on each axis (Z: 3, 5, 10 mm), logging the pulse counts.
2. Compare settle time, overshoot and the shape of u(t) with Figure D6, and note any move that ends outside one count.
3. Repeat Z open-loop at 6.5 V and log a short finely sampled transient on X and Z to resolve τ. Re-run Section 1 of the notebook with the new numbers; the rest follows.

## 10. Files and re-running

```
controller_design.ipynb      executed notebook (Sections 1 to 11)
controller_design.md         this file
ctrl_sim.py                  event-based simulator of the firmware law (vectorised)
inputs/                      identification tables used (copied from results_new_tests/)
figures_controllers/         D1 to D12
results_controllers/         gains_firmware_format.csv, gain_commands.txt, verification_by_move.csv,
                             stress_tests.csv, vlim_options.csv, long_moves.csv, requirement_check.csv,
                             settle_vs_move_length.csv, plant_boxes.csv, linear_check.csv,
                             firmware_default_vs_designed_Y.csv, search_coarse_*.csv, search_refined_*.csv
```

Run from this folder: `jupyter nbconvert --execute --to notebook --inplace controller_design.ipynb` (about 8 minutes on two cores; most of it is the gain search).

| figure | content |
|---|---|
| D1 | plant uncertainty boxes |
| D2 | S domain: root loci and linear time to one count |
| D3 | Z domain poles of the start design |
| D4 | linear model against the firmware law |
| D5 | gain search maps |
| D6 | chosen gains, nominal plant, several move lengths |
| D7 | every plant of the box, design move |
| D8 | settle time and overshoot against move length |
| D9 | long moves |
| D10 | stress tests |
| D11 | firmware default against designed gains (Y) |
| D12 | linear check of the chosen gains |
