# Mechanical Design

Chassis, steering, and drivetrain of the robot, and an inventory of the CAD
files that describe them.

**CAD files:** [`CAD/`](../../../../CAD) (SolidWorks `.SLDPRT`/`.SLDASM` +
neutral `.STEP`/`.stp`)

---

## 1. Design goals

The mechanical platform is built around four goals stated in the top-level
README:

- compact chassis that fits within the WRO Future Engineers size limits,
- stable weight distribution for predictable handling,
- efficient steering geometry,
- modular mounts so the camera and LiDAR can be positioned and re-positioned.

---

## 2. Drivetrain

A **rear differential drive** driven by **two** N20 gear-motors mounted
mirror-imaged onto a common input, feeding a purpose-built differential gearset:

- **Motors:** two N20 gear-motors (`CAD/Motor_N20.SLDPRT`, instanced twice in
  the assembly). The pair was adopted to supply drive torque the earlier
  single-motor layout could not. Because they are mounted mirrored, their output
  shafts turn in opposite directions for the same electrical command, and the
  firmware accounts for the inversion; they are **not** one-motor-per-wheel.
- **Differential:** custom-designed gearset in
  [`CAD/DifferentialGear`](../../../../CAD/DifferentialGear):
  - `DifferentialGear__RingGear.SLDPRT` + `...RingGearCap.SLDPRT`
  - `DifferentialGear__Pinions.SLDPRT`
  - `DifferentialGear__SideGear.SLDPRT`
  - `Nut__M3_L27.5_.SLDPRT`

A differential lets the two driven wheels rotate at different speeds through a
turn, avoiding the scrubbing and heading disturbance a locked axle would cause.
The two motors join *before* the differential, at its input, so differential
action between the left and right wheels is preserved.

---

## 3. Steering

Front steering actuated by an **MG90S** micro servo. The steering linkage
knuckles are in [`CAD/MG90`](../../../../CAD/MG90):

- `MG90S_Servo.STEP` — the servo (source: GrabCAD, see `CAD/MG90/soures.txt`)
- `ShoulderUnidirectional.STEP`
- `Shoulder_BiDirectional.STEP`
- `Shoulder_FourDirected.STEP`

The multiple "shoulder" variants are steering-knuckle iterations — evidence of
design iteration on the steering geometry.

---

## 4. Sensor & component mounting

CAD models of the mounted hardware are included so the assembly and sensor
positions are fully described:

| Component | CAD file |
|-----------|----------|
| RPLIDAR S3 | `CAD/RPLIDAR_S3.SLDASM` / `.stp` |
| Raspberry Pi Camera Module 3 | `CAD/RaspburryPi_Camera_Module3.SLDPRT` |
| SparkFun OTOS (PAA5160E1, SEN-24904) | `CAD/SparkFunOpticalTrackingOdometrySensor_...SLDPRT` / `.step` |
| CN3903 (power module) | `CAD/CN3903.SLDASM` / `.STEP` |
| Bearings | `CAD/Bearing_MR128ZZ.SLDPRT`, `CAD/Bearing_MR63ZZ.SLDPRT` |
| Fasteners | `CAD/Nut__M3_Female_.SLDPRT`, `CAD/NutRing__3M.SLDPRT` |
| Test object (marker stand-in) | `CAD/Object/BOX_5x5x10__.SLDPRT` / `.STEP` |

---

## 5. Decision log (rubric criterion 1)

Already inferable from the CAD:

- **Differential drive** — allows different wheel speeds in turns, reducing tire
  scrub and heading upset compared with a fixed axle.
- **Servo-steered front axle (Ackermann-style)** with a dedicated linkage,
  rather than skid/differential steering — closer to a real "self-driving car"
  and matches the category's kinematics focus.
- **Custom differential + iterated steering shoulders** — the multiple shoulder
  versions show the geometry was revised, not adopted blindly.

The CAD directory contains SolidWorks and STEP source files, including several
steering-linkage iterations. It does not contain exported renders, STL files,
measured dimensions, mass, motor gearing data, torque/speed tests, or a weight
distribution record. Those values must be taken from the final assembled robot;
they are not inferred here from filenames or CAD metadata.
