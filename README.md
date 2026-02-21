# ARTUN UAV — TEKNOFEST 2023

Autonomous medical delivery drone software developed by **ARTUN UAV Team** for the TEKNOFEST 2023 competition. The system enables a UAV to autonomously pick up delivery tasks from a Firebase Realtime Database, fly to a target location (e.g. a hospital), wait for delivery confirmation, and return home — with live GPS tracking throughout the mission.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Firebase Data Structure](#firebase-data-structure)
- [File Structure](#file-structure)
- [Dependencies](#dependencies)
- [Connection Modes](#connection-modes)
- [Mission Flow](#mission-flow)
- [Configuration](#configuration)
- [Test Scripts](#test-scripts)
- [Key Functions Reference](#key-functions-reference)
- [License](#license)

---

## Overview

The drone receives a delivery target (GPS coordinates) from Firebase, autonomously flies to that location, lands, waits for a human operator to confirm delivery via Firebase, then returns to its home position and lands. The entire flight path and status are streamed back to Firebase in real time so a ground operator or mobile app can track the vehicle.

---

## System Architecture

```
┌─────────────────────┐         ┌──────────────────────────┐
│   Ground Operator   │ ──────► │  Firebase Realtime DB    │
│   / Mobile App      │ ◄────── │  (europe-west1 region)   │
└─────────────────────┘         └────────────┬─────────────┘
                                             │  REST / pyrebase4
                                ┌────────────▼─────────────┐
                                │   Companion Computer     │
                                │   (Raspberry Pi / PC)    │
                                │                          │
                                │   main.py                │
                                │   base_functions_*.py    │
                                │   firebase.py            │
                                └────────────┬─────────────┘
                                             │  MAVLink
                                             │  (serial / TCP)
                                ┌────────────▼─────────────┐
                                │   Flight Controller      │
                                │   (ArduCopter / GUIDED)  │
                                └──────────────────────────┘
```

---

## Firebase Data Structure

The system uses the following nodes in Firebase Realtime Database:

| Node | Field | Type | Direction | Description |
|---|---|---|---|---|
| `HospitalLocation` | `hospitalLocation_lat` | `float` | Read | Target latitude |
| `HospitalLocation` | `hospitalLocation_long` | `float` | Read | Target longitude |
| `Live` | `live_location_lat` | `float` | Write | Current drone latitude |
| `Live` | `live_location_long` | `float` | Write | Current drone longitude |
| `Delivery` | `estimated_delivery_time` | `string` | Write | ETA in `HH:MM` format |
| `Delivery` | `isDelivered` | `bool` | Read | Set to `True` by operator after delivery |

The ground operator writes the target coordinates to `HospitalLocation` before the mission, and sets `isDelivered = True` after the payload has been handed over. The drone writes its live GPS position and estimated arrival time continuously during flight.

---

## File Structure

```
uav-teknofest-competition-2023/
│
├── main.py                          # Main mission script (entry point)
├── base_functions_sitl.py           # Core flight functions — SITL/simulation mode
├── base_functions_vehicle.py        # Core flight functions — physical vehicle (UART)
├── firebase.py                      # Firebase initialisation and DB handle
│
└── vehicle_test_scripts/            # Standalone test scripts for pre-flight checks
    ├── base_functions_vehicle.py    # Copy of vehicle base functions for test context
    ├── takeoff.py                   # Simple arm → takeoff → land test
    └── nine_points_rtl.py           # 9-waypoint survey mission with RTL
```

---

## Dependencies

Install all Python dependencies with:

```bash
pip install dronekit pymavlink pyrebase4
```

| Package | Purpose |
|---|---|
| `dronekit` | High-level ArduPilot vehicle control (arm, takeoff, goto, mode switching) |
| `pymavlink` | Low-level MAVLink message construction (NED velocity / position targets) |
| `pyrebase4` | Firebase Realtime Database client for Python |
| `datetime` | Delivery ETA calculation |
| `math` | Haversine-approximation distance formula |
| `time` | Polling delays and timing |

---

## Connection Modes

The project ships with **two interchangeable base function files** that are otherwise identical except for how they connect to the flight controller:

### SITL / Simulation (`base_functions_sitl.py`)

Used for testing with ArduCopter SITL or a companion computer reachable over TCP (e.g. Parallels VM):

```python
connection_string = "tcp:10.211.55.6:5762"
vehicle = connect(connection_string, wait_ready=True)
```

### Physical Vehicle (`base_functions_vehicle.py`)

Used with a Raspberry Pi connected to the flight controller over UART:

```python
connection_string = "/dev/ttyAMA0"
baud_rate = 921600
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
```

> **Note:** `main.py` imports from `base_functions` (no suffix). Before running, symlink or rename the appropriate file:
> ```bash
> # For simulation
> ln -sf base_functions_sitl.py base_functions.py
>
> # For real vehicle
> ln -sf base_functions_vehicle.py base_functions.py
> ```

---

## Mission Flow

```
1. Wait until flight controller is armable (GPS lock, sensors healthy)
2. Record current GPS position as HOME
3. Push live location to Firebase
4. Poll Firebase for HospitalLocation
   └─ Loop until non-zero coordinates are received
5. Calculate and publish estimated delivery time (ETA) to Firebase
6. Arm motors → switch to GUIDED mode → take off to 10 m
7. Fly to hospital location (continuous proximity check, threshold: 2 m)
8. Land at hospital location
9. Push live location to Firebase
10. Poll Firebase for isDelivered == True (1-second interval)
11. Arm motors → take off to 10 m
12. Fly back to HOME location
13. Land at home
14. Push live location to Firebase
15. Clear all Firebase fields (reset for next mission)
```

---

## Configuration

Edit `main.py` to change the cruise altitude:

```python
altitude = 10   # metres (default: 10 m)
```

The delivery time estimator in `base_functions_sitl.py` assumes a constant airspeed:

```python
v = 5   # m/s — adjust to match the drone's actual cruise speed
```

The proximity threshold for waypoint arrival (in `advanced_goto`) is **2 metres**. This is set inside the function and can be tuned directly:

```python
if distance <= 2:
    break
```

---

## Test Scripts

Located in `vehicle_test_scripts/`. These are self-contained scripts for pre-flight validation and do **not** depend on Firebase.

### `takeoff.py`

Waits for GUIDED mode to be engaged by the GCS, then arms, takes off to 10 m, hovers for 10 seconds, and lands.

```bash
cd vehicle_test_scripts
python takeoff.py
```

### `nine_points_rtl.py`

Visits 9 pre-defined GPS waypoints at 20 m altitude in sequence, then triggers RTL (Return To Launch). Useful for validating navigation accuracy over the competition field.

```bash
cd vehicle_test_scripts
python nine_points_rtl.py
```

Waypoints are hardcoded around coordinates `39.48°N, 29.89°E` (competition test area). Edit the `location1`–`location9` variables to match your field.

---

## Key Functions Reference

| Function | File | Description |
|---|---|---|
| `arm_and_takeoff(targetAltitude)` | base_functions | Pre-arm checks → GUIDED mode → arm → takeoff, blocks until 95% of target altitude is reached |
| `advanced_goto(aLocation)` | base_functions | `simple_goto` wrapper that blocks until the vehicle is within 2 m of the target |
| `get_distance_metres(loc1, loc2)` | base_functions | Flat-earth approximation of ground distance in metres (from ArduPilot test suite) |
| `get_current_location()` | base_functions | Returns `vehicle.location.global_relative_frame` |
| `send_live_location()` | base_functions_sitl | Reads current GPS and writes `lat`/`lon` to `Firebase/Live` |
| `calculate_delivery_time(aLocation)` | base_functions_sitl | Computes ETA as `now + distance/5 m/s` and writes `HH:MM` string to Firebase |
| `mode_land()` | base_functions_sitl | Switches to LAND mode and blocks until altitude ≤ 1 m |
| `clear_firebase_data()` | base_functions_sitl | Resets all Firebase fields to zero / default after mission completion |
| `wait_until_armable()` | base_functions_sitl | Blocks the script until the autopilot reports `is_armable == True` |
| `send_ned_velocity(vx, vy, vz, duration)` | base_functions_vehicle | Sends `SET_POSITION_TARGET_LOCAL_NED` velocity commands for `duration` seconds |
| `goto_position_target_local_ned(n, e, d)` | base_functions_vehicle | Sends a direct NED position target MAVLink message |

---
