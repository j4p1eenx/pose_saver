# Persistent Pose & Autonomous Localization

<p align="center">
  <strong>A ROS 2 package that enables a TurtleBot3 to recover its last known position and localize autonomously after a restart.</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros" />
  <img src="https://img.shields.io/badge/C++-17-blue?logo=cplusplus" />
  <img src="https://img.shields.io/badge/Navigation-Nav2-orange" />
  <img src="https://img.shields.io/badge/Database-Firebase-FFCA28?logo=firebase" />
  <img src="https://img.shields.io/badge/Docker-Containerized-2496ED?logo=docker" />
  <img src="https://img.shields.io/badge/Testing-GTest-lightgrey" />
</p>

---

## Overview

A mobile robot normally loses its localization estimate when it powers down. After restarting, it may require an operator to manually provide its initial position before it can navigate.

**`pose_saver_pkg` automates this recovery process.**

The package continuously saves the robot's confidently estimated pose to a Firebase Realtime Database. When the robot starts again, it retrieves the saved pose and verifies it against the current LiDAR scan and occupancy map before trusting it.

The system handles three situations:

1. **Normal restart:** The saved pose matches the environment, so the robot restores its position without requiring manual localization.
2. **Robot moved while powered off:** The saved pose does not match the current sensor data, so the system rejects it and performs global localization.
3. **No usable saved pose:** No valid record is available, or the database is unreachable. The robot falls back to global localization.

The system reports its localization state and either reaches `LOCALIZED` or terminates with `FAILED` and a reason.

---

## System Demonstration

The demonstration uses a simulated TurtleBot3 in Gazebo with ROS 2 Nav2 localization.

The complete workflow covers:

- Saving a confidently localized robot pose to Firebase.
- Retrieving the pose during startup.
- Verifying the saved pose using LiDAR-to-map matching.
- Restoring a valid pose through AMCL.
- Detecting an incorrect saved pose and triggering global localization.
- Reporting the final localization state.

<p align="center">
  <em>Demo video: startup recovery, normal restart, moved-robot recovery and global localization.</em>
</p>

<!-- Add your demo video or GIF here -->

---

## System Architecture

```text
                  ┌──────────────────────────────┐
                  │       Gazebo Simulation      │
                  │          TurtleBot3          │
                  └──────────────┬───────────────┘
                                 │
                        /scan, /odom, /tf
                                 │
                                 ▼
                  ┌──────────────────────────────┐
                  │       Nav2 Localization      │
                  │       Map Server + AMCL      │
                  └──────────────┬───────────────┘
                                 │
                            /amcl_pose
                                 │
                                 ▼
                  ┌──────────────────────────────┐
                  │        pose_saver_node       │
                  │                              │
                  │       Startup State Machine  │
                  │       Pose Validation        │
                  │       Scan-to-Map Matching   │
                  │       Recovery Localization  │
                  │       Pose Persistence       │
                  └──────────────┬───────────────┘
                                 │
                          HTTPS REST API
                                 │
                                 ▼
                  ┌──────────────────────────────┐
                  │    Firebase Realtime DB      │
                  │       Saved Robot Pose       │
                  └──────────────────────────────┘

                  pose_crud_cli
                         │
                         └────── HTTPS REST API
```

### Main Components

| Component | Responsibility |
|---|---|
| Gazebo | Simulates the TurtleBot3 and its environment. |
| Map Server | Provides the occupancy map used for localization. |
| AMCL | Estimates the robot's pose using LiDAR measurements and the map. |
| `pose_saver_node` | Implements the pose recovery state machine, validation, verification and saving logic. |
| Firebase Realtime Database | Stores the robot's latest valid pose across restarts. |
| `pose_crud_cli` | Provides Create, Read, Update and Delete operations for stored pose records. |
| Launch file | Starts the simulation, localization stack, RViz and custom package. |
| Docker | Provides a reproducible environment for building and running the project. |

### ROS Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/map` | Subscription | Occupancy map used for pose verification. |
| `/scan` | Subscription | Latest LiDAR measurements. |
| `/amcl_pose` | Subscription | AMCL pose estimate and covariance. |
| `/initialpose` | Publisher | Sends a verified saved pose to AMCL. |
| `/cmd_vel` | Publisher | Commands the recovery rotation. |
| `~/status` | Publisher | Reports the current localization state and details. |
| `/reinitialize_global_localization` | Service client | Requests global particle initialization. |
| `/request_nomotion_update` | Service client | Requests an AMCL update without robot movement. |
| TF | Transform lookup | Obtains the robot-to-LiDAR transform for scan projection. |

---

## Startup and Recovery Workflow

The package uses an explicit state machine to control startup and localization recovery.

```text
                 ┌──────────────┐
                 │  WAIT_READY  │
                 └──────┬───────┘
                        │
                        ▼
                 ┌──────────────┐
                 │  LOAD_POSE   │
                 └──────┬───────┘
                        │
                 Saved pose found?
                    /       \
                  Yes        No
                   │          │
                   ▼          │
             ┌──────────┐     │
             │ CONFIRM  │     │
             └────┬─────┘     │
                  │           │
             Pose valid?      │
              /     \        │
            Yes      No      │
             │        │      │
             ▼        └──┬───┘
       ┌───────────┐     ▼
       │ LOCALIZED │  ┌─────────────────┐
       └───────────┘  │ GLOBAL_LOCALIZE │
                      └────────┬────────┘
                               │
                       Converged in time?
                          /          \
                        Yes           No
                         │             │
                         ▼             ▼
                  ┌───────────┐  ┌──────────┐
                  │ LOCALIZED │  │  FAILED  │
                  └───────────┘  └──────────┘
```

### 1. Startup Readiness

Before attempting recovery, the node waits for the required ROS interfaces and data:

- Simulation clock.
- Occupancy map.
- LiDAR scan.
- Required TF transforms.
- AMCL localization interface.

A startup timeout prevents the node from proceeding with incomplete data.

### 2. Saved Pose Retrieval

The node retrieves the latest pose record from Firebase.

Before using it, the record is checked for:

- Valid numeric values.
- A valid orientation quaternion.
- Matching `map_id`.
- Acceptable record age.

If no usable record is available, the system proceeds to global localization.

### 3. Saved Pose Verification

A saved pose is only a hypothesis. It must agree with what the robot's sensors currently observe.

The node projects LiDAR endpoints into the occupancy map using the candidate pose and calculates the fraction of valid endpoints that fall on occupied map cells.

This produces a scan-match score:

- A high score indicates that the saved pose is consistent with the observed environment.
- A low score indicates that the saved pose is unlikely to represent the robot's current position.

The candidate is rejected if it fails the configured scan-match threshold.

### 4. AMCL Confirmation

After scan verification, the node publishes the candidate through `/initialpose`.

It then requests an AMCL no-motion update and checks whether the resulting estimate agrees with the saved pose.

Confirmation considers:

- Position uncertainty.
- Position drift.
- Angular drift.
- Scan-match score.
- Stability over a configured time interval.

Only after the confirmation checks pass does the system enter `LOCALIZED`.

### 5. Global Localization Recovery

If the saved pose cannot be trusted, the node requests global localization from AMCL.

The robot performs a controlled rotation to provide sensor observations from different orientations. AMCL uses these observations to refine its particle distribution and converge on the robot's actual position.

The node monitors convergence and either:

- Enters `LOCALIZED` when the estimate satisfies the configured conditions.
- Enters `FAILED` if localization does not converge within the timeout.

---

## Database Design

The project uses **Firebase Realtime Database** through its REST API.

HTTP requests are handled using `libcurl`, while JSON serialization and parsing are handled using `nlohmann/json`.

### Record Structure

Each robot has one record under:

```text
/poses/<robot_id>
```

Example:

```json
{
  "robot_id": "turtlebot3_burger_01",
  "map_id": "turtlebot3_world",
  "x": -0.628,
  "y": 0.068,
  "z": 0.0,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.588,
  "qw": 0.809,
  "cov_xx": 0.0083,
  "cov_yy": 0.0070,
  "cov_yaw": 0.0089,
  "stamp_unix": 1794451081.49
}
```

The position and orientation are stored in the map frame. The timestamp represents Unix time in seconds.

### Pose Saving Strategy

The node saves poses only when the robot is confidently localized.

The saving logic considers:

- Minimum movement since the previous save.
- A periodic heartbeat interval.
- AMCL position uncertainty.
- The current localization state.

This prevents an unreliable localization estimate from overwriting a previously valid pose.

Database operations are handled outside the main ROS executor using asynchronous startup retrieval and a background save worker. This prevents slow network requests from blocking ROS callbacks.

### Database Setup

To connect your own Firebase database:

1. Open the [Firebase Console](https://console.firebase.google.com/).
2. Create a Firebase project.
3. Navigate to **Build → Realtime Database**.
4. Create a database.
5. Configure the database rules for the `/poses` path.
6. Copy the database URL.
7. Provide the URL to the application through `POSE_DB_URL` or the package configuration.

Example:

```bash
export POSE_DB_URL="https://<project>-default-rtdb.<region>.firebasedatabase.app"
```

If authentication is enabled, provide the token through `POSE_DB_TOKEN`.

**Security note:** The demo configuration may use permissive database rules for ease of evaluation. A production deployment should use authenticated access and restrictive per-robot permissions. Never commit database credentials or tokens to the repository.

---

## Getting Started

### Prerequisites

- Ubuntu 24.04 LTS for the documented ROS 2 Jazzy setup.
- Docker Engine.
- Docker Compose plugin.
- An X11-compatible desktop session for Gazebo and RViz GUI rendering.
- Internet connectivity for Firebase communication.

The Docker environment provides the ROS 2 workspace and project dependencies, so a separate native ROS installation is not required on the host.

### Build the Docker Environment

From the repository directory:

```bash
docker compose build
```

The Docker Compose configuration builds the containerized ROS 2 environment.

### Run the Simulation

The following commands use the project's Docker helper script to simplify simulation startup and robot control.

**Run this setup in every terminal before proceeding:**

```bash
cd ~/ros2_ws/src/pose_saver_pkg
source scripts/docker_helpers.sh
xhost +local:docker
```

**Terminal 1 — Start the simulation**

```bash
dlaunch
```

This launches the configured Docker simulation environment, including Gazebo, Nav2 localization, RViz and the pose-saving node.

Wait for the simulation and localization components to initialize.

**Terminal 2 — Control the robot**

```bash
dteleop
```

This starts TurtleBot3 keyboard teleoperation.

Use the keyboard controls displayed in the terminal to move the robot around the environment.

For a better understanding of the complete workflow and expected behaviour, refer to the demonstration video above.

### Monitor Localization Status

In another terminal, run the setup commands above and then execute:

```bash
docker compose exec demo bash -lc \
"source /ros2_ws/install/setup.bash && ros2 topic echo /pose_saver_node/status --qos-durability transient_local --qos-reliability reliable"
```

The status message reports the current state, scenario, details and database connectivity.

---

## Reproducing the Recovery Scenarios

The following scenarios demonstrate the main requirements of the project.

### Scenario 1 — Normal Restart

**Objective:** Restore the robot's previous pose when it has not moved while powered off.

1. Start the simulation using `dlaunch`.
2. Open Terminal 2 and start keyboard teleoperation using `dteleop`.
3. Drive the robot to a different location.
4. Wait for the pose to be saved to Firebase.
5. Press `Ctrl+C` **only once** in the simulation terminal.
6. Allow approximately 3–4 seconds for the simulation to shut down.
7. Relaunch the simulation using:

```bash
dlaunch
```

**Expected behaviour:**

- The launch process retrieves the saved pose.
- The simulated robot spawns at the saved position.
- The LiDAR scan matches the occupancy map at that pose.
- AMCL confirms the restored estimate.
- The node enters `LOCALIZED` without performing a recovery spin.

The robot should spawn exactly where it was left.

Expected scenario label:

```text
1_NORMAL_RESTART
```

### Scenario 2 — Robot Moved While Powered Off

**Objective:** Detect that the saved pose no longer represents the robot's actual location.

First, stage a saved pose that differs from the robot's next spawn position:

```bash
dcli update turtlebot3_burger_01 0.0 1.0 1.57 turtlebot3_world
```

Now launch the robot at the default spawn position:

```bash
dlaunch x_pose:=-2.0 y_pose:=-0.5 yaw_pose:=0.0
```

**Expected behaviour:**

- The stored pose does not match the current LiDAR observations.
- The system rejects the candidate pose.
- AMCL global localization is triggered.
- The robot rotates to gather additional sensor observations.
- AMCL converges on the actual position.
- The node enters `LOCALIZED` and saves the recovered pose.

Expected scenario label:

```text
2_MOVED_WHILE_OFF
```

Global localization typically takes approximately 35–50 seconds in the configured simulation environment.

### Scenario 3 — Localization Failure

**Objective:** Verify that the system reports a clear failure when global localization cannot converge within the configured timeout.

First, reset the simulation environment:

```bash
dreset
```

Then delete the existing saved record:

```bash
dcli delete turtlebot3_burger_01
```

Launch using the failure-demo parameter configuration:

```bash
dlaunch params_file:=/ros2_ws/install/pose_saver_pkg/share/pose_saver_pkg/config/params_fail_demo.yaml
```

**Expected behaviour:**

- No usable saved pose is available.
- The system attempts global localization.
- The shortened timeout expires before convergence.
- The node reports a localization failure.
- The robot is stopped and the state becomes `FAILED`.

Expected result:

```text
LOCALIZATION FAILED ...
state=FAILED
```

---

## Additional Recovery Tests

The following tests cover the no-usable-pose cases.

### Test 3A — No Record Anywhere

Reset the environment and delete the robot's database record:

```bash
dreset
dcli delete turtlebot3_burger_01
```

Launch the simulation:

```bash
dlaunch
```

**Expected result:**

```text
detail: no saved pose (no_record)
```

The node proceeds to global localization and eventually reaches `LOCALIZED` if convergence succeeds.

### Test 3B — Database Unreachable and No Usable Record

Reset the environment:

```bash
dreset
```

Simulate an unavailable database by overriding the database URL:

```bash
POSE_DB_URL=http://127.0.0.1:1 dlaunch
```

**Expected result:**

```text
detail: db_unreachable
```

The system cannot retrieve a pose from Firebase and proceeds to global localization. It reaches `LOCALIZED` if convergence succeeds.

---

## CRUD Command-Line Interface

The package includes `pose_crud_cli`, a command-line utility for managing robot pose records in Firebase.

| Operation | Description |
|---|---|
| `create` | Creates a pose record if one does not already exist. |
| `read` | Retrieves a robot's saved pose. |
| `readall` | Retrieves all stored pose records. |
| `update` | Updates an existing robot pose. |
| `delete` | Deletes a robot's saved pose. |

Example commands:

```bash
# Create a record
dcli create turtlebot3_burger_01 -2.0 -0.5 0.0 turtlebot3_world

# Read a record
dcli read turtlebot3_burger_01

# Read all records
dcli readall

# Update a record
dcli update turtlebot3_burger_01 1.0 0.5 1.57 turtlebot3_world

# Delete a record
dcli delete turtlebot3_burger_01
```

Arguments:

```text
<robot_id> <x> <y> <yaw_rad> [map_id]
```

The CLI communicates with the same Firebase REST database used by the main node.

---

## Testing

The package includes unit tests for the core pose and scan-verification logic.

Run the tests inside the Docker environment:

```bash
docker compose exec demo bash -lc \
"source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon test --packages-select pose_saver_pkg --event-handlers console_direct+"
```

View the test summary:

```bash
docker compose exec demo bash -lc \
"source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon test-result --verbose"
```

The tests cover:

- JSON serialization and deserialization.
- Rejection of records with missing required fields.
- Validation of invalid pose values, including NaN values and invalid quaternions.
- Scan-match scoring for a correct candidate pose.
- Detection of displaced and rotated candidate poses.
- Handling of scans with insufficient valid beams.

---

## Key Design Decisions

### 1. LiDAR-to-Map Verification Instead of Trusting the Saved Pose

A stored pose cannot automatically be assumed to represent the robot's current location.

The system uses the current LiDAR scan and occupancy map to evaluate whether the environment is consistent with the candidate pose.

This provides an independent sensor-based check before the saved pose is accepted.

### 2. Two-Stage Pose Validation

The system performs two checks:

- **Stage 1:** Verify the saved pose using scan-to-map matching.
- **Stage 2:** Initialize AMCL with the candidate and confirm that its estimate agrees with the saved pose.

This prevents a candidate from being accepted based on a single check.

### 3. Save Only Confidently Localized Poses

The node saves only when it is in the `LOCALIZED` state and the uncertainty is within the configured limit.

This avoids replacing a good database record with an unreliable estimate.

### 4. Background Database Operations

Network requests can take significantly longer than normal ROS callbacks.

The project uses asynchronous startup retrieval and a background worker for saving poses, keeping database communication from blocking the ROS executor.

### 5. Explicit Failure Handling

The system does not silently assume that a saved pose is correct or that global localization will always succeed.

A terminal `FAILED` state reports the reason when recovery cannot complete within the configured timeout.

---

## Configuration

The main node parameters are defined in `config/params.yaml`.

| Parameter | Default | Purpose |
|---|---:|---|
| `robot_id` | `turtlebot3_burger_01` | Identifies the robot's database record. |
| `map_id` | `turtlebot3_world` | Rejects poses belonging to another map. |
| `save_interval_sec` | `2.0` | Interval at which saving conditions are evaluated. |
| `save_min_delta_m` | `0.02` | Minimum movement required to trigger a save. |
| `save_heartbeat_sec` | `30.0` | Periodic save interval, including when stationary. |
| `max_pose_age_sec` | `604800.0` | Maximum accepted age of a saved pose. |
| `scan_match_threshold` | `0.60` | Minimum scan-match score required for acceptance. |
| `max_sigma_m` | `0.25` | Maximum accepted position uncertainty. |
| `max_drift_m` | `0.30` | Maximum position difference during AMCL confirmation. |
| `max_drift_yaw_rad` | `0.40` | Maximum angular difference during confirmation. |
| `startup_timeout_sec` | `60.0` | Maximum time to wait for startup readiness. |
| `confirm_timeout_sec` | `10.0` | Maximum time for restored-pose confirmation. |
| `global_timeout_sec` | `90.0` | Maximum time allowed for global localization. |
| `spin_speed_rad_s` | `0.5` | Angular speed during recovery rotation. |

The database URL and optional authentication token can be supplied through environment variables.

---

## Repository Structure

```text
pose_saver/
│
├── config/
│   ├── params.yaml
│   └── params_fail_demo.yaml
│
├── include/
│   └── pose_saver_pkg/
│       ├── cloud_db_client.hpp
│       └── pose_logic.hpp
│
├── launch/
│   └── pose_saver_turtlebot.launch.py
│
├── scripts/
│   └── docker_helpers.sh
│
├── src/
│   ├── pose_saver_node.cpp
│   ├── cloud_db_client.cpp
│   └── pose_crud_cli.cpp
│
├── test/
│   └── test_pose_logic.cpp
│
├── Dockerfile
├── docker-compose.yml
├── package.xml
├── CMakeLists.txt
└── README.md
```

---

## Known Limitations

- **Network dependency:** The current implementation relies on Firebase for persistent pose retrieval. There is no local JSON cache for offline recovery. Ensure network connectivity when starting the simulation so the node can retrieve the saved pose from the cloud.
- **Global localization is relatively slow:** Recovery takes approximately 35–50 seconds in the configured world because the robot rotates in place.
- **Environment-specific thresholds:** Scan-match thresholds are tuned for the current TurtleBot3 simulation environment and should be evaluated for different maps and LiDAR configurations.
- **Small displacements:** Very small displacements may not be detected by scan matching; AMCL is expected to correct these.
- **Simulation-specific spawning:** Spawning at the saved pose is a simulation mechanism for reproducing a robot that has remained stationary while powered off. A physical robot cannot be teleported to its previous pose.
- **System clock dependency:** The record age check depends on the system clock being reliable.
- **Database security:** Database access should be secured with appropriate authentication and restrictive Firebase rules before production use.
- **No pose history:** The current implementation stores one latest pose record per robot rather than maintaining pose history.
- **Testing coverage:** Unit tests cover the core logic; full database and scenario integration tests are not automated.

---

## Future Improvements

- Add a local JSON cache to support offline pose recovery when Firebase is unreachable.
- Implement faster global localization recovery.
- Introduce adaptive scan-match thresholds for different environments.
- Add automated integration testing using ROS 2 launch testing.
- Add mock database testing for network failure scenarios.
- Introduce more structured ROS status messages and service interfaces.
- Strengthen database authentication and access control.
- Implement recovery retries after localization failure.

---

## Technologies Used

- **ROS 2 Jazzy** — Robot middleware and communication.
- **C++** — Main pose-saving and localization recovery logic.
- **Nav2 / AMCL** — Map-based robot localization.
- **Gazebo** — Robot simulation.
- **Firebase Realtime Database** — Cloud pose persistence.
- **libcurl** — HTTP communication with Firebase.
- **nlohmann/json** — JSON serialization and parsing.
- **Docker & Docker Compose** — Reproducible development and execution environment.
- **Colcon & GTest** — Build system and unit testing.

---

## Author

**Japleen**

GitHub: [@j4p1eenx](https://github.com/j4p1eenx)

Repository: [pose_saver](https://github.com/j4p1eenx/pose_saver)
