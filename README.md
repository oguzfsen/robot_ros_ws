# Robot ROS 2 Workspace

This workspace contains the ROS 2 software for an autonomous agricultural robot. The code is under active development and is intended to run on the robot's Linux/Raspberry Pi environment.

The current architecture separates high-level robot state, autonomous mission execution, coordinated motion commands, and direct hardware control.

## Packages

| Package | Responsibility |
| --- | --- |
| `robot_core` | Runtime nodes, mission handling, motion coordination, hardware access, database access, and cloud communication |
| `robot_interfaces` | Custom messages, services, and the autonomous action definition |
| `robot_bringup` | Shared parameters and launch files |

## Main Runtime Architecture

```text
Cloud / operator
      |
      v
 signalr_node --------> database_node
      |                       |
      | /user/command         | MissionGet
      v                       v
   core_node ----------> auto_act_node
      |                    /robot/autonomous
      | manual Motion             |
      +---------------------------+
                  |
                  v
             motion_node
          /      |      |      \
         v       v      v       v
 relay_node steer_node hubmotor_node brake_node
```

`core_node` owns the main robot state. Manual control and emergency stop commands originate there. Autonomous driving is delegated to `auto_act_node` through an action.

Both manual and autonomous control publish one `Motion` message. `motion_node` converts that high-level command into synchronized relay, steering, hub-motor, and brake commands.

## Robot States

The main states are defined by `RobotState` in `robot_core/Tools.py`.

- `IDLE`: robot is waiting and no movement command is active.
- `MANUAL`: movement is controlled by the RC transmitter.
- `MISSION`: an autonomous mission action is active.
- `EMERGENCY`: movement is disarmed and stopped.
- `MALFUNTION`: a malfunction was reported and movement is disarmed and stopped. The spelling is retained because it is currently part of the code-level enum.

`core_node` publishes the current state on `/robot/state`.

## Autonomous Mission Flow

The current start flow is:

1. A `start_mission` command reaches `core_node` with a mission ID.
2. `core_node` accepts the request only while the robot is `IDLE` and no other mission request is active.
3. The raw mission JSON is requested from `database_node` through `MissionGet`.
4. `core_node` parses the JSON with `Mission` and performs the initial validity check.
5. The raw JSON is sent to `auto_act_node` as an `/robot/autonomous` action goal.
6. `auto_act_node` parses its own runtime `Mission` object and keeps the action alive until the mission completes or is canceled.
7. Mission progress is published separately on `/robot/mission_state` as `MissionState`.
8. When the action completes, `core_node` returns to `IDLE` if it is still in `MISSION` state.

An emergency or malfunction causes `core_node` to request cancellation of the active action. Cancellation is also remembered if the emergency happens while the goal request is still waiting for acceptance.

The action deliberately has no feedback fields. Runtime progress is distributed through `/robot/mission_state` instead.

### `Autonomous.action`

Goal:

- `mission_raw`: original mission JSON string

Result:

- `completed`: `true` only when the complete mission finishes successfully

Feedback:

- intentionally empty

### Autonomous Control Loop Status

`auto_act_node` has a timer-driven `update()` method running at `control_frequency`. The action lifecycle, cancellation, and mission-state publishing are present.

The actual autonomous control calculation is still intentionally empty. Topic subscriptions, waypoint advancement, steering calculation, and publication of `Motion` commands will be added incrementally inside `update()`.

## Motion Layer

`Motion.msg` is the single high-level movement interface used by both `core_node` and `auto_act_node`:

- `arm`
- `throttle`
- `steering`
- `reverse`

`motion_node` fans this command out to four hardware-facing nodes:

| Output | Node | Purpose |
| --- | --- | --- |
| `RobotRelay` | `relay_node` | Ignition, reverse, brake-related relays, back-energy, and alarm lamp |
| `Steer` | `steer_node` | Four-wheel steering targets and `stop_all` |
| `HubMotor` | `hubmotor_node` | Ackermann-corrected power values for four hub motors |
| `Brake` | `brake_node` | Brake actuator command |

When `Motion.arm` is false, `motion_node` sends `Steer.stop_all = true`. A zero/low throttle command also results in a brake command.

## Safety and Malfunction Flow

- `steer_node` and `hubmotor_node` monitor their critical hardware and publish `Malfunction` when their configured malfunction conditions are reached.
- Temporary I2C failures are retried; persistent failures are handled by the node-level malfunction logic.
- `core_node` listens for `Malfunction` and `Emergency`, changes the robot state, cancels the autonomous action, and repeatedly publishes a disarmed zero-motion command.
- `brake_node` listens for malfunction directly and can brake without waiting for a command from `core_node`.
- An external Arduino is intended to monitor the system heartbeat independently of the Raspberry Pi.

## Mission Storage and Cloud Flow

Mission definitions arrive through `signalr_node` and are stored by `database_node` rather than being transported as a dedicated ROS mission message.

```text
Cloud mission
    -> signalr_node
    -> /database/mission_set
    -> SQLite mission table
```

The mission table stores:

- `mission_id`
- `receive_date`
- `raw_json`
- `last_waypoint`

`MissionProgressSet` is available for persisting waypoint progress when progress storage is connected to autonomous execution.

Mission upload results are queued by `signalr_node` and included in the next outgoing `RobotStatus.responses` JSON string.

## GNSS and RTCM

`gnss_rover_node` publishes robot GNSS data and subscribes to `/station/rtcm`.

RTCM correction frames arrive through SignalR as base64 payloads, are decoded by `signalr_node`, and are immediately published to `/station/rtcm` for the GNSS rover.

The intended deployment requires GNSS to be present. Startup enforcement for a completely missing GNSS source is still planned.

## Main Nodes Launched by `robot.launch.py`

- `i2c_srv_node`
- `database_node`
- `rc_node`
- `brake_node`
- `relay_node`
- `encoder_node`
- `steer_node`
- `hubmotor_node`
- `compass_node`
- `gnss_rover_node`
- `signalr_node`
- `motion_node`
- `auto_act_node`
- `core_node`

Steering calibration is started separately with `steerzero.launch.py`.

## Configuration

General runtime parameters are stored in:

- `src/robot_bringup/config/params.yaml`
- `src/robot_bringup/config/database.yaml`

Shared topic and action names include:

- `/robot/motion`
- `/robot/malfunction`
- `/robot/emergency`
- `/robot/mission_state`
- `/robot/autonomous`
- `/database/mission_get`
- `/database/mission_set`
- `/database/mission_progress_set`

Hardware addresses, GPIO pins, limits, frequencies, and watchdog timeouts are also configured through these YAML files.

## Build and Launch

These commands must be run in a supported ROS 2 environment with the required system and Python dependencies installed:

```bash
cd ~/robot_ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup robot.launch.py
```

The current Windows development environment does not contain ROS 2 or `colcon`, so only static Python and configuration checks can be performed there.

## Important Remaining Work

- Fill `auto_act_node.update()` with sensor input processing, waypoint progression, steering/throttle calculation, and `Motion` publication.
- Connect mission progress persistence through `MissionProgressSet`.
- Decide and implement action cancellation when RC control changes the robot from `MISSION` to `MANUAL` or `IDLE`.
- Implement the pending `pause_mission`, `stop_mission`, and `return_home` commands.
- Implement the short full-power hub-motor shock strategy in `motion_node`, with the required safety conditions, for cases where commanded power does not produce movement.
- Enforce the required-GNSS startup policy.
- Complete end-to-end testing on the Raspberry Pi/robot hardware.
