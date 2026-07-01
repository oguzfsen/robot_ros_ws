# robot_ros_ws

This README documents only the architecture decisions clarified in the current implementation work, so a future coding session can quickly continue from the same assumptions.

## Mission Flow

- A user mission is sent from the cloud side.
- The station side forwards the mission over serial as raw bytes without interpreting the payload.
- The robot-side `telemetry_node` receives that serial payload.
- The robot-side `telemetry_node` parses the incoming mission JSON and stores it by calling the database service `/database/mission_set`.
- When the database service replies, the result is queued as a response entry and appended to the next outgoing robot status message through `RobotStatus.responses`.

In short:

`cloud -> station telemetry -> serial bytes -> robot telemetry -> database service -> next robot status.responses`

## Station Compatibility Contract

The robot-side telemetry implementation must stay compatible with the station-side telemetry node in:

`/home/oguz/repos/station_ws/src/station_core/station_core/telemetry_node.py`

Important compatibility rules:

- The station forwards `/user/command`, `/user/mission`, and `/station/rtcm` to serial as raw bytes.
- The station expects robot status on serial as UTF-8 JSON text.
- The station does not parse or transform mission/command payloads before sending them to the robot.

Because of that, the robot-side telemetry node must:

- parse incoming serial data as a mixed stream of JSON packets and RTCM binary frames,
- publish RTCM frames immediately to `/station/rtcm`,
- publish command JSON to `/user/command`,
- handle mission JSON locally and save it through the database service,
- write outgoing robot status back to serial as JSON.

## RTCM Handling

`/station/rtcm` is required and must remain active.

Reason:

- `gnss_rover_node.py` subscribes to `/station/rtcm`.
- RTCM correction frames arrive through telemetry serial traffic.
- The robot-side telemetry node must detect RTCM frames (starting with `0xD3`) and publish them immediately to `/station/rtcm`.

## Mission Database Ownership

The mission table is owned by `database_node`, not by `telemetry_node`.

That means:

- `database_node` is responsible for creating/checking the SQLite schema at startup.
- `telemetry_node` must not create mission tables directly.
- `telemetry_node` stores missions only through ROS services.

## Mission-Related Services

Current mission service design is intentionally minimal.

### `MissionGet.srv`

Request:

- `mission_id`

Response:

- `mission_id`
- `receive_date`
- `raw_json`
- `last_waypoint`

### `MissionSet.srv`

Request:

- `mission_id`
- `receive_date`
- `raw_json`

Response:

- `success`
- `error`
- `data`

This service is for storing a newly received mission definition.

### `MissionProgressSet.srv`

Request:

- `mission_id`
- `last_waypoint`

Response:

- `success`
- `error`
- `data`

This service exists so the future autonomous execution flow can persist waypoint progress whenever a waypoint is passed or the mission is paused/interrupted.

## Mission Table

The mission table is intentionally small and stores only:

- `mission_id`
- `receive_date`
- `raw_json`
- `last_waypoint`

The database is the source of truth for:

- which mission version is stored,
- when it was received,
- the original mission JSON,
- the last persisted waypoint index.

## Mission Class

`src/robot_core/robot_core/Mission.py` is the robot-side helper for mission parsing and runtime waypoint handling.

Key points:

- It converts mission JSON into an ordered waypoint list.
- It provides public helpers such as target lookup, completion tracking, distance-to-target checks, and waypoint advancement.
- `target_sufficient_distance` is used to decide whether the robot is close enough to the active waypoint.
- It can produce a minimal storage record for the database flow.

The `__main__` section in `Mission.py` acts as an in-file usage showcase.

## `RobotStatus.responses`

`RobotStatus.responses` is a JSON string containing response entries for incoming requests such as mission uploads and, later, command execution results.

Current implemented behavior:

- mission upload result is added to `responses` with:
  - `request_id`
  - `success`
  - `error_text`
  - `response_text`

Example meaning:

- mission saved successfully,
- mission save failed with an error.

At the moment, mission responses are implemented; command execution responses are planned for later command-handling work.

## Removed Interface

`robot_interfaces/msg/Mission.msg` was removed because mission transport inside ROS is no longer needed for this design.

Mission handling now happens as:

- serial input arrives at `telemetry_node`,
- the mission is stored through the database service,
- later, `core_node` will query the mission from the database by `mission_id` when autonomous execution is requested.

## Future Core Flow

The intended future autonomous flow is:

- a start-mission command arrives with a `mission_id`,
- `core_node` requests the mission from `database_node`,
- `core_node` constructs/uses `Mission`,
- waypoint progress is periodically persisted with `MissionProgressSet`.
