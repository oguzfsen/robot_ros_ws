"""Surus mantigi testleri; ROS, executor veya robot donanimi calistirilmaz."""

import importlib.util
import csv
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.stop_steering = False
        self.coast = False
        self.autonomous = False


class Node:
    def __init__(self, *args, **kwargs):
        pass

    def has_parameter(self, name):
        return False

    def get_name(self):
        return 'auto_act_node'

    def declare_parameter(self, name, default):
        return default

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 0))

    def get_logger(self):
        return SimpleNamespace(info=lambda text: None, warning=lambda text: None, error=lambda text: None)

    def create_subscription(self, *args):
        pass

    def create_publisher(self, *args):
        return Publisher()

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)


@pytest.fixture
def node(monkeypatch, tmp_path):
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, default: str(tmp_path) if name == 'drive_log_directory' else default)
    modules = {
        'rclpy': {'ok': lambda: True},
        'rclpy.node': {'Node': Node},
        'rclpy.parameter': {'Parameter': type('Parameter', (), {})},
        'rclpy.action': {'ActionServer': lambda *args, **kwargs: None, 'CancelResponse': SimpleNamespace(ACCEPT=1), 'GoalResponse': SimpleNamespace(ACCEPT=1, REJECT=2)},
        'rclpy.callback_groups': {'ReentrantCallbackGroup': lambda: None},
        'rclpy.executors': {'MultiThreadedExecutor': object},
        'std_msgs': {},
        'std_msgs.msg': {'Float32': Message, 'Int32': Message},
        'robot_interfaces': {},
        'robot_interfaces.action': {'Autonomous': SimpleNamespace(Goal=Message, Result=Message)},
        'robot_interfaces.msg': {'MissionState': Message, 'Gnss': Message, 'Compass': Message, 'Motion': Message, 'Emergency': Message, 'RcInput': Message},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    source = Path(__file__).parents[1] / 'robot_core'
    for name in ('Tools', 'Mission', 'auto_act_node'):
        spec = importlib.util.spec_from_file_location('robot_core.' + name, source / (name + '.py'))
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, spec.name, module)
        spec.loader.exec_module(module)

    driver = module.AutoActNode()
    # Mevcut geometri/faz testlerinde sureyi atla; fren suresi ayri test edilir.
    driver.BRAKE_SETTLE_TIME = 0.0
    driver.BRAKE_RELEASE_TIME = 0.0
    driver.rc_connected = True
    driver.latitude = 0.0
    driver.longitude = 0.0
    driver.compass_heading = 0.0
    driver.speed = 0.0
    yield driver
    driver.autonomous_logger.close()


def waypoint(node, east, north):
    return SimpleNamespace(latitude=math.degrees(north / node.EARTH_RADIUS), longitude=math.degrees(east / node.EARTH_RADIUS))


def mission(node, points):
    coordinates = []
    for east, north in points:
        point = waypoint(node, east, north)
        coordinates.append([point.latitude, point.longitude])
    raw = json.dumps({'ID': 42, 'home': coordinates[0], 'yollar': [{'etiket': 'test', 'konumlar': coordinates[1:]}], 'rotalar': [{'etiket': 'test', 'overlayType': 'polyline'}]})
    node.mission = sys.modules['robot_core.Mission'].Mission(raw)
    return node.mission


def position(node, east, north):
    point = waypoint(node, east, north)
    node.latitude = point.latitude
    node.longitude = point.longitude


def test_initialization_and_no_commands_without_mission(node):
    assert node.topic_motion == '/robot/motion'
    assert node.control_timer.period == pytest.approx(0.1)
    node.update()
    assert not node.motion_publisher.messages


def test_waits_for_first_sensor_values(node):
    mission(node, [(0, 10)])
    node.compass_heading = None
    node.update()
    message = node.motion_publisher.messages[-1]
    assert message.arm and not message.stop_steering
    assert message.throttle == message.steering == 0


@pytest.mark.parametrize('distance', [0.0, 1.0, 2.0, 3.0, 4.0])
def test_three_metre_triangle_and_projection(node, distance):
    node.previous_waypoint = waypoint(node, distance, -10)
    node.current_target = waypoint(node, distance, 10)
    east, north = node.line_point()
    assert east == pytest.approx(distance, abs=1e-7)
    if distance <= 3.0:
        assert math.hypot(east, north) == pytest.approx(3.0)
        assert north >= 0.0
    else:
        assert north == pytest.approx(0.0)


def test_line_point_does_not_go_past_endpoint(node):
    node.previous_waypoint = waypoint(node, 0, -10)
    node.current_target = waypoint(node, 0, 1)
    assert node.line_point() == pytest.approx((0, 1))
    node.previous_waypoint = node.current_target
    assert node.line_point() == pytest.approx((0, 1))


@pytest.mark.parametrize('heading, steering', [(0, 0), (20, -10), (340, 10), (60, -30), (300, 30)])
def test_heading_sign_wrap_and_four_wheel_steering(node, heading, steering):
    node.compass_heading = heading
    node.speed = 0.4
    node.go_to_point((0, 10), 1.0)
    assert node.steering == steering
    assert node.arm and node.throttle > 0


def test_current_steering_predicts_next_heading(node):
    node.compass_heading = 340
    node.speed = 0.6
    node.average_steering = 30
    node.go_to_point((0, 10), 1.0)
    assert 0 < node.steering < 10



def test_last_five_accelerations_and_speed_limit(node):
    for speed in [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7]:
        node.speed = speed
        node.update_acceleration()
    assert len(node.accelerations) == 5
    assert node.acceleration == pytest.approx(1.2)
    node.drive(5.0, 100, False)
    assert node.target_speed == 1.0
    assert node.throttle < node.CRUISE_THROTTLE and node.coast
    node.speed = 0.95
    node.drive(1.0, 100, False)
    assert node.throttle < node.CRUISE_THROTTLE and node.coast  # Ivme artinca gaz azalir.
    assert node.steering == 30
    node.speed = 1.0
    node.acceleration = -0.2
    node.drive(0.6, 0, False)
    assert node.throttle == 0 and node.coast


def test_speed_control_reacts_to_acceleration_and_target(node):
    node.speed = 0.7
    node.acceleration = 0.0
    node.drive(1.0, 0, False)
    rising_throttle = node.throttle
    assert rising_throttle > 0
    node.acceleration = 0.4
    node.drive(0.6, 0, False)
    assert node.throttle < rising_throttle


def test_half_metre_acceptance_and_phase_transitions(node):
    route = mission(node, [(0, 10), (0, 20)])
    node.update()
    assert node.phase == node.TO_START
    position(node, 0, 8.1)
    node.update()
    assert node.phase == node.CLOSE_TO_POINT and node.target_speed == 0.6
    position(node, 0, 9.4)
    node.update()
    assert route.index == 0
    position(node, 0, 9.6)
    node.update()
    assert route.index == 1 and node.phase == node.ON_LINE


def test_visible_next_point_starts_turn_inside_acceptance_circle(node):
    route = mission(node, [(0, 1), (3, 10)])
    node.update()
    target = node.point_vector(route.get_current_target())
    turn = node.close_point()
    assert turn[0] > target[0]
    assert math.hypot(turn[0] - target[0], turn[1] - target[1]) <= 0.25
    assert route.index == 0


def test_invisible_next_point_stops_at_waypoint_before_reverse(node):
    route = mission(node, [(0, 1), (0, -10)])
    node.speed = 0.4
    node.update()
    assert node.close_point() == pytest.approx((0, 1))
    assert route.index == 0 and node.target_speed == 0.6
    position(node, 0, 0.7)
    node.update()
    assert route.index == 1 and node.phase == node.REVERSE_MANEUVER
    assert node.throttle == 0 and not node.reverse
    node.update()
    assert not node.reverse  # Hala hareketli; geri roleye gecilemez.
    node.speed = 0.0
    node.update()
    assert node.reverse and node.throttle > 0 and node.reverse_count == 1


def test_maneuver_changes_direction_only_after_stopping(node):
    mission(node, [(0, -10)])
    node.update()
    node.update()
    assert node.reverse and node.reverse_count == 1
    position(node, 0, -1.8)
    node.speed = 0.6
    node.update()
    assert node.throttle == 0 and node.reverse and node.steering == 0
    assert node.maneuver_stopping
    node.update()
    assert node.reverse
    node.speed = 0.0
    node.update()
    assert not node.reverse and node.throttle > 0
    assert node.reverse_count == 1


def test_maneuver_exits_after_target_enters_front_view(node):
    mission(node, [(0, -10)])
    node.update()
    node.update()
    node.speed = 0.4
    node.compass_heading = 150
    node.update()
    assert node.throttle == 0 and node.reverse
    node.speed = 0.0
    node.update()
    assert node.phase == node.TO_START and node.reverse
    node.update()
    assert node.throttle > 0 and not node.reverse


def test_five_reverse_legs_then_stop_without_false_completion(node):
    route = mission(node, [(0, -100)])
    node.update()
    for leg in range(10):
        node.speed = 0.0
        node.update()
        assert node.phase == node.REVERSE_MANEUVER
        assert node.reverse == (leg % 2 == 0)
        assert node.reverse_count == leg // 2 + 1
        position(node, 0, -2 if leg % 2 == 0 else 0)
        node.speed = 0.2
        node.update()
        assert node.throttle == 0
    node.speed = 0.0
    node.update()
    assert node.phase == node.THE_END and node.reverse_count == 5
    assert not route.is_completed
    assert not node.arm and not node.reverse
    node.update()
    assert node.throttle == 0 and not route.is_completed
    row = log_rows(node)[-1]
    assert row['event'] == 'error' and row['error'] == node.mission_error
    assert len(node.emergency_publisher.messages) == 1
    assert node.emergency_publisher.messages[0].emergency
    assert node.emergency_publisher.messages[0].node_name == 'auto_act_node'
    assert node.emergency_publisher.messages[0].error == node.mission_error


def test_each_new_turn_resets_reverse_count(node):
    node.reverse_count = 5
    node.go_to_point((0, -10), 0.6)
    assert node.phase == node.REVERSE_MANEUVER
    assert node.reverse_count == 0


def test_final_waypoint_stops_before_marking_mission_complete(node):
    route = mission(node, [(0, 0), (0, 10)])
    route.set_index(route.waypoint_count - 1)
    node.speed = 0.3
    node.update()
    assert not route.is_completed and node.phase == node.THE_END
    assert node.throttle == 0 and node.steering == 0
    node.speed = 0.0
    node.update()
    assert route.is_completed
    message = node.motion_publisher.messages[-1]
    assert not message.arm and not message.reverse
    assert message.throttle == message.steering == 0
    assert node.mission_state_publisher.messages[-1].completion_rate == 100


def test_cancel_and_new_mission_reset_drive_state(node):
    mission(node, [(0, 10)])
    node.update()
    node.reverse_count = 4
    node.mission = None
    node.update()
    message = node.motion_publisher.messages[-1]
    assert not message.arm and message.throttle == 0
    count = len(node.motion_publisher.messages)
    node.update()
    assert len(node.motion_publisher.messages) == count
    mission(node, [(0, 20)])
    node.update()
    assert node.reverse_count == 0 and node.throttle > 0


def test_cancel_during_update_cannot_publish_old_drive_command(node):
    mission(node, [(0, 10)])
    node.update()
    node.mission = None
    node.publish_motion()
    message = node.motion_publisher.messages[-1]
    assert not message.arm and not message.reverse
    assert message.throttle == message.steering == 0


def test_action_returns_success_after_stop(node, monkeypatch):
    route = mission(node, [(0, 0)])
    node.mission = None
    states = []
    handle = SimpleNamespace(request=SimpleNamespace(mission_raw=route.raw_json), is_cancel_requested=False, succeed=lambda: states.append('succeeded'), abort=lambda: states.append('aborted'))
    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', lambda period: node.update())
    result = node.execute_callback(handle)
    assert result.completed and states == ['succeeded']
    assert not node.motion_publisher.messages[-1].arm
    assert node.mission is None and not node.goal_active


def test_actual_waypoint_must_be_visible_even_if_line_point_is_visible(node):
    mission(node, [(0, 0.4), (10 * math.sin(math.radians(61)), 10 * math.cos(math.radians(61)))])
    node.update()
    assert abs(node.heading_error(node.line_point())) < 60
    assert abs(node.heading_error(node.point_vector(node.current_target))) > 60
    assert node.phase == node.REVERSE_MANEUVER and node.throttle == 0
    node.update()
    assert node.reverse and node.reverse_count == 1


@pytest.mark.parametrize('points', [[(0, 0), (0, 20), (20, 20), (20, 0)], [(0, 0), (0, 20), (3, 20), (3, 0)], [(10, 10), (20, 20)]])
def test_route_completes_with_simple_four_wheel_motion_model(node, control_time, points):
    """Geometri/faz entegrasyonu; bu basit model gercek robot hiz garantisi degildir."""
    route = mission(node, points)
    node.BRAKE_SETTLE_TIME = type(node).BRAKE_SETTLE_TIME
    node.BRAKE_RELEASE_TIME = type(node).BRAKE_RELEASE_TIME
    east = north = heading = speed = angle = 0.0
    for frame in range(3000):
        control_time[0] += 0.1
        position(node, east, north)
        node.compass_heading = math.degrees(heading) % 360.0
        node.speed = speed
        node.average_steering = angle
        node.update()
        if route.is_completed:
            break
        if node.arm and not node.stop_steering:
            angle += max(-6.0, min(6.0, node.steering - angle))
        if node.throttle < node.throttle_deadband and not node.coast:
            speed = max(0.0, speed - 0.1)
        else:
            speed += (node.throttle / 35.0 - speed) * 0.1
        assert speed <= 1.0
        signed_speed = -speed if node.reverse else speed
        heading += 2.0 * signed_speed * math.tan(math.radians(angle)) / node.wheelbase * 0.1
        east += signed_speed * math.sin(heading) * 0.1
        north += signed_speed * math.cos(heading) * 0.1
    assert route.is_completed
    assert not node.arm and not node.reverse
    assert node.throttle == node.steering == 0


def log_rows(node):
    with node.autonomous_logger._writer.path.open() as stream:
        return list(csv.DictReader(stream))


def test_log_contains_measurements_and_exact_published_motion(node):
    mission(node, [(0, 10), (10, 20)])
    node.speed = 0.4
    node.accuracy = 0.02
    node.fix_ok = True
    node.average_steering = 4.5
    node.update()
    message = node.motion_publisher.messages[-1]
    row = log_rows(node)[-1]
    assert row['event'] == 'update' and row['phase'] == node.TO_START
    assert row['mission_id'] == '42' and row['waypoint_index'] == '0'
    assert float(row['target_distance_m']) == pytest.approx(10, abs=0.1)
    assert float(row['aim_north_m']) == pytest.approx(10)
    assert float(row['heading_error_deg']) == pytest.approx(0)
    assert float(row['speed_m_s']) == 0.4 and float(row['target_speed_m_s']) == 0.8
    assert float(row['gnss_accuracy_m']) == 0.02 and row['gnss_fix_ok'] == 'True'
    assert float(row['average_steering_deg']) == 4.5
    assert row['arm'] == str(message.arm) and row['reverse'] == str(message.reverse)
    assert int(row['throttle']) == message.throttle and int(row['steering_deg']) == message.steering
    assert row['cross_track_error_m'] == ''


def test_log_contains_signed_line_error(node):
    route = mission(node, [(2, -10), (2, 10)])
    route.set_index(1)
    node.update()
    row = log_rows(node)[-1]
    assert float(row['cross_track_error_m']) == pytest.approx(2.0)
    assert math.hypot(float(row['aim_east_m']), float(row['aim_north_m'])) == pytest.approx(3.0)


def test_waiting_for_data_logs_blanks_instead_of_invented_measurements(node):
    mission(node, [(0, 10)])
    node.speed = None
    node.latitude = None
    node.update()
    row = log_rows(node)[-1]
    assert row['event'] == 'waiting_for_data'
    assert row['latitude'] == row['speed_m_s'] == row['predicted_speed_m_s'] == row['target_distance_m'] == ''
    assert row['target_latitude'] != ''
    assert row['arm'] == 'True' and row['stop_steering'] == 'False' and row['throttle'] == '0'


def test_cancel_closes_log_and_next_mission_gets_another_file(node):
    mission(node, [(0, -10)])
    node.update()
    node.update()
    first_path = node.autonomous_logger._writer.path
    node.mission = None
    node.update()
    row = log_rows(node)[-1]
    assert row['event'] == 'mission_ended' and row['reverse_count'] == '1'
    assert row['arm'] == row['reverse'] == 'False' and row['throttle'] == '0'
    assert not node.autonomous_logger._writer.is_open
    contents = first_path.read_text()
    mission(node, [(0, 20)])
    node.update()
    assert node.autonomous_logger._writer.path != first_path
    assert first_path.read_text() == contents
    rows = log_rows(node)
    assert len(rows) == 1 and rows[0]['reverse_count'] == '0'


def test_completion_logs_final_zero_command_and_closes_file(node):
    mission(node, [(0, 0)])
    node.update()
    row = log_rows(node)[-1]
    assert row['event'] == 'completed' and row['completion_rate'] == '100'
    assert row['arm'] == row['reverse'] == 'False'
    assert row['throttle'] == row['steering_deg'] == '0'
    assert not node.autonomous_logger._writer.is_open


def test_log_write_failure_does_not_interrupt_driving(node, monkeypatch):
    mission(node, [(0, 10)])
    node.update()

    def fail_write(values):
        raise OSError('disk full')

    monkeypatch.setattr(node.autonomous_logger._writer, 'write', fail_write)
    node.update()
    assert node.throttle > 0
    assert not node.autonomous_logger._writer.is_open
    node.update()
    assert node.throttle > 0


def test_shutdown_records_stop_and_closes_log(node, monkeypatch):
    mission(node, [(0, 10)])
    node.update()
    node.action_server = SimpleNamespace(destroy=lambda: None)
    monkeypatch.setattr(Node, 'destroy_node', lambda self: True, raising=False)
    assert node.destroy_node()
    row = log_rows(node)[-1]
    assert row['event'] == 'shutdown'
    assert row['arm'] == row['reverse'] == 'False'
    assert row['throttle'] == row['steering_deg'] == '0'
    assert not node.autonomous_logger._writer.is_open


def test_completion_event_survives_action_cleanup_between_callbacks(node):
    route = mission(node, [(0, 10)])
    node.update()
    route.mark_completed()
    node.mission = None
    node.log_drive()
    assert log_rows(node)[-1]['event'] == 'completed'


class Goal:
    def __init__(self, node, raw):
        self.node = node
        self.request = SimpleNamespace(mission_raw=raw)
        self.is_cancel_requested = False
        self.is_active = True
        self.states = []

    def finish(self, state):
        message = self.node.motion_publisher.messages[-1]
        assert not message.arm and not message.reverse and not message.coast
        assert message.throttle == message.steering == 0
        assert self.node.mission is None
        assert not self.node.autonomous_logger._writer.is_open
        self.is_active = False
        self.states.append(state)

    def succeed(self):
        self.finish('succeeded')

    def abort(self):
        self.finish('aborted')

    def canceled(self):
        self.finish('canceled')


@pytest.mark.parametrize('core_requests_cancel', [False, True])
def test_maneuver_limit_ends_action_with_or_without_core_cancel(node, monkeypatch, core_requests_cancel):
    route = mission(node, [(0, -100)])
    node.mission = None
    goal = Goal(node, route.raw_json)
    ticks = []
    order = []
    monkeypatch.setattr(node, 'get_logger', lambda: SimpleNamespace(info=lambda text: None, error=lambda text: order.append(('error', text))))
    publish = node.emergency_publisher.publish

    def emergency(message):
        assert node.throttle == 0 and not node.arm
        order.append(('emergency', message.error))
        publish(message)
        if core_requests_cancel:
            goal.is_cancel_requested = True

    def step(period):
        ticks.append(node.mission)
        assert len(ticks) < 30, 'Action did not exit after maneuver limit'
        node.speed = 0.0
        if node.maneuver_start is not None and not node.maneuver_stopping:
            position(node, 0, -2 if node.reverse else 0)
            node.speed = 0.2
        node.update()

    monkeypatch.setattr(node.emergency_publisher, 'publish', emergency)
    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert not result.completed
    assert goal.states == ['canceled' if core_requests_cancel else 'aborted']
    assert not ticks[-1].is_completed
    assert node.reverse_count == 5 and not node.goal_active
    assert order == [('error', node.mission_error), ('emergency', node.mission_error)]
    assert 'Geri manevra siniri' in node.mission_error
    assert log_rows(node)[-1]['error'] == node.mission_error


def test_action_waits_for_final_stop_then_succeeds_and_accepts_another_goal(node, monkeypatch):
    route = mission(node, [(0, 0), (0, 10)])
    node.mission = None
    goal = Goal(node, route.raw_json)
    ticks = []

    def step(period):
        ticks.append(node.mission)
        assert len(ticks) <= 2
        node.mission.set_index(node.mission.waypoint_count - 1)
        node.speed = 0.3 if len(ticks) == 1 else 0.0
        node.update()
        assert node.mission.is_completed == (len(ticks) == 2)

    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert result.completed and goal.states == ['succeeded'] and len(ticks) == 2
    assert ticks[-1].is_completed
    assert not node.emergency_publisher.messages
    assert log_rows(node)[-1]['event'] == 'completed'
    assert node.goal_callback(goal.request) == sys.modules['rclpy.action'].GoalResponse.ACCEPT


def test_control_exception_reports_emergency_and_aborts_action(node, monkeypatch):
    route = mission(node, [(0, 10)])
    node.mission = None
    goal = Goal(node, route.raw_json)

    def fail_drive(point, speed):
        raise ValueError('test control error')

    monkeypatch.setattr(node, 'go_to_point', fail_drive)
    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', lambda period: node.update())
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['aborted']
    assert 'test control error' in node.emergency_publisher.messages[0].error
    assert log_rows(node)[-1]['event'] == 'error'


def test_invalid_mission_reports_reason_and_aborts(node):
    goal = Goal(node, '{invalid json')
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['aborted']
    assert len(node.emergency_publisher.messages) == 1
    assert node.emergency_publisher.messages[0].error == node.mission_error
    assert not node.goal_active


@pytest.mark.parametrize('finish_at_cancel', [False, True])
def test_cancel_stops_action_without_success_or_emergency(node, monkeypatch, finish_at_cancel):
    route = mission(node, [(0, 10)])
    node.mission = None
    goal = Goal(node, route.raw_json)

    def step(period):
        node.update()
        if finish_at_cancel:
            node.mission.mark_completed()
        goal.is_cancel_requested = True

    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['canceled']
    assert not node.emergency_publisher.messages
    assert log_rows(node)[-1]['event'] == 'canceled'


def test_shutdown_exits_action_loop(node, monkeypatch):
    route = mission(node, [(0, 10)])
    node.mission = None
    goal = Goal(node, route.raw_json)

    def step(period):
        node.update()
        node.shutdown_requested = True

    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['aborted']
    assert not node.emergency_publisher.messages
    assert log_rows(node)[-1]['event'] == 'shutdown'


@pytest.mark.parametrize('compass, gnss, reverse, expected', [(359, 1, False, 2), (1, 359, False, 2), (0, 10, False, 10), (0, 180, True, 0), (359, 181, True, 2)])
def test_heading_comparison_wraps_and_accounts_for_reverse(node, compass, gnss, reverse, expected):
    node.compass_heading = compass
    node.gnss_heading = gnss
    node.gnss_heading_accuracy = 2.0
    node.speed = 0.6
    node.reverse = reverse
    node.update_heading()
    assert node.sensor_heading_difference == pytest.approx(expected)
    assert not node.emergency_publisher.messages


@pytest.mark.parametrize('speed, angle, acceleration, accuracy, fix, reverse', [(0.59, 0, 0, 2, True, False), (0.8, 6, 0, 2, True, False), (0.8, 0, 0.3, 2, True, False), (0.8, 0, 0, 10, True, False), (0.8, 0, 0, None, True, False), (0.8, 0, 0, 2, False, False), (0.8, 0, 0, 2, True, True)])
def test_compass_is_primary_outside_stable_straight_motion(node, speed, angle, acceleration, accuracy, fix, reverse):
    node.compass_heading = 40.0
    node.gnss_heading = 225.0 if reverse else 45.0
    node.gnss_heading_accuracy = accuracy
    node.speed = speed
    node.average_steering = angle
    node.acceleration = acceleration
    node.fix_ok = fix
    node.reverse = reverse
    node.heading_correction = 1.0
    node.update_heading()
    assert node.navigation_heading == 40.0 and node.heading_correction == 0.0
    assert not node.emergency_publisher.messages


@pytest.mark.parametrize('compass, gnss, correction', [(40, 45, 1), (359, 4, 1), (1, 356, -1)])
def test_reliable_gnss_gently_adjusts_straight_driving(node, compass, gnss, correction):
    node.compass_heading = compass
    node.gnss_heading = gnss
    node.gnss_heading_accuracy = 2.0
    node.fix_ok = True
    node.speed = 0.8
    node.update_heading()
    assert node.heading_correction == pytest.approx(correction * 0.1)
    for frame in range(100):
        node.update_heading()
    assert node.heading_correction == pytest.approx(correction, abs=0.001)
    assert (node.navigation_heading - compass - correction + 180) % 360 - 180 == pytest.approx(0, abs=0.001)
    aim = (math.sin(math.radians(compass)), math.cos(math.radians(compass)))
    assert node.heading_error(aim) == pytest.approx(-correction, abs=0.001)
    node.steering = 15
    node.update_heading()
    assert node.navigation_heading == compass


@pytest.mark.parametrize('phase', ['reverse_maneuver', 'recovery', 'the_end'])
def test_maneuvers_use_compass_even_with_accurate_gnss(node, phase):
    node.phase = phase
    node.gnss_heading = 5.0
    node.gnss_heading_accuracy = 1.0
    node.fix_ok = True
    node.speed = 0.8
    node.update_heading()
    assert node.navigation_heading == node.compass_heading


def test_heading_mismatch_is_logged_without_ending_action(node, monkeypatch):
    route = mission(node, [(0, 10)])
    node.mission = None
    goal = Goal(node, route.raw_json)
    node.speed = 0.6
    node.gnss_heading = 25.5
    node.gnss_heading_accuracy = 2.0
    node.fix_ok = True
    ticks = []

    def step(period):
        ticks.append(1)
        assert len(ticks) <= 3
        node.update()
        assert not node.mission_error and node.navigation_heading == node.compass_heading
        assert node.phase == node.TO_START
        if len(ticks) == 3:
            goal.is_cancel_requested = True

    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['canceled']
    assert not node.emergency_publisher.messages
    rows = log_rows(node)
    assert rows[-1]['event'] == 'canceled' and rows[-1]['error'] == ''
    assert float(rows[-1]['sensor_heading_difference_deg']) == 25.5
    assert rows[0]['navigation_heading_deg'] == rows[0]['compass_heading_deg']
    assert any(message.throttle > 0 for message in node.motion_publisher.messages)


@pytest.fixture
def control_time(monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'monotonic', lambda: now[0])
    return now


def test_four_seconds_stuck_recovers_two_metres_then_resumes(node, control_time):
    route = mission(node, [(0, 10)])
    node.update()
    node.average_steering = -12.0
    observe_first_motion(node)
    control_time[0] += 3.99
    node.update()
    assert node.phase == node.TO_START
    control_time[0] += 0.01
    node.update()
    assert node.phase == node.RECOVERY and node.recovery_phase == node.TO_START
    assert node.recovery_reverse and node.throttle == 0 and node.arm
    assert route.index == 0
    node.average_steering = 0.0
    node.update()
    assert node.reverse and node.throttle > 0 and node.steering == 0
    assert node.target_speed == 0.6
    row = log_rows(node)[-1]
    assert row['phase'] == 'recovery' and row['recovery_phase'] == 'to_start'
    assert row['recovery_reverse'] == 'True'
    position(node, 0, -1.8)
    node.speed = 0.6
    node.update()
    assert node.recovery_stopping and node.throttle == 0 and node.reverse
    node.update()
    assert node.phase == node.RECOVERY  # Yonu degistirmek icin durus beklenir.
    node.speed = 0.0
    node.update()
    assert node.phase == node.TO_START and node.throttle == 0 and node.reverse
    node.update()
    assert node.throttle > 0 and not node.reverse
    assert route.index == 0 and not node.emergency_publisher.messages


def test_stuck_duration_requires_continuous_standstill(node, control_time):
    mission(node, [(0, 10)])
    node.update()
    observe_first_motion(node)
    control_time[0] += 3.5
    node.speed = 0.1
    node.check_stuck()
    assert node.stuck_since is None
    node.speed = 0.0
    node.check_stuck()
    control_time[0] += 3.9
    node.check_stuck()
    node.log_drive()
    assert node.phase == node.TO_START
    assert float(log_rows(node)[-1]['stuck_duration_s']) == pytest.approx(3.9)
    control_time[0] += 0.1
    node.check_stuck()
    assert node.phase == node.RECOVERY


def test_planned_stop_and_missing_data_do_not_trigger_recovery(node, control_time):
    route = mission(node, [(0, 0)])
    node.update()
    control_time[0] += 10
    node.update()
    assert route.is_completed and node.phase == node.THE_END
    assert node.stuck_since is None and node.recovery_phase is None
    mission(node, [(0, 10)])
    node.update()
    node.speed = None
    control_time[0] += 10
    node.update()
    assert node.stuck_since is None and node.recovery_phase is None


@pytest.mark.parametrize('reverse', [False, True])
def test_recovery_preserves_interrupted_maneuver_and_its_distance(node, control_time, reverse):
    route = mission(node, [(0, -100)])
    node.update()
    observe_first_motion(node)
    node.update()
    node.maneuver_reverse = reverse
    node.maneuver_heading = 180.0 if reverse else 0.0
    node.reverse_count = 3
    node.stop()
    node.update()
    original_start = node.maneuver_start
    control_time[0] += 4
    node.update()
    assert node.phase == node.RECOVERY and node.recovery_reverse != reverse
    node.update()
    assert node.reverse != reverse
    # Kurtarma, orijinal manevranin baslangicinin 2 m gerisine goturur.
    position(node, 0, 2 if reverse else -2)
    node.speed = 0.3
    node.update()
    assert node.recovery_stopping and node.throttle == 0
    node.speed = 0.0
    node.update()
    assert node.phase == node.REVERSE_MANEUVER
    node.update()
    assert node.reverse == reverse and node.throttle > 0
    assert not node.maneuver_stopping
    assert node.maneuver_distance() == pytest.approx(-2.0)
    assert node.maneuver_start == original_start and node.reverse_count == 3 and route.index == 0
    position(node, 0, 0)
    node.speed = 0.2
    node.update()
    assert node.throttle > 0 and not node.maneuver_stopping
    position(node, 0, -2 if reverse else 2)
    node.update()
    assert node.maneuver_stopping and node.throttle == 0


def test_recovery_does_not_consume_waypoints_it_crosses(node, control_time):
    route = mission(node, [(0, -1), (0, -10)])
    node.update()
    observe_first_motion(node)
    node.update()
    control_time[0] += 4
    node.update()
    assert node.phase == node.RECOVERY
    node.update()
    position(node, 0, -1)
    node.speed = 0.2
    node.update()
    assert route.index == 0 and node.phase == node.RECOVERY


def test_failed_recovery_reports_emergency_and_ends_action(node, monkeypatch, control_time):
    route = mission(node, [(0, 10)])
    node.mission = None
    goal = Goal(node, route.raw_json)
    ticks = []

    def step(period):
        ticks.append(1)
        assert len(ticks) <= 4
        if len(ticks) in (2, 4):
            control_time[0] += 4
        node.update()
        if len(ticks) == 1:
            observe_first_motion(node)

    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['aborted']
    assert len(node.emergency_publisher.messages) == 1
    assert 'Kurtarma basarisiz' in node.mission_error
    assert log_rows(node)[-1]['event'] == 'error'


def test_cancel_during_recovery_stops_and_closes_action(node, monkeypatch, control_time):
    route = mission(node, [(0, 10)])
    node.mission = None
    goal = Goal(node, route.raw_json)
    ticks = []

    def step(period):
        ticks.append(1)
        assert len(ticks) <= 3
        if len(ticks) == 2:
            control_time[0] += 4
        node.update()
        if len(ticks) == 1:
            observe_first_motion(node)
        if len(ticks) == 3:
            assert node.phase == node.RECOVERY and node.reverse
            goal.is_cancel_requested = True

    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['canceled']
    assert not node.emergency_publisher.messages


def test_missing_position_during_recovery_logs_wait_and_resets_stall_timer(node, control_time):
    mission(node, [(0, 10)])
    node.update()
    observe_first_motion(node)
    control_time[0] += 4
    node.update()
    node.update()
    node.latitude = None
    node.update()
    assert log_rows(node)[-1]['event'] == 'waiting_for_data'
    assert node.stuck_since is None and not node.emergency_publisher.messages


@pytest.fixture
def braking_node(node, control_time):
    node.BRAKE_SETTLE_TIME = type(node).BRAKE_SETTLE_TIME
    return node


def test_direction_change_waits_for_brake_and_speed_without_disarming(braking_node, control_time):
    node = braking_node
    mission(node, [(0, -10)])
    node.update()
    assert node.arm and node.throttle == 0 and not node.stop_steering
    control_time[0] += 0.99
    node.update()
    assert not node.reverse and node.reverse_count == 0
    control_time[0] += 0.01
    node.update()
    assert node.arm and node.reverse and node.throttle > 0 and not node.stop_steering
    position(node, 0, -1.8)
    node.speed = 0.6
    node.update()
    assert node.throttle == 0 and node.arm and node.reverse
    assert node.steering == 0 and not node.stop_steering
    control_time[0] += 1.0
    node.update()
    assert node.reverse and node.throttle == 0  # Sure doldu ama robot hala hareketli.
    node.speed = 0.0
    node.update()
    assert not node.reverse and node.throttle > 0
    assert all(message.arm for message in node.motion_publisher.messages)


def test_recovery_waits_at_both_direction_changes_and_keeps_arm(braking_node, control_time):
    node = braking_node
    mission(node, [(0, 10)])
    node.update()
    observe_first_motion(node)
    control_time[0] += 4.0
    node.update()
    assert node.phase == node.RECOVERY and not node.reverse
    control_time[0] += 0.9
    node.update()
    assert node.throttle == 0 and not node.reverse and node.arm
    assert node.stuck_since is None
    control_time[0] += 0.1
    node.update()
    assert node.reverse and node.throttle > 0
    position(node, 0, -1.8)
    node.speed = 0.6
    node.update()
    node.speed = 0.0
    control_time[0] += 0.9
    node.update()
    assert node.phase == node.RECOVERY and node.reverse and not node.stop_steering
    control_time[0] += 0.1
    node.update()
    assert node.phase == node.TO_START and node.throttle == 0
    node.update()
    assert node.throttle > 0 and not node.reverse
    assert all(message.arm for message in node.motion_publisher.messages)


def test_normal_completion_waits_for_brake_before_disarming_action(braking_node, monkeypatch, control_time):
    node = braking_node
    route = mission(node, [(0, 0)])
    node.mission = None
    goal = Goal(node, route.raw_json)
    ticks = []

    def step(period):
        ticks.append(1)
        assert len(ticks) <= 3
        if len(ticks) == 2:
            control_time[0] += 0.9
        if len(ticks) == 3:
            control_time[0] += 0.1
        node.update()
        assert node.arm == (len(ticks) < 3)
        assert node.mission.is_completed == (len(ticks) == 3)

    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert result.completed and goal.states == ['succeeded']
    assert node.stop_steering
    assert float(log_rows(node)[-1]['brake_elapsed_s']) == pytest.approx(1.0)


def test_emergency_does_not_wait_for_brake_before_disarming(braking_node):
    node = braking_node
    mission(node, [(0, 10)])
    node.speed = 0.6
    node.update()
    node.fail_mission('test emergency while moving')
    assert not node.arm and node.stop_steering
    assert node.throttle == node.steering == 0
    assert not node.stop_complete()
    assert node.emergency_publisher.messages[-1].emergency


def test_cancel_does_not_wait_for_brake_before_disarming(braking_node, monkeypatch):
    node = braking_node
    route = mission(node, [(0, -10)])
    node.mission = None
    goal = Goal(node, route.raw_json)

    def step(period):
        node.update()
        assert node.arm and not node.stop_complete()
        goal.is_cancel_requested = True

    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['canceled']
    assert not node.arm and node.stop_steering


@pytest.mark.parametrize('throttle', [0, 1, 4, 5])
def test_brake_threshold_does_not_disarm_or_count_as_stuck(node, monkeypatch, control_time, throttle):
    mission(node, [(0, 10)])
    node.update()
    observe_first_motion(node)
    node.coast = False
    monkeypatch.setattr(node, 'speed_throttle', lambda: throttle)
    node.drive(0.6, 20, False)
    node.publish_motion()
    node.check_stuck()
    assert node.arm
    assert not node.stop_steering
    assert (node.stuck_since is None) == (throttle < 5)
    if throttle < 5:
        assert node.steering == 0
    row = log_rows(node)[-1]
    assert row['arm'] == 'True'


def test_power_command_resets_brake_wait(braking_node, control_time):
    node = braking_node
    mission(node, [(0, -10)])
    node.update()
    control_time[0] += 0.9
    node.drive(0.6, 0, False)
    node.publish_motion()
    assert node.brake_since is None
    node.stop()
    control_time[0] += 0.2
    assert not node.stop_complete()
    control_time[0] += 0.8
    assert node.stop_complete()


@pytest.fixture
def motion_node(node, monkeypatch):
    monkeypatch.setattr(sys.modules['std_msgs.msg'], 'Int32MultiArray', Message, raising=False)
    for name in ['RcInput', 'HubMotor', 'RobotRelay', 'Steer', 'Brake', 'Malfunction']:
        monkeypatch.setattr(sys.modules['robot_interfaces.msg'], name, Message, raising=False)
    path = Path(__file__).parents[1] / 'robot_core' / 'motion_node.py'
    spec = importlib.util.spec_from_file_location('robot_core.motion_node', path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module.MotionNode()


@pytest.mark.parametrize('arm, stop_steering, expected_stop', [(True, True, True), (True, False, False), (False, False, True)])
def test_motion_node_controls_steering_independently_of_relay(motion_node, arm, stop_steering, expected_stop):
    motion = motion_node
    message = Message()
    message.arm = arm
    message.throttle = 0
    message.steering = 0
    message.reverse = True
    message.stop_steering = stop_steering
    motion.motion_callback(message)
    assert motion.relay_publisher.messages[-1].ignition == arm
    assert motion.relay_publisher.messages[-1].reverse
    assert motion.steer_publisher.messages[-1].stop_all == expected_stop
    assert motion.brake_publisher.messages[-1].brake


def observe_first_motion(node):
    """Kurtarma testlerini gorevin ilk hareketi gozlenmis durumuna getirir."""
    previous_speed = node.speed
    node.speed = 0.1
    node.check_stuck()
    node.speed = previous_speed
    node.check_stuck()


def test_first_start_ignores_stall_until_robot_actually_moves(node, control_time):
    mission(node, [(0, 10)])
    for elapsed in [0.0, 4.0, 10.0, 30.0]:
        control_time[0] = 1000.0 + elapsed
        node.update()
        assert not node.mission_has_moved and node.stuck_since is None
        assert node.phase == node.TO_START and not node.reverse
        assert node.throttle >= node.throttle_deadband and node.arm
    assert not node.emergency_publisher.messages
    assert log_rows(node)[-1]['mission_has_moved'] == 'False'
    node.speed = 0.1
    node.update()
    assert node.mission_has_moved and node.phase == node.TO_START
    node.speed = 0.0
    node.update()
    control_time[0] += 3.99
    node.update()
    assert node.phase == node.TO_START
    control_time[0] += 0.01
    node.update()
    assert node.phase == node.RECOVERY
    assert log_rows(node)[-1]['mission_has_moved'] == 'True'


def test_initial_start_exemption_resets_only_for_new_mission(node, control_time):
    mission(node, [(0, 10)])
    node.speed = 0.1
    node.update()
    node.speed = 0.0
    node.stop()
    node.update()
    assert node.mission_has_moved and node.stuck_since is not None
    mission(node, [(0, 20)])
    node.update()
    control_time[0] += 10.0
    node.update()
    assert not node.mission_has_moved and node.stuck_since is None
    assert node.phase == node.TO_START and not node.reverse


@pytest.mark.parametrize('compass, difference', [(246.7, 0.0), (257.2, 10.5)])
def test_reported_reverse_heading_is_corrected_once(node, compass, difference):
    node.speed = 0.6
    node.reverse = True
    node.compass_heading = compass
    node.gnss_heading = 66.7
    node.gnss_heading_accuracy = 4.3
    node.update_heading()
    assert node.gnss_heading == 66.7  # Ham GNSS olcumu korunur.
    assert node.gnss_body_heading == pytest.approx(246.7)
    assert node.sensor_heading_difference == pytest.approx(difference)
    assert node.navigation_heading == compass
    assert not node.mission_error and not node.emergency_publisher.messages


@pytest.fixture
def core_node(node, monkeypatch):
    monkeypatch.setattr(sys.modules['rclpy.action'], 'ActionClient', lambda *args: None, raising=False)
    for name in ['Int32MultiArray', 'Int32']:
        monkeypatch.setattr(sys.modules['std_msgs.msg'], name, Message, raising=False)
    for name in ['RcInput', 'Command', 'Malfunction']:
        monkeypatch.setattr(sys.modules['robot_interfaces.msg'], name, Message, raising=False)
    services = ModuleType('robot_interfaces.srv')
    services.MissionGet = SimpleNamespace(Request=Message)
    monkeypatch.setitem(sys.modules, 'robot_interfaces.srv', services)
    buzzer = ModuleType('robot_core.Buzzer')
    buzzer.Buzzer = lambda pin: SimpleNamespace(shutdown=lambda: None)
    monkeypatch.setitem(sys.modules, 'robot_core.Buzzer', buzzer)
    monkeypatch.setattr(Node, 'create_client', lambda *args: SimpleNamespace(), raising=False)
    path = Path(__file__).parents[1] / 'robot_core' / 'core_node.py'
    spec = importlib.util.spec_from_file_location('robot_core.core_node', path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    core = module.CoreNode()
    core.rc_connected = True
    return core


def test_core_mission_updates_publish_state_without_competing_motion(core_node):
    states = sys.modules['robot_core.Tools'].RobotState
    core_node.robot_state = states.MISSION
    for frame in range(5):
        core_node.update()
    assert len(core_node.state_publisher.messages) == 5
    assert all(message.data == int(states.MISSION) for message in core_node.state_publisher.messages)
    assert not core_node.motion_publisher.messages


def test_core_remains_available_to_cancel_action_on_emergency(core_node):
    states = sys.modules['robot_core.Tools'].RobotState
    core_node.robot_state = states.MISSION
    core_node.autonomous_task_active = True
    cancellations = []
    core_node.autonomous_goal_handle = SimpleNamespace(cancel_goal_async=lambda: cancellations.append(True))
    core_node.update()
    core_node.emergency_callback(SimpleNamespace(emergency=True, error='test autonomous failure'))
    core_node.update()
    assert core_node.robot_state == states.EMERGENCY
    assert cancellations == [True]
    message = core_node.motion_publisher.messages[-1]
    assert not message.arm and not message.reverse and message.throttle == 0


@pytest.mark.parametrize('arm, coast, expected_brake', [(True, True, False), (True, False, True), (False, True, True), (False, False, True)])
def test_zero_throttle_coasts_only_when_explicitly_requested_and_armed(motion_node, arm, coast, expected_brake):
    message = Message()
    message.arm = arm
    message.coast = coast
    message.throttle = message.steering = 0
    message.reverse = False
    motion_node.motion_callback(message)
    assert motion_node.brake_publisher.messages[-1].brake == expected_brake
    assert motion_node.hubmotor_publisher.messages[-1].front_left == 0
    motion_node.stop_everything()
    assert motion_node.brake_publisher.messages[-1].brake
    assert not motion_node.relay_publisher.messages[-1].ignition


def test_speed_trim_and_speed_limit_coast_without_braking(node, motion_node):
    mission(node, [(0, 10)])
    node.speed = 0.95
    node.acceleration = 0.4
    node.driving_mission = node.mission
    node.drive(0.8, 8, False)
    node.publish_motion()
    motion_node.rc_callback(SimpleNamespace(connected=True, ignition=0))
    motion_node.state_callback(SimpleNamespace(data=int(sys.modules["robot_core.Tools"].RobotState.MISSION)))
    motion_node.motion_callback(node.motion_publisher.messages[-1])
    assert node.throttle < node.CRUISE_THROTTLE and node.coast and node.steering == 8
    assert not node.stop_steering and not motion_node.brake_publisher.messages[-1].brake
    node.speed = 1.01
    node.drive(0.8, 8, False)
    node.publish_motion()
    motion_node.motion_callback(node.motion_publisher.messages[-1])
    assert node.throttle == 0 and node.coast and node.steering == 8
    assert not motion_node.brake_publisher.messages[-1].brake
    assert node.arm





@pytest.mark.parametrize('speed, expected', [(0.0, 0), (0.299, 0), (0.3, 0), (0.301, 30)])
@pytest.mark.parametrize('reverse', [False, True])
def test_steering_requires_motion_above_point_three_regardless_of_accuracy(node, speed, expected, reverse):
    node.speed = speed
    node.speed_accuracy = 999.0
    node.drive(0.6, 30, reverse)
    assert node.steering == expected
    assert node.reverse == reverse


def test_stopped_wheels_center_before_any_launch_power(node, motion_node, control_time):
    mission(node, [(0, 10)])
    node.average_steering = -30.0
    node.update()
    assert node.steering == 0 and node.throttle == 0 and node.arm
    assert not node.stop_steering and not node.coast
    states = sys.modules['robot_core.Tools'].RobotState
    motion_node.rc_callback(SimpleNamespace(connected=True, ignition=0))
    motion_node.state_callback(SimpleNamespace(data=int(states.MISSION)))
    motion_node.motion_callback(node.motion_publisher.messages[-1])
    steer = motion_node.steer_publisher.messages[-1]
    assert steer.front_left == steer.front_right == steer.rear_left == steer.rear_right == 0
    assert not steer.stop_all
    control_time[0] += 5
    node.update()
    assert node.throttle == 0 and node.stuck_since is None
    node.average_steering = 0.0
    node.update()
    assert node.throttle > 0 and node.steering == 0


@pytest.mark.parametrize('reverse', [False, True])
def test_launch_waits_for_release_and_ramps_without_a_power_pulse(node, control_time, reverse):
    node.BRAKE_RELEASE_TIME = type(node).BRAKE_RELEASE_TIME
    outputs = []
    for frame in range(51):
        control_time[0] = 1000.0 + frame / 10.0
        node.drive(0.6, 30, reverse)
        assert node.steering == 0 and node.coast
        if frame < 10:
            assert node.throttle == 0
        else:
            outputs.append(node.throttle)
    assert outputs[0] <= node.CRUISE_THROTTLE + 1
    assert max(outputs) <= node.START_THROTTLE
    assert all(0 <= b - a <= 1 for a, b in zip(outputs, outputs[1:]))
    assert outputs[-1] == node.START_THROTTLE
    node.speed = 0.31
    node.acceleration = 0.4
    node.drive(0.6, 30, reverse)
    assert not node.launching and node.steering == 30
    assert node.throttle <= outputs[-1]


@pytest.mark.parametrize('speed, acceleration', [(0.3, 0), (0.55, 1.3), (0.8, -0.5), (1.1, 0.4), (1.56, -0.5)])
def test_speed_regulation_never_cycles_mechanical_brake(node, motion_node, speed, acceleration):
    states = sys.modules['robot_core.Tools'].RobotState
    motion_node.rc_callback(SimpleNamespace(connected=True, ignition=0))
    motion_node.state_callback(SimpleNamespace(data=int(states.MISSION)))
    mission(node, [(0, 10)])
    node.driving_mission = node.mission
    node.speed = speed
    node.acceleration = acceleration
    node.drive(0.8, 0, False)
    node.publish_motion()
    motion_node.motion_callback(node.motion_publisher.messages[-1])
    assert node.coast and not motion_node.brake_publisher.messages[-1].brake
    if speed >= 1.0:
        assert node.throttle == 0
    node.stop()
    node.publish_motion()
    motion_node.motion_callback(node.motion_publisher.messages[-1])
    assert motion_node.brake_publisher.messages[-1].brake


@pytest.mark.parametrize('connected, armed', [(False, False), (False, True), (True, True)])
def test_rc_is_required_at_both_mission_entry_points(node, core_node, connected, armed):
    states = sys.modules['robot_core.Tools'].RobotState
    request = SimpleNamespace(mission_raw='{}')
    node.rc_connected = core_node.rc_connected = connected
    node.rc_armed = core_node.rc_ignition = armed
    core_node.robot_state = states.IDLE
    assert node.goal_callback(request) == sys.modules['rclpy.action'].GoalResponse.REJECT
    core_node.start_mission(42)
    assert not core_node.autonomous_task_active


@pytest.mark.parametrize('handle_ready', [False, True])
def test_rc_arm_cancels_and_publishes_manual_without_waiting_for_action(core_node, handle_ready):
    states = sys.modules['robot_core.Tools'].RobotState
    core_node.robot_state = states.MISSION
    core_node.autonomous_task_active = True
    calls = []
    handle = SimpleNamespace(cancel_goal_async=lambda: calls.append('cancel'))
    core_node.autonomous_goal_handle = handle if handle_ready else None
    rc = SimpleNamespace(connected=True, ignition=1, reverse=0, throttle=0, steering=0)
    core_node.rc_callback(rc)
    assert core_node.robot_state == states.MANUAL
    assert core_node.autonomous_cancel_requested
    assert core_node.motion_publisher.messages[-1].arm
    assert not core_node.motion_publisher.messages[-1].autonomous
    assert core_node.state_publisher.messages[-1].data == int(states.MANUAL)
    assert calls == (['cancel'] if handle_ready else [])
    if not handle_ready:
        handle.accepted = True
        handle.get_result_async = lambda: SimpleNamespace(add_done_callback=lambda fn: None)
        core_node._autonomous_goal_response(SimpleNamespace(result=lambda: handle))
        assert calls == ['cancel']
    core_node.rc_callback(rc)
    assert calls == ['cancel']
    core_node._autonomous_result_received(SimpleNamespace(result=lambda: SimpleNamespace(result=SimpleNamespace(completed=False))))
    assert core_node.robot_state == states.MANUAL
    core_node.emergency_callback(SimpleNamespace(node_name='auto_act_node', emergency=True, error='old mission error'))
    assert core_node.robot_state == states.MANUAL


def test_arm_on_then_off_while_mission_is_loading_does_not_start_it(core_node):
    states = sys.modules['robot_core.Tools'].RobotState
    core_node.autonomous_task_active = True
    core_node.rc_callback(SimpleNamespace(connected=True, ignition=1, reverse=0, throttle=0, steering=0))
    core_node.rc_callback(SimpleNamespace(connected=True, ignition=0, reverse=0, throttle=0, steering=0))
    assert core_node.robot_state == states.IDLE
    assert core_node.autonomous_cancel_requested
    core_node._mission_received(SimpleNamespace(result=lambda: pytest.fail('Canceled fetch must not be used')))
    assert not core_node.autonomous_task_active


def test_rc_loss_cancels_mission_and_blocks_delayed_service_response(core_node):
    states = sys.modules['robot_core.Tools'].RobotState
    core_node.robot_state = states.MISSION
    core_node.autonomous_task_active = True
    calls = []
    core_node.autonomous_goal_handle = SimpleNamespace(cancel_goal_async=lambda: calls.append(True))
    core_node.rc_callback(SimpleNamespace(connected=False))
    assert calls == [True] and not core_node.rc_connected
    assert core_node.robot_state == states.IDLE
    assert not core_node.motion_publisher.messages[-1].arm
    core_node._mission_received(SimpleNamespace(result=lambda: pytest.fail('Disconnected mission must not start')))
    assert not core_node.autonomous_task_active


def test_motion_rejects_old_autonomous_commands_after_rc_takeover(motion_node):
    states = sys.modules['robot_core.Tools'].RobotState
    auto = Message()
    auto.arm = auto.coast = auto.autonomous = True
    auto.throttle = 50
    auto.steering = 20
    auto.reverse = False
    motion_node.motion_callback(auto)
    assert not motion_node.hubmotor_publisher.messages  # RC/state not received yet.
    motion_node.rc_callback(SimpleNamespace(connected=True, ignition=0))
    motion_node.state_callback(SimpleNamespace(data=int(states.MISSION)))
    motion_node.motion_callback(auto)
    assert motion_node.hubmotor_publisher.messages[-1].front_left > 0
    motion_node.rc_callback(SimpleNamespace(connected=True, ignition=1))
    count = len(motion_node.hubmotor_publisher.messages)
    motion_node.motion_callback(auto)
    assert len(motion_node.hubmotor_publisher.messages) == count
    manual = Message()
    manual.arm = True
    manual.throttle = 25
    manual.steering = -5
    manual.reverse = True
    motion_node.motion_callback(manual)
    assert motion_node.relay_publisher.messages[-1].reverse
    count = len(motion_node.hubmotor_publisher.messages)
    auto.arm = False
    auto.throttle = 0  # Even delayed action cleanup cannot brake manual driving.
    motion_node.motion_callback(auto)
    assert len(motion_node.hubmotor_publisher.messages) == count
    motion_node.rc_callback(SimpleNamespace(connected=True, ignition=0))
    motion_node.state_callback(SimpleNamespace(data=int(states.MISSION)))  # Same old mission.
    assert motion_node.autonomous_blocked
    motion_node.state_callback(SimpleNamespace(data=int(states.IDLE)))
    motion_node.state_callback(SimpleNamespace(data=int(states.MISSION)))  # Explicit new mission.
    assert not motion_node.autonomous_blocked


def test_rc_revocation_stops_updates_and_action_cancel_finishes_without_emergency(node, monkeypatch):
    route = mission(node, [(0, 10)])
    node.mission = None
    goal = Goal(node, route.raw_json)
    ticks = []
    def step(period):
        ticks.append(1)
        assert len(ticks) <= 2
        node.update()
        if len(ticks) == 1:
            node.rc_callback(SimpleNamespace(connected=True, ignition=1))
            node.rc_callback(SimpleNamespace(connected=True, ignition=0))
            node.update()
            assert node.throttle == 0 and node.control_revoked
        else:
            node.cancel_callback(goal)
            goal.is_cancel_requested = True
    monkeypatch.setattr(sys.modules['robot_core.auto_act_node'].time, 'sleep', step)
    result = node.execute_callback(goal)
    assert not result.completed and goal.states == ['canceled']
    assert not node.emergency_publisher.messages
    assert log_rows(node)[-1]['control_revoked'] == 'True'


@pytest.mark.parametrize('delay', [0.3, 0.6, 1.0])
@pytest.mark.parametrize('loaded', [False, True])
def test_speed_settles_on_different_delayed_motor_models(node, control_time, delay, loaded):
    """Iki varsayimsal zemin: saha garantisi degil, hiz dongusunun regresyon testi."""
    from collections import deque
    node.BRAKE_RELEASE_TIME = type(node).BRAKE_RELEASE_TIME
    commands = deque([0] * int(delay * node.control_frequency))
    speeds = []
    for frame in range(1400):
        control_time[0] += 0.1
        node.update_acceleration()
        node.drive(0.8, 0, False)
        assert node.coast
        commands.append(node.throttle)
        throttle = commands.popleft()
        if loaded:
            acceleration = 0.0183 * max(0, throttle - 25) - 1.146 * node.speed
        else:
            acceleration = throttle / 35.0 - node.speed
        node.speed = max(0.0, node.speed + acceleration * 0.1)
        speeds.append(node.speed)
    # Sadece yerlesmis hiz; modelin gecikmeli kalkisi mutlak hiz sinirini kanitlamaz.
    assert 0.77 < min(speeds[-200:]) <= max(speeds[-200:]) < 0.83
    assert max(speeds[-200:]) - min(speeds[-200:]) < 0.03
