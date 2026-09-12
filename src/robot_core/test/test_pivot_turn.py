"""Eksenel donus: gercek node fonksiyonlari, sahte ROS ve I2C; donanim calistirilmaz."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from test_auto_act_node import Message, Node, Publisher, node, motion_node, control_time, mission, position, log_rows


@pytest.mark.parametrize('east, expected', [(-10, -1), (10, 1)])
def test_pivot_waits_for_stop_four_angles_and_brake_release(node, control_time, east, expected):
    mission(node, [(east, 0)])
    node.BRAKE_SETTLE_TIME = 1.0
    node.BRAKE_RELEASE_TIME = 1.0
    node.speed = 0.4
    node.update()
    assert node.phase == node.PIVOT_TURN and node.pivot_direction == expected
    assert node.pivot == 0 and node.throttle == 0 and node.arm
    control_time[0] += 1.1
    node.update()
    assert node.pivot_stage == 'stopping'  # Sure dolsa da once durmali.
    node.speed = 0.0
    node.update()
    assert node.pivot_stage == 'aligning' and node.pivot == 0
    node.update()
    assert node.pivot == expected and node.throttle == 0 and not node.coast
    node.steering_angles = [45, -45, -45, 40]  # Ortalama yerine her teker onemli.
    node.update()
    assert node.pivot_stage == 'aligning' and node.throttle == 0
    node.steering_angles = [45, -45, -45, 45]
    node.update()
    assert node.pivot_stage == 'turning' and node.throttle == 0 and node.coast
    control_time[0] += 0.9
    node.update()
    assert node.throttle == 0 and node.coast
    control_time[0] += 0.11
    node.update()
    msg = node.motion_publisher.messages[-1]
    assert msg.pivot == expected and msg.arm and not msg.reverse and msg.coast
    assert 0 < msg.throttle <= node.CRUISE_THROTTLE + 1
    assert all(msg.arm for msg in node.motion_publisher.messages)
    row = log_rows(node)[-1]
    assert row['pivot_stage'] == 'turning' and int(row['pivot']) == expected
    assert float(row['steering_rear_right_deg']) == 45


def begin_turn(node, east=10):
    mission(node, [(east, 0)])
    node.update()
    node.update()
    node.update()
    node.steering_angles = [45, -45, -45, 45]
    node.update()
    node.update()
    assert node.pivot_stage == 'turning' and node.throttle > 0


def test_pivot_requires_individual_angle_feedback(node):
    mission(node, [(10, 0)])
    node.steering_angles = None
    for tick in range(20):
        node.update()
    assert node.pivot_stage == 'stopping' and node.pivot == 0 and node.throttle == 0
    node.steering_angles_callback(SimpleNamespace(data=[0, 0, 0]))
    assert node.steering_angles is None
    node.steering_angles_callback(SimpleNamespace(data=[0, 0, 0, 0]))
    node.update()
    assert node.pivot_stage == 'aligning'


def test_turn_target_does_not_follow_offset_gnss_antenna(node):
    begin_turn(node)
    target = node.pivot_heading
    position(node, 1, 1)
    node.update()
    assert node.pivot_heading == target == pytest.approx(90)


@pytest.mark.parametrize('first, second, expected', [(359, 1, 20), (1, 359, -20)])
def test_compass_turn_rate_wrap(node, first, second, expected):
    node.compass_heading = first
    node.update_turn_rate()
    node.compass_heading = second
    node.update_turn_rate()
    assert node.turn_rate == expected


def test_rotation_does_not_trigger_linear_recovery_at_zero_gnss_speed(node, control_time):
    begin_turn(node)
    for tick in range(60):
        control_time[0] += 0.1
        node.compass_heading += 0.7
        node.update()
        assert node.phase == node.PIVOT_TURN and node.recovery_phase is None
        assert not node.mission_error
    assert node.speed == 0.0 and node.turn_rate == pytest.approx(7)


def test_pivot_brakes_with_x_wheels_then_centers_before_forward_launch(node, control_time):
    begin_turn(node)
    node.BRAKE_SETTLE_TIME = 1.0
    node.compass_heading = 87.0
    node.update()
    assert node.pivot_stage == 'braking' and node.pivot == 1
    assert node.throttle == 0 and not node.coast
    for tick in range(9):
        control_time[0] += 0.1
        node.update()
        assert node.pivot == 1 and node.throttle == 0
    control_time[0] += 0.2
    node.update()
    assert node.pivot_stage == 'centering' and node.pivot == 1
    node.update()
    assert node.pivot == 0 and node.throttle == 0 and node.phase == node.PIVOT_TURN
    node.steering_angles = [0, 0, 0, 5]
    node.update()
    assert node.pivot_stage == 'centering' and node.throttle == 0
    node.steering_angles = [0, 0, 0, 0]
    node.update()
    assert node.phase == node.TO_START and node.throttle == 0
    node.update()
    assert node.pivot == 0 and not node.reverse and node.throttle > 0 and node.steering == 0


def test_pivot_overshoot_changes_direction_only_after_braking(node, control_time):
    begin_turn(node)
    node.BRAKE_SETTLE_TIME = 1.0
    node.compass_heading = 110.0
    node.update()
    assert node.pivot_stage == 'braking' and node.pivot == 1
    for tick in range(12):
        control_time[0] += 0.1
        node.update()
        assert node.throttle == 0
        if node.pivot_direction == -1:
            break
    assert node.pivot_direction == -1 and node.pivot == 1 and node.turn_rate == 0
    node.update()
    assert node.pivot == -1 and node.throttle == 0


@pytest.mark.parametrize('how', ['cancel', 'rc', 'emergency'])
def test_pivot_interrupt_clears_all_movement_commands(node, how):
    begin_turn(node)
    if how == 'cancel':
        node.cancel_callback(None)
    elif how == 'rc':
        node.rc_callback(SimpleNamespace(connected=True, ignition=1))
    else:
        node.fail_mission('test')
    node.update()
    msg = node.motion_publisher.messages[-1]
    assert msg.pivot == 0 and msg.throttle == 0 and msg.steering == 0
    assert not msg.arm and not msg.coast and not msg.reverse


@pytest.mark.parametrize('direction, reverse', [(-1, [True, False, True, False]), (1, [False, True, False, True])])
def test_motion_pivot_wheel_order_and_equal_power(motion_node, direction, reverse):
    motion_node.avg_steering = 27  # Ackermann eksenel donuste uygulanmamali.
    msg = Message()
    msg.arm = True
    msg.throttle = 40
    msg.steering = 0
    msg.reverse = False
    msg.pivot = direction
    msg.coast = True
    motion_node.motion_callback(msg)
    steer = motion_node.steer_publisher.messages[-1]
    hub = motion_node.hubmotor_publisher.messages[-1]
    relay = motion_node.relay_publisher.messages[-1]
    names = ['front_left', 'front_right', 'rear_left', 'rear_right']
    assert [getattr(steer, n) for n in names] == [45, -45, -45, 45]
    assert steer.pivot and not steer.stop_all
    assert [getattr(hub, n) for n in names] == [40] * 4
    assert relay.independent_reverse and [getattr(relay, 'reverse_' + n) for n in names] == reverse
    assert not motion_node.brake_publisher.messages[-1].brake
    # Normal komut, dort yonu yeniden birlikte surer; arka dumenler ters acidadir.
    msg.pivot = 0
    msg.steering = 15
    msg.reverse = True
    motion_node.motion_callback(msg)
    steer = motion_node.steer_publisher.messages[-1]
    relay = motion_node.relay_publisher.messages[-1]
    assert [getattr(steer, n) for n in names] == [15, 15, -15, -15]
    assert not steer.pivot and not relay.independent_reverse and relay.reverse


def test_motion_disarmed_command_cannot_pivot(motion_node):
    msg = Message()
    msg.arm = False
    msg.throttle = 50
    msg.steering = 0
    msg.reverse = False
    msg.pivot = 1
    msg.coast = True
    motion_node.motion_callback(msg)
    assert not motion_node.steer_publisher.messages[-1].pivot
    assert not motion_node.relay_publisher.messages[-1].independent_reverse
    assert motion_node.hubmotor_publisher.messages[-1].front_left == 0
    assert motion_node.brake_publisher.messages[-1].brake


def import_hardware_node(name, monkeypatch):
    services = ModuleType('robot_interfaces.srv')
    services.I2COperation = SimpleNamespace(Request=Message)
    services.EncoderZeroGet = SimpleNamespace(Request=Message)
    monkeypatch.setitem(sys.modules, 'robot_interfaces.srv', services)
    path = Path(__file__).parents[1] / 'robot_core' / (name + '.py')
    spec = importlib.util.spec_from_file_location('robot_core.' + name, path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


class Future:
    def __init__(self):
        self.callback = None
        self.success = True

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return SimpleNamespace(success=self.success, error='test I2C error')

    def finish(self, success=True):
        self.success = success
        self.callback(self)


@pytest.fixture
def relay_node(motion_node, monkeypatch):
    module = import_hardware_node('relay_node', monkeypatch)
    monkeypatch.setattr(Node, 'create_client', lambda *args: None, raising=False)
    monkeypatch.setattr(module, '_wait_for_service', lambda *args: None)
    monkeypatch.setattr(module.RelayNode, 'write_register_sync', lambda *args: None)
    relay = module.RelayNode()
    relay.writes = []

    def write(register, value):
        future = Future()
        relay.writes.append((register, value, future))
        return future

    monkeypatch.setattr(relay, 'write_register_async', write)
    return relay


def relay_command(direction=0, reverse=False):
    msg = Message()
    msg.ignition = True
    msg.reverse = reverse
    msg.brake = False
    msg.back_energy = True
    msg.alarm_lamp = True
    msg.independent_reverse = direction != 0
    msg.reverse_front_left = msg.reverse_rear_left = direction < 0
    msg.reverse_front_right = msg.reverse_rear_right = direction > 0
    return msg


def finish_relay_write(relay):
    relay.writes[-1][2].finish()  # DATA_A
    relay.writes[-1][2].finish()  # DATA_B


@pytest.mark.parametrize('direction, reverse, expected', [(-1, False, (True, False, True, False)), (1, False, (False, True, False, True)), (0, True, (True,) * 4), (0, False, (False,) * 4)])
def test_relay_keeps_existing_pin_addresses_and_writes_each_direction(relay_node, direction, reverse, expected):
    relay = relay_node
    assert relay.reverse_pins == [1, 2, 3, 13]
    relay.relay_callback(relay_command(direction, reverse))
    finish_relay_write(relay)
    assert relay.last_reverse == expected
    data = relay.writes[-2][1] | (relay.writes[-1][1] << 8)
    assert [not bool(data & (1 << pin)) for pin in relay.reverse_pins] == list(expected)


@pytest.mark.parametrize('during_b', [False, True])
def test_relay_restore_while_write_pending_is_not_lost(relay_node, during_b):
    relay = relay_node
    relay.relay_callback(relay_command())
    finish_relay_write(relay)
    relay.relay_callback(relay_command(-1))
    if during_b:
        relay.writes[-1][2].finish()
    relay.relay_callback(relay_command())  # Son uygulanmisla ayni, bekleyenle farkli.
    assert relay.desired_reverse == (False,) * 4
    if during_b:
        relay.writes[-1][2].finish()
    else:
        finish_relay_write(relay)
    assert relay.active_write_state is not None
    finish_relay_write(relay)
    assert relay.last_reverse == (False,) * 4
    assert relay.pending_write_future is None


def test_relay_retries_after_partial_failed_write(relay_node):
    relay = relay_node
    relay.relay_callback(relay_command())
    finish_relay_write(relay)
    relay.relay_callback(relay_command(-1))
    relay.writes[-1][2].finish()
    relay.writes[-1][2].finish(False)
    count = len(relay.writes)
    relay.relay_callback(relay_command())
    assert len(relay.writes) == count + 1
    finish_relay_write(relay)
    assert relay.last_reverse == (False,) * 4


@pytest.fixture
def steer_node(motion_node, monkeypatch):
    module = import_hardware_node('steer_node', monkeypatch)
    steer = object.__new__(module.SteerNode)
    steer.max_steer_angle_deg = 30.0
    steer.pivot_max_steer_angle_deg = 45.0
    steer.pivot_active = False
    steer.steer_angle_safety_margin_deg = 5.0
    steer.deadband_deg = 2.0
    steer.malfunction_active = False
    steer.current_angles = [0] * 4
    steer.target_angles = [0] * 4
    steer.commands = []
    steer.errors = []
    steer.start_next_i2c_write = lambda: None
    steer.stop_all_motors = lambda: None
    steer.reset_all_motor_response_checks = lambda: None
    steer.check_motor_no_response = lambda *args: False
    steer.turn_left = lambda motor, error: steer.commands.append((motor, 'left'))
    steer.turn_right = lambda motor, error: steer.commands.append((motor, 'right'))
    steer.stop_motor = lambda motor: steer.commands.append((motor, 'stop'))
    steer.trigger_malfunction = steer.errors.append
    return steer


def steer_command(pivot):
    msg = Message()
    msg.pivot = pivot
    msg.stop_all = False
    msg.front_left = msg.rear_right = 45 if pivot else 0
    msg.front_right = msg.rear_left = -45 if pivot else 0
    return msg


def test_steer_allows_45_only_for_pivot_and_can_return_to_center(steer_node):
    steer = steer_node
    msg = steer_command(True)
    msg.pivot = False
    steer.steer_callback(msg)
    assert steer.target_angles == [30, -30, -30, 30]
    msg.pivot = True
    steer.steer_callback(msg)
    assert steer.target_angles == [45, -45, -45, 45]
    for motor, target in enumerate(steer.target_angles, 1):
        steer.steer_to_angle(motor, target)
    assert steer.commands == [(1, 'right'), (2, 'left'), (3, 'left'), (4, 'right')]
    steer.current_angles = [45, -45, -45, 45]
    steer.steer_callback(steer_command(False))
    assert steer.pivot_active and steer.target_angles == [0] * 4
    steer.commands.clear()
    for motor in range(1, 5):
        steer.steer_to_angle(motor, 0)
    assert not steer.errors
    assert steer.commands == [(1, 'left'), (2, 'right'), (3, 'right'), (4, 'left')]
    steer.current_angles = [0] * 4
    steer.steer_callback(steer_command(False))
    assert not steer.pivot_active


@pytest.mark.parametrize('pivot, angle', [(False, 36), (True, 51)])
def test_steer_physical_angle_guard_still_applies(steer_node, pivot, angle):
    steer_node.steer_callback(steer_command(pivot))
    steer_node.current_angles[0] = angle
    steer_node.steer_to_angle(1, 0)
    assert len(steer_node.errors) == 1 and not steer_node.commands


def test_steer_publishes_individual_calibrated_angles(steer_node, control_time):
    steer = steer_node
    steer.steer_callback(steer_command(True))
    steer.current_angles = [45, -45, -45, 45]
    steer.encoder_ready = True
    steer.last_encoder_update_sec = control_time[0]
    steer.encoder_timeout_sec = 0.5
    steer.encoder_malfunction_timeout = 5.0
    steer.average_steering_publisher = Publisher()
    steer.steering_angles_publisher = Publisher()
    steer.control_callback()
    assert steer.average_steering_publisher.messages[-1].data == 0
    assert steer.steering_angles_publisher.messages[-1].data == [45.0, -45.0, -45.0, 45.0]


def test_new_mission_cannot_launch_with_x_wheels_left_by_canceled_pivot(node):
    begin_turn(node)
    node.mission = None
    node.update()
    mission(node, [(0, 10)])
    for tick in range(5):
        node.update()
        assert node.pivot == 0 and node.steering == 0 and node.throttle == 0
        assert node.arm and not node.coast
    node.steering_angles = [0] * 4
    node.update()
    assert node.throttle > 0 and node.steering == 0
