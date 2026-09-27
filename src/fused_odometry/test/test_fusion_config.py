from pathlib import Path
import re

import yaml


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "fused_odometry.yaml"
WHEEL_TOPIC = "/fusion/input/wheel_odom"
VISUAL_TOPIC = "/fusion/input/visual_odom"
CONTROL_IMU_TOPIC = "/imu/control"


def load_parameters(node_name):
    document = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    return document[node_name]["ros__parameters"]


def test_default_ekf_uses_wheel_body_velocity_without_visual_translation():
    parameters = load_parameters("fused_ekf")
    sensor_topics = {
        name: value
        for name, value in parameters.items()
        if re.fullmatch(r"(?:odom|pose|twist|imu)\d+", name)
    }

    assert WHEEL_TOPIC in sensor_topics.values()
    assert VISUAL_TOPIC not in sensor_topics.values()
    assert parameters["odom0"] == WHEEL_TOPIC
    assert parameters["odom0_config"] == [
        False,
        False,
        False,
        False,
        False,
        False,
        True,
        True,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
    ]


def test_wheel_stream_remains_available_to_supervision():
    parameters = load_parameters("fused_odometry_gate")

    assert parameters["wheel_topic"] == "/wheel/odom"
    assert parameters["wheel_output_topic"] == WHEEL_TOPIC


def test_control_imu_is_single_corrected_feedback_topic():
    gate = load_parameters("fused_odometry_gate")
    ekf = load_parameters("fused_ekf")

    assert gate["imu_topic"] == "/cup_car_serial/bmi088_attitude"
    assert gate["imu_output_topic"] == CONTROL_IMU_TOPIC
    assert gate["imu_expected_frame"] == "base_link"
    assert ekf["imu0"] == CONTROL_IMU_TOPIC


def test_visual_correction_does_not_bypass_smoothing_by_default():
    parameters = load_parameters("map_odom_correction")
    assert parameters["use_absolute_visual_correction"] is False
    assert parameters["direct_visual_tracking"] is False
    assert parameters["correction_time_constant_s"] >= 1.0
    assert parameters["stationary_correction_time_constant_s"] < parameters[
        "correction_time_constant_s"
    ]
    assert parameters["visual_recovery_time_constant_s"] >= 1.0
    assert parameters["max_correction_step_m"] <= 0.05
    assert parameters["max_correction_yaw_step_rad"] <= 0.05
    assert parameters["moving_max_correction_step_m"] < parameters[
        "max_correction_step_m"
    ]
    assert parameters["moving_max_correction_yaw_step_rad"] < parameters[
        "max_correction_yaw_step_rad"
    ]
    assert parameters["visual_recovery_max_correction_step_m"] <= parameters[
        "moving_max_correction_step_m"
    ]
    assert parameters["visual_recovery_max_correction_yaw_step_rad"] <= parameters[
        "moving_max_correction_yaw_step_rad"
    ]


def test_local_map_odometry_publishes_map_frame_bridge():
    parameters = load_parameters("local_map_odometry")

    assert parameters["input_topic"] == "/odometry/local"
    assert parameters["output_topic"] == "/odometry/local_map"
    assert parameters["input_frame"] == "odom"
    assert parameters["output_frame"] == "map"
    assert parameters["base_frame"] == "base_link"
    assert parameters["publish_tf"] is False


def test_vehicle_center_offset_is_preserved_during_turn_hold():
    parameters = load_parameters("map_odom_correction")
    assert parameters["hold_global_xy_during_turn"] is True
    assert parameters["turn_hold_max_linear_speed_mps"] >= 0.05


def test_visual_lateral_velocity_is_not_fused_into_local_ekf():
    parameters = load_parameters("fused_ekf")
    assert parameters["odom0_config"][6:9] == [True, True, False]


def test_wheel_distance_has_stronger_weight_than_visual_speed():
    parameters = load_parameters("fused_odometry_gate")
    assert parameters["wheel_vx_variance"] < parameters["visual_vx_variance"]
    assert parameters["wheel_vy_variance"] < parameters["visual_vy_variance"]
    assert parameters["wheel_base_offset_x_m"] == 0.05


def test_moving_arc_is_not_classified_as_in_place_turn():
    parameters = load_parameters("fused_odometry_gate")

    assert parameters["wheel_in_place_max_linear_speed_mps"] <= 0.02
    assert parameters["wheel_in_place_max_linear_speed_mps"] < 0.06
    assert parameters["wheel_in_place_min_yaw_rate_radps"] <= 0.07
