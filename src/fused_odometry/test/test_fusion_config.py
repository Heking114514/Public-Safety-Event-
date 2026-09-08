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


def test_default_ekf_excludes_encoder_measurements():
    parameters = load_parameters("fused_ekf")
    sensor_topics = {
        name: value
        for name, value in parameters.items()
        if re.fullmatch(r"(?:odom|pose|twist|imu)\d+", name)
    }

    assert WHEEL_TOPIC not in sensor_topics.values()
    assert parameters["odom0"] == VISUAL_TOPIC
    assert "odom1" not in parameters
    assert parameters["odom0_config"] == [
        False,
        False,
        False,
        False,
        False,
        False,
        True,
        False,
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

    assert gate["imu_topic"] == "/imu/filtered"
    assert gate["imu_output_topic"] == CONTROL_IMU_TOPIC
    assert ekf["imu0"] == CONTROL_IMU_TOPIC


def test_visual_correction_does_not_bypass_smoothing_by_default():
    parameters = load_parameters("map_odom_correction")
    assert parameters["direct_visual_tracking"] is False
