#!/usr/bin/env python3
"""离线验证 MC02 固件控制函数和 ROS 里程计配置的方向约定。"""

import argparse
import ctypes
from pathlib import Path
import re
import subprocess
import tempfile

import yaml


def function(source, name):
    match = re.search(r"(?:static )?(?:void|float|int) " + name + r"\([^)]*\)\s*\{", source)
    if match is None:
        raise ValueError(f"找不到固件函数 {name}")
    end = match.end()
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def defines(source, names):
    lines = source.splitlines()
    result = []
    for name in names:
        index = next(i for i, line in enumerate(lines)
                     if re.match(r"#define\s+" + name + r"\b", line))
        while True:
            result.append(lines[index])
            if not lines[index].rstrip().endswith("\\"):
                break
            index += 1
    return "\n".join(result) + "\n"


def build_controls(firmware, temporary):
    cmd = (firmware / "application/cmd/robot_cmd.c").read_text()
    chassis = (firmware / "application/chassis/chassis.c").read_text()
    protocol = (firmware / "modules/master_machine/master_process.c").read_text()
    robot = (firmware / "application/robot_def.h").read_text()
    macros = defines(robot, [
        "TRACK_WIDTH", "RADIUS_WHEEL", "MOTOR_ENCODER_COUNTS_PER_REV",
        "LEFT_ENCODER_COUNTS_PER_WHEEL_REV", "RIGHT_ENCODER_COUNTS_PER_WHEEL_REV",
        "LEFT_REDUCTION_RATIO_WHEEL", "RIGHT_REDUCTION_RATIO_WHEEL",
        "CHASSIS_YAW_HOLD_DIR", "CHASSIS_YAW_HOLD_KP", "CHASSIS_YAW_HOLD_KI",
        "CHASSIS_YAW_HOLD_KD", "CHASSIS_YAW_HOLD_DEADBAND",
        "CHASSIS_YAW_HOLD_ENCODER_ASSIST_MAX_DEG", "CHASSIS_YAW_HOLD_ENCODER_ASSIST_GAIN",
        "CHASSIS_YAW_HOLD_MAX_RATIO", "CHASSIS_YAW_HOLD_MAX_REF",
        "CHASSIS_YAW_HOLD_I_MAX_RATIO", "CHASSIS_YAW_HOLD_I_MAX_REF",
    ])
    macros += defines(cmd, ["NAV_LEFT_REF_PER_MPS", "NAV_RIGHT_REF_PER_MPS"])
    macros += defines(chassis, [
        "CHASSIS_MOTION_REF_EPSILON", "CHASSIS_STRAIGHT_W_EPSILON",
        "CHASSIS_IMU_MAX_AGE_MS", "CHASSIS_HEADING_FUSION_TAU_S",
        "CHASSIS_WHEEL_REF_LIMIT", "CHASSIS_INNER_WHEEL_MIN_REF",
    ])
    # 只替代硬件读数与任务状态；被测映射、融合、PID 和限幅均取真实固件函数。
    stubs = r'''
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include "bmi088_diag.h"
#define ROBOT_USE_REMOTE_TEST_MODE 0
#define CHASSIS_USE_INS 0
#define PI 3.14159265358979323846f
static struct { float v, w; } input, chassis_cmd_send;
#define vision_recv_data (&input)
static BMI088DiagState test_imu;
static uint32_t tick;
static uint8_t straight_tracking, heading_tracking;
static int8_t heading_direction;
static float heading_ref_deg, heading_i_term, heading_fused_error_deg;
static float heading_imu_error_deg, heading_last_error, heading_last_correction;
static float straight_last_phase_error_deg, straight_last_correction;
static uint32_t heading_last_ms;
typedef struct { int32_t left_total_count, right_total_count; } Chassis_Host_Feedback_s;
uint32_t HAL_GetTick(void) { return tick; }
uint8_t BMI088DiagGetState(BMI088DiagState *state) { *state = test_imu; return 1; }
'''
    wrappers = r'''
void map_command(float v, float w, int32_t *left, int32_t *right,
                 float *vehicle_v, float *vehicle_w)
{
    input.v = v; input.w = w;
    NavigationControlSet();
    float turn = chassis_cmd_send.w;
    ChassisLimitInsideWheel(chassis_cmd_send.v, &turn);
    float left_ref = chassis_cmd_send.v - turn;
    float right_ref = chassis_cmd_send.v + turn;
    ChassisLimitWheelRefs(&left_ref, &right_ref);
    float logical_left = left_ref / NAV_LEFT_REF_PER_MPS;
    float logical_right = right_ref / NAV_RIGHT_REF_PER_MPS;
    /* 独立于 ENC 轮序，以用户实测的内部正差速右转约定检查执行方向。 */
    *vehicle_v = (logical_left + logical_right) * 0.5f;
    *vehicle_w = (logical_left - logical_right) / (TRACK_WIDTH / 1000.0f);
    /* 模拟电机完全跟踪目标一秒，使用固件自身分辨率得到逻辑通道计数。 */
    Chassis_Host_Feedback_s feedback;
    feedback.left_total_count = (int32_t)lroundf(
        left_ref * MOTOR_ENCODER_COUNTS_PER_REV / 360.0f);
    feedback.right_total_count = (int32_t)lroundf(
        right_ref * MOTOR_ENCODER_COUNTS_PER_REV / 360.0f);
    char frame[96];
    HostAppendEncoder(frame, sizeof(frame), 2000, 2, &feedback);
    unsigned now, sequence;
    long first, second;
    assert(sscanf(frame, "ENC,%u,%u,%ld,%ld", &now, &sequence, &first, &second) == 4);
    *left = (int32_t)first; *right = (int32_t)second;
}
float heading_correction(float v, float yaw, float gyro, float reference, uint32_t steps)
{
    straight_tracking = 1; heading_tracking = 0;
    tick = 1000;
    test_imu = (BMI088DiagState){0};
    test_imu.status = BMI088_DIAG_YAW_VALID | BMI088_DIAG_BIAS_VALID;
    test_imu.last_update_ms = tick; test_imu.yaw_deg = reference;
    ChassisApplyHeadingAssist(v * NAV_LEFT_REF_PER_MPS, 0);
    test_imu.yaw_deg = yaw; test_imu.gyro_z_lpf_dps = gyro;
    /* 实车已确认内部正差速使车右转：车体 yaw = -逻辑轮差 * R/b。 */
    straight_last_phase_error_deg = -ChassisWrapDeg(yaw - reference) * TRACK_WIDTH / RADIUS_WHEEL;
    float correction = 0;
    for (uint32_t i = 0; i < steps; ++i) {
        tick += 10; test_imu.last_update_ms = tick;
        correction = -ChassisApplyHeadingAssist(v * NAV_LEFT_REF_PER_MPS, 0);
    }
    return correction;
}
'''
    functions = "\n".join([
        function(cmd, "NavigationControlSet"),
        function(chassis, "ChassisClamp"), function(chassis, "ChassisWrapDeg"),
        function(chassis, "ChassisLimitInsideWheel"), function(chassis, "ChassisLimitWheelRefs"),
        function(chassis, "ChassisApplyHeadingAssist"),
        function(protocol, "HostAppendEncoder"),
    ])
    source = temporary / "controls.c"
    source.write_text(stubs + macros + functions + wrappers)
    library = temporary / "controls.so"
    subprocess.run([
        "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
        "-I" + str(firmware / "modules/imu"), str(source), "-lm", "-o", str(library),
    ], check=True)
    controls = ctypes.CDLL(str(library))
    controls.map_command.argtypes = [ctypes.c_float, ctypes.c_float,
                                    ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32),
                                    ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
    controls.map_command.restype = None
    controls.heading_correction.argtypes = [ctypes.c_float] * 4 + [ctypes.c_uint32]
    controls.heading_correction.restype = ctypes.c_float
    return controls


def build_odometry(workspace, physical, temporary):
    # 所有物理值来自待验证的 YAML；执行真实积分器，包含左右独立分辨率与轴偏移。
    drive = physical["differential_drive"]
    fields = {
        "wheel_radius_m": "wheel_radius_m", "wheel_track_m": "wheel_track_m",
        "left_encoder_counts_per_revolution": "left_encoder_counts_per_revolution",
        "right_encoder_counts_per_revolution": "right_encoder_counts_per_revolution",
        "left_distance_scale": "left_distance_scale", "right_distance_scale": "right_distance_scale",
        "yaw_slip_scale": "yaw_slip_scale", "max_wheel_speed_mps": "max_wheel_speed_mps",
        "base_offset_x_m": "base_link_offset_x_m", "base_offset_y_m": "base_link_offset_y_m",
    }
    assignments = "\n".join(f"config.{key} = {float(drive[value])!r};"
                            for key, value in fields.items())
    source = temporary / "odometry.cpp"
    source.write_text('''
#include <iostream>
#include <iomanip>
#include "wheel_odometry/odometry_integrator.hpp"
int main() {
    wheel_odometry::IntegratorConfig config;
''' + assignments + '''
    config.nominal_sample_period_s = 0.05;
    config.nominal_sequence_increment = 1;
    int32_t left, right;
    while (std::cin >> left >> right) {
        wheel_odometry::OdometryIntegrator integrator(config);
        integrator.update({1000, 1, 0, 0});
        if (!integrator.update({2000, 2, left, right}).publish) return 1;
        std::cout << std::setprecision(12) << integrator.state().linear_velocity_mps
                  << ' ' << integrator.state().angular_velocity_radps << '\\n';
    }
}
''')
    package = workspace / "src/wheel_odometry"
    binary = temporary / "odometry"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
        "-I" + str(package / "include"), str(source),
        str(package / "src/odometry_integrator.cpp"), "-o", str(binary),
    ], check=True)
    return binary


def verify(firmware, workspace):
    physical = yaml.safe_load((workspace / "src/vehicle_config/config/vehicle_physical.yaml").read_text())
    with tempfile.TemporaryDirectory(prefix="mc02-direction-") as directory:
        temporary = Path(directory)
        controls = build_controls(firmware, temporary)
        odometry = build_odometry(workspace, physical, temporary)
        commands, counts = [], []
        for v in (-0.08, 0.0, 0.08):
            # 选取限速范围内的命令，验证完整轮速限幅链路后的幅值。
            for w in (-0.5, -0.25, 0.0, 0.25, 0.5):
                left, right = ctypes.c_int32(), ctypes.c_int32()
                vehicle_v, vehicle_w = ctypes.c_float(), ctypes.c_float()
                controls.map_command(v, w, ctypes.byref(left), ctypes.byref(right),
                                     ctypes.byref(vehicle_v), ctypes.byref(vehicle_w))
                assert abs(vehicle_v.value - v) < 2e-5, (v, w, vehicle_v.value)
                assert abs(vehicle_w.value - w) < 2e-5, (v, w, vehicle_w.value)
                commands.append((v, w))
                counts.append(f"{left.value} {right.value}\n")
        result = subprocess.run([str(odometry)], input="".join(counts),
                                check=True, text=True, capture_output=True)
        rows = result.stdout.splitlines()
        assert len(rows) == len(commands)
        for (v, w), row in zip(commands, rows):
            measured_v, measured_w = map(float, row.split())
            assert abs(measured_v - v) < 2e-5, (v, w, measured_v)
            assert abs(measured_w - w) < 2e-5, (v, w, measured_w)
        heading_cases = 0
        for v in (-0.08, 0.08):
            # 左右偏航、纯角速度扰动及跨越 +/-180 度的误差。
            for yaw, gyro, reference, expected_sign in [
                (5, 0, 0, -1), (-5, 0, 0, 1), (0, 5, 0, -1), (0, -5, 0, 1),
                (-179, 0, 179, -1), (179, 0, -179, 1),
            ]:
                for steps in (1, 1500):
                    correction = controls.heading_correction(v, yaw, gyro, reference, steps)
                    assert correction * expected_sign > 0, (v, yaw, gyro, reference, steps, correction)
                    heading_cases += 1
            for steps in (1, 1500):
                assert controls.heading_correction(v, 0, 0, 0, steps) == 0
        print(f"通过：{len(commands)} 组速度命令经真实控制、ENC 上报与里程计积分后方向、幅值一致")
        print(f"通过：{heading_cases} 组前进/倒车瞬时及持续航向纠偏与 4 组零偏差检查")
        print("本测试假设电机跟踪目标，不替代实车轮向与 IMU 正方向验证。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", type=Path, required=True, help="MC02 固件源码目录")
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    verify(args.firmware.resolve(), args.workspace.resolve())
