#!/usr/bin/env python3
"""隔离验收：调用真实 C++ 控制器，验证柔性纠偏和简化闭环响应。

模型仅用于软件回归：30 Hz、100 ms 指令延迟、160 ms 一阶响应。
不模拟轮胎接触、人工扶车、定位误差，结果不代表实车安全验收。
所有编译产物放在 TemporaryDirectory，退出自动清理，无 ROS/串口操作。
"""

import ctypes
import argparse
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
DT = 1.0 / 30.0
PARAMETERS = (
    "path_pid_kp", "path_pid_ki", "path_pid_kd", "path_yaw_rate_damping",
    "path_pid_integral_limit", "max_path_angular_speed", "cross_track_gain",
    "stanley_softening_speed", "max_cross_track_correction",
    "cross_track_slowdown_start", "cross_track_slowdown_full",
    "cross_track_minimum_speed", "pre_turn_path_angular_ratio",
    "path_adaptive_max_angular_ratio", "path_adaptive_heading_start",
    "path_adaptive_heading_full", "path_adaptive_minimum_speed",
    "path_adaptive_gain_scale", "path_angular_activation_error",
    "path_yaw_response_threshold", "min_path_angular_speed",
    "max_linear_acceleration", "max_linear_deceleration",
    "max_angular_acceleration", "max_angular_deceleration",
    "effective_braking_deceleration", "braking_control_delay",
    "braking_safety_margin", "path_adaptive_braking_blend_distance",
    "min_linear_command_speed", "max_linear_speed",
    "angular_gain", "turn_yaw_rate_damping",
)

HARNESS = r'''
#include "visual_navigation/adaptive_path.hpp"
#include "visual_navigation/path_control.hpp"
#include "visual_navigation/path_tracking_controller.hpp"
#include "visual_navigation/rate_limiter.hpp"
using namespace visual_navigation;
struct State {
  PathTrackingController pid;
  double p[33];
  double previous_v = 0.0;
  double previous_w = 0.0;
  int ticks = 0;
};
extern "C" {
void *create(const double *p) {
  State *s = new State;
  std::copy(p, p + 33, s->p);
  s->pid.configure(p[0], p[1], p[2], p[3], p[4], p[5]);
  return s;
}
void destroy(void *state) {delete static_cast<State *>(state);}
double blend(double error, double start, double full) {
  return AdaptiveErrorBlend(error, start, full);
}
void probe(const double *p, double speed, double heading, double cross,
           double brake, double *output) {
  output[0] = AdaptiveCorrectionSpeedLimit(speed, heading, cross,
    p[14], p[15], p[9], p[10], p[16]);
  output[1] = AdaptivePathAngularSpeed(1.0, output[0], heading, cross,
    p[12], p[13], p[14], p[15], p[9], p[10], brake);
}
void step(void *state, int adaptive, double heading, double cross,
          double yaw_rate, double requested_speed, int direction,
          double remaining, int scan, double *output) {
  State &s = *static_cast<State *>(state);
  const double *p = s.p;
  const double dt = 1.0 / 30.0;
  double v = scan ? requested_speed : CrossTrackSpeedLimit(
    requested_speed, std::abs(cross), p[9], p[10], p[11]);
  if (adaptive) v = AdaptiveCorrectionSpeedLimit(v, heading, cross,
    p[14], p[15], p[9], p[10], p[16]);
  if (remaining >= 0.0 && !scan)
    v = BrakingSpeedLimit(v, remaining, p[25], p[26], p[27]);
  const double signed_v = direction * v;
  const double tracking = std::abs(LimitRate(signed_v, s.previous_v,
    p[21], p[22], dt));
  const double control_cross = adaptive ? cross : direction * cross;
  const double error = StanleyPathError(heading, control_cross,
    p[6], tracking, p[7], p[8]);
  const double scale = adaptive ? 1.0 + (p[17] - 1.0) *
    AdaptiveErrorBlend(std::abs(error), p[14], p[15]) : 1.0;
  const auto time = PathTrackingController::TimePoint{} +
    std::chrono::nanoseconds(static_cast<long long>(s.ticks++ * dt * 1.0e9));
  double w = s.pid.update_pid(error, control_cross,
    std::abs(heading) < .30 && std::abs(cross) < .08,
    yaw_rate, time, scale);
  v *= scan ? 1.0 : std::max(0.0, std::cos(error));
  w = adaptive
    ? AdaptivePathDeadband(w, error, yaw_rate, p[18], p[19], p[20], p[5])
    : CompensatePathTurnDeadband(w, error, yaw_rate,
        p[18], p[19], p[20], p[5]);
  double brake = 0.0;
  if (remaining >= 0.0)
    brake = AdaptiveErrorBlend(BrakingDistance(std::abs(s.previous_v),
      p[25], p[26], p[27]) + p[28] - remaining, 0.0, p[28]);
  w = adaptive ? AdaptivePathAngularSpeed(w, direction * v, heading,
    control_cross, p[12], p[13], p[14], p[15], p[9], p[10], brake)
    : StopRequiredPathAngularSpeed(w, direction * v, p[12]);
#ifdef CONTRAST_BASELINE
  if (scan && !adaptive) w = std::clamp(
    p[31] * heading - p[32] * yaw_rate, -std::min(p[5], .25), std::min(p[5], .25));
#endif
  const double target_v = std::clamp(direction * v, -p[30], p[30]);
  s.previous_v = LimitRate(target_v, s.previous_v, p[21], p[22], dt);
  if (std::abs(target_v) > 1.0e-9 && std::abs(s.previous_v) < p[29])
    s.previous_v = std::copysign(p[29], target_v);
  else if (std::abs(target_v) <= 1.0e-9 && std::abs(s.previous_v) < p[29])
    s.previous_v = 0.0;
  s.previous_w = LimitRate(w, s.previous_w, p[23], p[24], dt);
  output[0] = s.previous_v;
  output[1] = s.previous_w;
  output[2] = error;
}
}
'''


class Controller:
    def __init__(self, library, parameters):
        self.lib = library
        self.parameters = (ctypes.c_double * len(parameters))(*parameters)
        self.state = library.create(self.parameters)

    def close(self):
        self.lib.destroy(self.state)

    def step(self, adaptive, heading, cross, yaw_rate, speed=.05,
             direction=1, remaining=-1., scan=False):
        output = (ctypes.c_double * 3)()
        self.lib.step(self.state, adaptive, heading, cross, yaw_rate,
                      speed, direction, remaining, scan, output)
        return tuple(output)

    def probe(self, speed, heading, cross, brake=0.):
        output = (ctypes.c_double * 2)()
        self.lib.probe(self.parameters, speed, heading, cross, brake, output)
        return tuple(output)


def simulate(library, parameters, adaptive, cross, heading, *,
             duration=40., direction=1, scan=False, bias=0., delay=3, tau=.16):
    controller = Controller(library, parameters)
    initial_cross = cross
    yaw = -heading
    velocity = rate = distance = 0.
    queue = [(0., 0.)] * delay
    rows = []
    try:
        for tick in range(round(duration / DT)):
            requested = controller.step(adaptive, -yaw, cross, rate,
                                        .035 if scan else .05, direction,
                                        scan=scan)
            queue.append(requested[:2])
            target_v, target_w = queue.pop(0)
            velocity += DT / tau * (target_v - velocity)
            rate += DT / tau * (target_w + bias - rate)
            yaw += rate * DT
            # yaw 是相对期望车身朝向的误差。倒车时车身朝向=路线+pi，
            # 因此路线坐标中的横向速度=-v*sin(yaw)，不能漏掉这次换基。
            body_yaw = yaw + (math.pi if direction < 0 else 0.)
            cross += velocity * math.sin(body_yaw) * DT
            distance += abs(velocity * math.cos(yaw)) * DT
            rows.append((tick * DT, cross, yaw, requested[0], requested[1], distance))
    finally:
        controller.close()
    lateral = [row[1] for row in rows]
    settled = next((row[0] for index, row in enumerate(rows)
                    if abs(row[1]) <= .01 and
                    all(abs(later) <= .01 for later in lateral[index:])), None)
    overshoot = max((max(0., -value * math.copysign(1., initial_cross or -heading))
                     for value in lateral), default=0.)
    return {
        "settle_s": settled,
        "final_cross_m": rows[-1][1],
        "peak_cross_m": max(abs(value) for value in lateral),
        "integral_error_m_s": sum(abs(value) for value in lateral) * DT,
        "maximum_w_rad_s": max(abs(row[4]) for row in rows),
        "distance_m": distance,
        "overshoot_m": overshoot,
        "rows": rows,
    }


class FlexibleTrackingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="flexible_tracking_")
        temporary = Path(cls.temporary.name)
        source = temporary / "harness.cpp"
        source.write_text(HARNESS)
        cls.variants = []
        cls.results = {}
        for package, config in (
            ("visual_navigation", "visual_navigation/config/waypoint_navigation.yaml"),
            ("contrast_navigation_core", "contrast_controller/config/contrast_controller.yaml"),
        ):
            output = temporary / (package + ".so")
            flags = ["-DCONTRAST_BASELINE"] if package == "contrast_navigation_core" else []
            subprocess.run(["g++", "-std=c++17", "-O2", "-shared", "-fPIC", *flags,
                            "-Wall", "-Wextra", "-Werror",
                            "-I", str(WORKSPACE / "src" / package / "include"),
                            str(source), "-o", str(output)], check=True)
            lib = ctypes.CDLL(str(output))
            double = ctypes.c_double
            array = ctypes.POINTER(double)
            lib.create.argtypes = [array]
            lib.create.restype = ctypes.c_void_p
            lib.destroy.argtypes = [ctypes.c_void_p]
            lib.step.argtypes = [ctypes.c_void_p, ctypes.c_int, double, double,
                                 double, double, ctypes.c_int, double,
                                 ctypes.c_int, array]
            lib.probe.argtypes = [array, double, double, double, double, array]
            lib.blend.argtypes = [double, double, double]
            lib.blend.restype = double
            data = yaml.safe_load((WORKSPACE / "src" / config).read_text())
            base = yaml.safe_load((WORKSPACE / "src/visual_navigation/config/"
                                   "waypoint_navigation.yaml").read_text())
            params = dict(next(iter(base.values()))["ros__parameters"])
            # Contrast launch 先加载正式导航参数，再覆盖专用配置。
            params.update(next(iter(data.values()))["ros__parameters"])
            params["path_adaptive_minimum_speed"] = min(
                params["max_linear_speed"], max(params["min_linear_command_speed"],
                    params["path_adaptive_minimum_speed"]))
            cls.variants.append((package, lib, [params[key] for key in PARAMETERS]))

    @classmethod
    def tearDownClass(cls):
        print("\n闭环验收结果（软件模型，不是实车）：")
        print(json.dumps(cls.results, ensure_ascii=False, indent=2))
        cls.temporary.cleanup()

    def test_continuous_symmetric_monotonic_envelope(self):
        for package, lib, parameters in self.variants:
            controller = Controller(lib, parameters)
            try:
                for axis in ("heading", "cross"):
                    previous_v, previous_w = .05, parameters[12] * .05
                    full = .25 if axis == "heading" else .06
                    for index in range(501):
                        error = full * index / 500.
                        heading, cross = (error, 0.) if axis == "heading" else (0., error)
                        v, w = controller.probe(.05, heading, cross)
                        opposite = controller.probe(.05, -heading, -cross)
                        with self.subTest(package=package, axis=axis, error=error):
                            self.assertAlmostEqual(v, opposite[0], places=12)
                            self.assertAlmostEqual(w, opposite[1], places=12)
                            self.assertLessEqual(v, previous_v + 1e-12)
                            self.assertGreaterEqual(w, previous_w - 1e-12)
                            self.assertLess(abs(w - previous_w), .004)
                        previous_v, previous_w = v, w
                self.assertEqual(controller.probe(0., .3, .1), (0., 0.))
                near_stop = controller.probe(.02, .3, .1, 1.)
                self.assertAlmostEqual(near_stop[1], parameters[12] * .02)
                self.assertLessEqual(near_stop[0], .02)
            finally:
                controller.close()

    def test_closed_loop_disturbance_and_direction(self):
        for package, lib, parameters in self.variants:
            self.results[package] = {}
            for name, cross, heading, direction, scan in (
                ("小偏差", .005, .02, 1, False),
                ("横向偏差", .06, 0., 1, False),
                ("航向偏差", 0., .20, 1, False),
                ("出弯偏差", .05, -.15, 1, False),
                ("正向扫描", .04, .08, 1, True),
                ("反向扫描", .04, .08, -1, True),
            ):
                old = simulate(lib, parameters, False, cross, heading,
                               direction=direction, scan=scan)
                new = simulate(lib, parameters, True, cross, heading,
                               direction=direction, scan=scan)
                mirror = simulate(lib, parameters, True, -cross, -heading,
                                  direction=direction, scan=scan)
                self.results[package][name] = {
                    "旧": {key: value for key, value in old.items() if key != "rows"},
                    "新": {key: value for key, value in new.items() if key != "rows"},
                }
                with self.subTest(package=package, case=name):
                    self.assertLess(abs(new["final_cross_m"]), .01)
                    # 现有 LimitRate 的正/负变化上限不同，闭环不能严格镜像。
                    self.assertLess(abs(new["final_cross_m"] +
                                        mirror["final_cross_m"]), .00005)
                    self.assertLess(new["peak_cross_m"], .08)
                    if name not in ("小偏差", "正向扫描"):
                        self.assertLess(new["integral_error_m_s"],
                                        old["integral_error_m_s"])
                    elif name == "正向扫描":
                        self.assertLess(new["integral_error_m_s"],
                                        old["integral_error_m_s"] * 1.05)
                    rows = new["rows"]
                    for previous, current in zip(rows, rows[1:]):
                        self.assertLessEqual(current[4] - previous[4],
                                             parameters[23] * DT + 1e-12)
                        self.assertLessEqual(previous[4] - current[4],
                                             parameters[24] * DT + 1e-12)
            self.assertEqual(len(self.results[package]), 6)

    def test_response_delay_and_bias(self):
        for package, lib, parameters in self.variants:
            for delay, tau, bias in ((3, .16, .015), (6, .30, 0.), (9, .40, 0.)):
                with self.subTest(package=package, delay=delay, tau=tau, bias=bias):
                    new = simulate(lib, parameters, True, .05, -.15,
                                   delay=delay, tau=tau, bias=bias)
                    self.assertLess(abs(new["final_cross_m"]), .015)
                    self.assertLess(new["peak_cross_m"], .06)
                    self.assertLess(new["overshoot_m"], .008)

    def test_braking_and_scan_direction(self):
        for package, lib, parameters in self.variants:
            for direction in (1, -1):
                controller = Controller(lib, parameters)
                try:
                    command = controller.step(True, 0., .04, 0., .035,
                                              direction, .02, True)
                    with self.subTest(package=package, direction=direction):
                        self.assertGreater(command[0] * direction, 0.)
                        # cross_track 相对路线前进方向定义；正误差都应右转。
                        self.assertLess(command[1], 0.)
                    for _ in range(30):
                        command = controller.step(True, .20, .04, 0., 0.,
                                                  direction, 0., False)
                    self.assertEqual(command[:2], (0., 0.))
                finally:
                    controller.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="保存模型结果 JSON 的位置")
    args, extra = parser.parse_known_args()
    result = unittest.main(argv=[__file__, *extra], verbosity=2, exit=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(FlexibleTrackingTest.results,
                                         ensure_ascii=False, indent=2))
    raise SystemExit(0 if result.result.wasSuccessful() else 1)
