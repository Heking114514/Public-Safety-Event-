"""Run after sourcing ROS and install/setup.bash, on Linux with /usr/bin/python3.
Uses a private PTY; never opens a robot serial device.
"""
import os
import pty
import re
import select
import subprocess
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from mission_control_interfaces.srv import SpeakText


class SpeechSerialPtyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def start_serial(self, protocol=None):
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.frames = []
        self.buffer = b""
        service = "/speech_pty_test/" + (protocol or "default")
        executable = os.path.join(
            get_package_prefix("cup_car_serial"), "lib", "cup_car_serial", "cmd_vel_serial_node"
        )
        executable = os.environ.get("SPEECH_TEST_EXECUTABLE", executable)
        command = [executable, "--ros-args", "-p", "device:=" + os.ttyname(self.slave),
                   "-p", "control_telemetry_timeout_s:=60.0", "-p", "speak_text_service:=" + service]
        if protocol is not None:
            command.extend(["-p", "speech_protocol:=" + protocol])
        self.process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.node = rclpy.create_node("speech_pty_test_" + (protocol or "default"))
        self.addCleanup(self.stop_serial)
        self.client = self.node.create_client(SpeakText, service)
        self.assertTrue(self.client.wait_for_service(timeout_sec=8))

    def stop_serial(self):
        self.node.destroy_node()
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        os.close(self.master)
        os.close(self.slave)

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if select.select([self.master], [], [], 0)[0]:
                self.buffer += os.read(self.master, 8192)
                while b"\n" in self.buffer:
                    line, self.buffer = self.buffer.split(b"\n", 1)
                    self.frames.append(line + b"\n")

    def speak(self, text="一"):
        request = SpeakText.Request()
        request.text = text
        return self.client.call_async(request)

    def status(self, ident, state):
        os.write(self.master, f"TTS,{ident},{state}\r\n".encode())

    def velocity_count(self):
        return sum(bool(re.fullmatch(rb"-?\d+\.\d{3},-?\d+\.\d{3}\r\n", line))
                   for line in self.frames)

    def test_default_preserves_old_wire_format_without_any_feedback(self):
        self.start_serial()  # Exercise the actual default, without a mode override.
        future = self.speak()
        self.pump(0.7)
        expected = b"TTS," + "一".encode("gbk") + b"\r\n"
        self.assertEqual([line for line in self.frames if line.startswith(b"TTS")], [expected])
        self.assertTrue(future.done())
        self.assertTrue(future.result().success)
        self.assertIn("queued", future.result().message)
        self.assertIn("no MCU acknowledgement", future.result().message)
        self.assertGreaterEqual(self.velocity_count(), 5)
        other = self.speak("二")
        self.pump(0.1)
        self.assertTrue(other.done())
        self.assertTrue(other.result().success)
        self.assertEqual([line for line in self.frames if line.startswith(b"TTS")], [expected])
        # 无 MCU 回执也立即返回；占用计时仅控制后台 FIFO 的发送次序。
        self.pump(2.0)
        self.assertTrue(future.done())
        self.assertTrue(future.result().success)
        self.assertIn("no MCU acknowledgement", future.result().message)
        self.assertEqual([line for line in self.frames if line.startswith(b"TTS")],
                         [expected, b"TTS," + "二".encode("gbk") + b"\r\n"])
        self.assertIsNone(self.process.poll())
        next_future = self.speak("三")
        self.pump(0.15)
        self.assertIn(b"TTS," + "二".encode("gbk") + b"\r\n", self.frames)
        self.assertTrue(next_future.done())
        self.assertTrue(next_future.result().success)
        self.assertNotIn(b"TTS," + "三".encode("gbk") + b"\r\n", self.frames)

    def test_long_legacy_text_and_busy_queue_return_without_waiting_for_playback(self):
        self.start_serial()
        future = self.speak("这是一段用于验证长文本不会延长小车驻停时间的播报")
        self.pump(0.3)
        self.assertTrue(future.done())
        self.assertTrue(future.result().success)
        queued = self.speak("到达一号巡逻点")
        self.pump(0.3)
        self.assertTrue(queued.done())
        self.assertTrue(queued.result().success)
        self.assertEqual(len([line for line in self.frames if line.startswith(b"TTS,")]), 1)
        self.assertGreaterEqual(self.velocity_count(), 5)

    def test_explicit_tts2_retries_and_waits_for_matching_status(self):
        self.start_serial("tts2")
        future = self.speak()
        self.pump(0.65)
        sent = [line for line in self.frames if line.startswith(b"TTS2,")]
        self.assertGreaterEqual(len(sent), 2)
        self.assertEqual(len(set(sent)), 1)
        ident = int(sent[0].split(b",", 2)[1])
        self.assertFalse(future.done())
        self.assertGreaterEqual(self.velocity_count(), 5)
        self.status(ident + 1 if ident < 4294967295 else 1, "ESTIMATED_DONE")
        self.pump(0.1)
        self.assertFalse(future.done())
        self.status(ident, "PLAYING")
        self.pump(0.15)
        count = sum(line.startswith(b"TTS2,") for line in self.frames)
        self.pump(0.3)
        self.assertEqual(sum(line.startswith(b"TTS2,") for line in self.frames), count)
        self.assertFalse(future.done())
        self.status(ident, "ESTIMATED_DONE")
        self.pump(0.2)
        self.assertTrue(future.done())
        self.assertTrue(future.result().success)
        self.frames.clear()
        failed = self.speak()
        self.pump(0.15)
        new_id = int(next(line for line in self.frames if line.startswith(b"TTS2,")).split(b",", 2)[1])
        self.assertNotEqual(new_id, ident)
        self.status(new_id, "BUSY")
        self.pump(0.2)
        self.assertTrue(failed.done())
        self.assertFalse(failed.result().success)


if __name__ == "__main__":
    unittest.main()
