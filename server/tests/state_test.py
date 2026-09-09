import copy
import unittest

from pydantic import ValidationError

from ros2_agent_server.models import RobotConfig
from ros2_agent_server.protocol import BridgeError, validate_request
from ros2_agent_server.state import StateStore
from helpers import arm_state, config, jpeg


class StateTest(unittest.TestCase):
    def setUp(self):
        self.config = config("dual_arm.json")
        self.now = 10.0
        self.store = StateStore(self.config, clock=lambda: self.now)

    def test_state_requires_all_arms_and_expires(self):
        with self.assertRaises(BridgeError) as missing:
            self.store.state(10)
        self.assertEqual(503, missing.exception.status)
        self.store.update_arm("left", arm_state())
        with self.assertRaises(BridgeError):
            self.store.state(10)
        self.store.update_arm("right", arm_state())
        result = self.store.state(1_000_000_123)
        self.assertEqual(["left", "right"], [arm["id"] for arm in result["arms"]])
        self.assertEqual({"sec": 1, "nanosec": 123}, result["stamp"])
        result["arms"][0]["gripper"]["opening"] = 0
        self.assertEqual(1, self.store.state(10)["arms"][0]["gripper"]["opening"])
        self.now += 2.1
        with self.assertRaises(BridgeError):
            self.store.state(10)

    def test_invalid_state_does_not_replace_last_valid_sample(self):
        self.store.update_arm("left", arm_state())
        state = arm_state()
        state["flange_pose"]["orientation"] = [0, 0, 0, 0]
        with self.assertRaises(ValidationError):
            self.store.update_arm("left", state)
        self.assertEqual(1, self.store._arms["left"][0]["flange_pose"]["orientation"][3])

    def test_camera_requires_new_sequence_and_capture_time(self):
        self.store.update_camera("overhead", jpeg(), "overhead_optical", 100)
        self.assertIsNone(self.store.fresh_camera("overhead", 1, 100))
        self.store.update_camera("overhead", jpeg("blue"), "overhead_optical", 101)
        self.assertIsNone(self.store.fresh_camera("overhead", 1, 102))
        self.assertIsNotNone(self.store.fresh_camera("overhead", 1, 101))
        self.assertTrue(self.store.update_camera("overhead", jpeg(), "overhead_optical", 99))
        self.assertEqual(3, self.store.sequence("overhead"))

    def test_bad_camera_payload_and_wrong_optical_frame_rejected(self):
        for data, frame, stamp in [(b"not jpeg", "overhead_optical", 10),
                                    (jpeg(), "left_optical", 10), (jpeg(), "overhead_optical", 0)]:
            with self.subTest(frame=frame, stamp=stamp), self.assertRaises((ValueError, OSError)):
                self.store.update_camera("overhead", data, frame, stamp)
        self.assertEqual(0, self.store.sequence("overhead"))

    def test_topology_and_ros_names(self):
        for name in ("single_arm.json", "dual_arm.json"):
            self.assertTrue(config(name).frame_ids)
        data = self.config.model_dump()
        for update in [{"arms": []}, {"arms": data["arms"] + [data["arms"][0]]},
                       {"server": {"namespace": "/invalid-name"}}]:
            with self.subTest(update=update), self.assertRaises(ValidationError):
                RobotConfig.model_validate(data | update)
        data["cameras"][1]["parent_frame"] = "right_flange"
        with self.assertRaises(ValidationError):
            RobotConfig.model_validate(data)

    def test_internal_requests_are_also_validated(self):
        good = {"frame_id": "world", "position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
        for operation, resource, payload, code in [
            ("move_arm", "unknown", good, 404), ("camera", "unknown", {}, 404),
            ("move_arm", "left", good | {"frame_id": "unknown"}, 422),
            ("move_arm", "left", good | {"duration": float("nan")}, 422),
            ("set_gripper", "left", {"opening": True}, 422),
            ("state", "", {"unexpected": 1}, 422), ("navigate", "", {}, 404),
        ]:
            with self.subTest(operation=operation), self.assertRaises(BridgeError) as error:
                validate_request(self.config, operation, resource, payload)
            self.assertEqual(code, error.exception.status)
