import unittest

from phil_robot.pipeline.intent_classifier import (
    looks_like_identity_confirmation_motion,
    looks_like_play_request,
    looks_like_ready_pose_request,
    normalize_intent_result,
    parse_intent_response,
)


class IntentClassifierTest(unittest.TestCase):
    def test_parse_intent_response_salvages_truncated_compact_json(self) -> None:
        raw_text = (
            '{\n'
            '  "i": "P",\n'
            '  "m": 1,\n'
            '  "d": 1,\n'
        )

        result = parse_intent_response(raw_text)

        self.assertEqual(result["intent"], "play_request")
        self.assertTrue(result["needs_motion"])
        self.assertTrue(result["needs_dialogue"])
        self.assertEqual(result["risk_level"], "medium")

    def test_parse_intent_response_reads_compact_json(self) -> None:
        raw_text = '{"i":"Q","m":0,"d":1,"r":"H"}'

        result = parse_intent_response(raw_text)

        self.assertEqual(result["intent"], "status_question")
        self.assertFalse(result["needs_motion"])
        self.assertTrue(result["needs_dialogue"])
        self.assertEqual(result["risk_level"], "high")

    def test_normalize_intent_result_promotes_play_request_keywords(self) -> None:
        raw_result = {
            "intent": "unknown",
            "needs_motion": False,
            "needs_dialogue": True,
            "risk_level": "medium",
        }

        result = normalize_intent_result(raw_result, "그대에게 연주해줘")

        self.assertEqual(result["intent"], "play_request")
        self.assertTrue(result["needs_motion"])
        self.assertTrue(result["needs_dialogue"])
        self.assertEqual(result["risk_level"], "medium")

    def test_normalize_intent_result_promotes_identity_confirmation_to_motion(self) -> None:
        raw_result = {
            "intent": "chat",
            "needs_motion": False,
            "needs_dialogue": True,
            "risk_level": "low",
        }

        result = normalize_intent_result(raw_result, "너의 이름 필 맞지?")

        self.assertEqual(result["intent"], "motion_request")
        self.assertTrue(result["needs_motion"])
        self.assertTrue(result["needs_dialogue"])
        self.assertEqual(result["risk_level"], "medium")

    def test_looks_like_identity_confirmation_motion(self) -> None:
        self.assertTrue(looks_like_identity_confirmation_motion("너의 이름 필 맞지?"))
        self.assertTrue(looks_like_identity_confirmation_motion("너의 이름은 모펫이니?"))
        self.assertFalse(looks_like_identity_confirmation_motion("너 이름 뭐니?"))

    def test_normalize_intent_result_promotes_ready_pose_to_motion(self) -> None:
        raw_result = {
            "intent": "chat",
            "needs_motion": False,
            "needs_dialogue": True,
            "risk_level": "low",
        }

        result = normalize_intent_result(raw_result, "준비")

        self.assertEqual(result["intent"], "motion_request")
        self.assertTrue(result["needs_motion"])
        self.assertTrue(result["needs_dialogue"])
        self.assertEqual(result["risk_level"], "medium")

    def test_looks_like_ready_pose_request(self) -> None:
        self.assertTrue(looks_like_ready_pose_request("준비"))
        self.assertTrue(looks_like_ready_pose_request("준비 자세 해줘"))
        self.assertFalse(looks_like_ready_pose_request("준비됐니?"))

    def test_normalize_intent_result_keeps_repertoire_question_as_chat(self) -> None:
        raw_result = {
            "intent": "play_request",
            "needs_motion": True,
            "needs_dialogue": True,
            "risk_level": "medium",
        }

        result = normalize_intent_result(raw_result, "너 무슨 노래 연주할 수 있니?")

        self.assertEqual(result["intent"], "chat")
        self.assertFalse(result["needs_motion"])
        self.assertTrue(result["needs_dialogue"])
        self.assertEqual(result["risk_level"], "low")

    def test_looks_like_play_request_requires_more_than_song_name(self) -> None:
        self.assertFalse(looks_like_play_request("그대에게"))
        self.assertTrue(looks_like_play_request("그대에게 틀어줘"))


if __name__ == "__main__":
    unittest.main()
