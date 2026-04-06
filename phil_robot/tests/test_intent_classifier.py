import unittest

from phil_robot.pipeline.intent_classifier import (
    looks_like_play_request,
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

    def test_looks_like_play_request_requires_more_than_song_name(self) -> None:
        self.assertFalse(looks_like_play_request("그대에게"))
        self.assertTrue(looks_like_play_request("그대에게 틀어줘"))


if __name__ == "__main__":
    unittest.main()
