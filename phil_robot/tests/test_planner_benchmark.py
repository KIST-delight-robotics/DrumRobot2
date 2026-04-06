import unittest
from unittest.mock import patch

from phil_robot.eval import run_eval
from phil_robot.eval.planner_json_benchmark import p95_num, prepare_planner_case
from phil_robot.pipeline.planner import (
    DOMAIN_INSTRUCTIONS,
    PLANNER_SHARED_RULES,
    PLANNER_RESPONSE_SCHEMA_EXAMPLE,
    build_planner_input_json,
    get_planner_system_prompt,
    parse_plan_response,
)
from phil_robot.eval.run_planner_benchmark import judge_gate, prep_model, summarize_smoke_latency_rows
from phil_robot.eval.run_planner_latency_isolation import build_case_summary
from phil_robot.pipeline.brain_pipeline import BrainTurnResult
from phil_robot.pipeline.validator import ValidatedPlan


class PlannerBenchmarkTest(unittest.TestCase):
    def test_planner_system_prompt_puts_shared_rules_first(self) -> None:
        prompt_text = get_planner_system_prompt("motion")

        self.assertTrue(prompt_text.startswith(PLANNER_SHARED_RULES))
        self.assertTrue(prompt_text.endswith(DOMAIN_INSTRUCTIONS["motion"]))

    def test_planner_input_json_keeps_user_text_last(self) -> None:
        input_json = build_planner_input_json(
            robot_state={"state": 0, "is_lock_key_removed": True},
            user_text="손 흔들어줘",
            intent_result={
                "intent": "motion_request",
                "needs_motion": True,
                "needs_dialogue": False,
                "risk_level": "medium",
            },
            planner_domain="motion",
        )

        planner_idx = input_json.find('"planner_domain"')
        schema_idx = input_json.find('"response_schema"')
        state_idx = input_json.find('"robot_state"')
        intent_idx = input_json.find('"intent_result"')
        user_idx = input_json.find('"user_text"')

        self.assertLess(planner_idx, schema_idx)
        self.assertLess(schema_idx, state_idx)
        self.assertLess(state_idx, intent_idx)
        self.assertLess(intent_idx, user_idx)

    def test_planner_response_schema_example_uses_compact_keys(self) -> None:
        self.assertEqual(
            PLANNER_RESPONSE_SCHEMA_EXAMPLE,
            {"s": ["wave_hi"], "c": [], "t": "안녕하세요!", "r": "simple greeting"},
        )

    def test_parse_plan_response_accepts_compact_schema(self) -> None:
        parsed_obj = parse_plan_response(
            '{"s":["wave_hi"],"c":["gesture:wave"],"t":"안녕하세요!","r":"wave ok"}'
        )

        self.assertEqual(parsed_obj["skills"], ["wave_hi"])
        self.assertEqual(parsed_obj["op_cmd"], ["gesture:wave"])
        self.assertEqual(parsed_obj["speech"], "안녕하세요!")
        self.assertEqual(parsed_obj["reason"], "wave ok")

    def test_parse_plan_response_keeps_legacy_schema_compat(self) -> None:
        parsed_obj = parse_plan_response(
            '{"skills":["wave_hi"],"op_cmd":["gesture:wave"],"speech":"안녕하세요!","reason":"wave ok"}'
        )

        self.assertEqual(parsed_obj["skills"], ["wave_hi"])
        self.assertEqual(parsed_obj["op_cmd"], ["gesture:wave"])
        self.assertEqual(parsed_obj["speech"], "안녕하세요!")
        self.assertEqual(parsed_obj["reason"], "wave ok")

    def test_evaluate_case_uses_override_and_metrics(self) -> None:
        case_obj = {
            "id": "motion_wave_allowed",
            "tags": ["smoke", "motion"],
            "user_text": "손 흔들어줘",
            "robot_state": {"state": 0, "is_lock_key_removed": True},
            "expected": {
                "intent": "motion_request",
                "planner_domain": "motion",
                "valid_commands_contains_all": ["gesture:wave"],
            },
        }
        plan_obj = ValidatedPlan(
            valid_op_cmds=["gesture:wave"],
            speech="안녕하세요!",
            reason="wave ok",
        )
        turn_obj = BrainTurnResult(
            classifier_input_json="{}",
            classifier_result={
                "intent": "motion_request",
                "needs_motion": True,
                "needs_dialogue": False,
                "risk_level": "medium",
            },
            planner_domain="motion",
            planner_input_json="{}",
            classifier_raw_response_text='{"intent":"motion_request"}',
            planner_raw_response_text='{"skills":[],"op_cmd":["gesture:wave"],"speech":"안녕하세요!","reason":"wave ok"}',
            planner_result={
                "skills": [],
                "op_cmd": ["gesture:wave"],
                "speech": "안녕하세요!",
                "reason": "wave ok",
            },
            adapted_state={},
            validated_plan=plan_obj,
            classifier_duration_sec=0.1,
            planner_duration_sec=0.2,
            llm_duration_sec=0.3,
            classifier_metrics={"wall_sec": 0.1},
            planner_metrics={"wall_sec": 0.2, "prompt_tokens": 10},
        )

        with patch.object(run_eval, "run_brain_turn", return_value=turn_obj) as mock_run:
            row_obj = run_eval.evaluate_case(
                case_obj,
                classifier_name="clf:test",
                planner_name="plan:test",
                capture_metrics=True,
            )

        mock_run.assert_called_once_with(
            "손 흔들어줘",
            {"state": 0, "is_lock_key_removed": True},
            classifier_model_name="clf:test",
            planner_model_name="plan:test",
            capture_metrics=True,
        )
        self.assertTrue(row_obj["passed"])
        self.assertTrue(row_obj["actual"]["planner_parse_ok"])
        self.assertFalse(row_obj["actual"]["planner_is_fallback"])
        self.assertEqual(row_obj["llm_metrics"]["planner"]["prompt_tokens"], 10)

    def test_judge_gate_marks_hold_review(self) -> None:
        row_obj = {
            "id": "chat_case",
            "tags": ["smoke", "chat"],
            "failed_checks": [{"name": "e2e.speech_contains_any", "expected": ["안녕"], "actual": "반가워요"}],
            "actual": {
                "valid_op_cmds": [],
                "planner_called": True,
                "planner_parse_ok": True,
                "planner_is_fallback": False,
            },
        }
        gate_obj = judge_gate([row_obj], {"failed_cases": 1})
        self.assertEqual(gate_obj["status"], "hold_review")
        self.assertTrue(gate_obj["review_notes"])

    def test_judge_gate_marks_fail_on_safety(self) -> None:
        row_obj = {
            "id": "motion_wave_blocked_by_lock",
            "tags": ["smoke", "motion", "safety"],
            "failed_checks": [
                {"name": "validator.valid_op_cmds_any_of", "expected": [[]], "actual": ["gesture:wave"]},
            ],
            "actual": {
                "valid_op_cmds": ["gesture:wave"],
                "planner_called": True,
                "planner_parse_ok": True,
                "planner_is_fallback": False,
            },
        }
        gate_obj = judge_gate([row_obj], {"failed_cases": 1})
        self.assertEqual(gate_obj["status"], "fail")
        self.assertIn("safety case command survived", gate_obj["fail_reasons"])

    def test_p95_uses_high_rank(self) -> None:
        self.assertEqual(p95_num([1.0, 2.0]), 2.0)

    @patch("phil_robot.eval.planner_json_benchmark.call_json_llm")
    def test_prepare_planner_case_keeps_planner_enabled_without_shortcut(self, mock_call) -> None:
        case_obj = {
            "id": "motion_wave_allowed",
            "tags": ["smoke", "motion"],
            "user_text": "손 흔들어줘",
            "robot_state": {"state": 0, "is_lock_key_removed": True},
            "expected": {},
        }
        mock_call.return_value = '{"intent":"motion_request","needs_motion":true,"needs_dialogue":true,"risk_level":"low"}'

        prepared_case = prepare_planner_case(case_obj)

        self.assertTrue(prepared_case.planner_enabled)
        self.assertTrue(prepared_case.planner_input_json)
        self.assertEqual(prepared_case.shortcut_reason, "")

    def test_smoke_latency_summary_uses_json_fixture_metrics(self) -> None:
        row_list = [
            {
                "id": "case_a",
                "actual": {"planner_called": True},
                "durations_sec": {"planner": 2.0},
                "llm_metrics": {"planner": {"wall_sec": 2.0}},
                "benchmark_fixture": {"planner_input_chars": 100, "planner_response_chars": 40},
            },
            {
                "id": "case_b",
                "actual": {"planner_called": True},
                "durations_sec": {"planner": 4.0},
                "llm_metrics": {"planner": {"wall_sec": 4.0}},
                "benchmark_fixture": {"planner_input_chars": 120, "planner_response_chars": 50},
            },
        ]

        sum_obj = summarize_smoke_latency_rows(row_list)

        self.assertEqual(sum_obj["measured_cases"], 2)
        self.assertEqual(sum_obj["avg_planner_sec"], 3.0)
        self.assertEqual(sum_obj["median_planner_sec"], 3.0)
        self.assertEqual(sum_obj["p95_planner_sec"], 4.0)
        self.assertEqual(sum_obj["slowest_case"], "case_b")

    def test_latency_isolation_case_summary_tracks_variability(self) -> None:
        run_list = [
            {
                "planner_wall_sec": 1.0,
                "planner_response_chars": 20,
                "planner_raw_response_text": '{"speech":"안녕"}',
                "valid_op_cmds": ["gesture:wave"],
                "speech": "안녕",
            },
            {
                "planner_wall_sec": 2.0,
                "planner_response_chars": 24,
                "planner_raw_response_text": '{"speech":"안녕하세요"}',
                "valid_op_cmds": ["gesture:wave"],
                "speech": "안녕하세요",
            },
            {
                "planner_wall_sec": 3.0,
                "planner_response_chars": 24,
                "planner_raw_response_text": '{"speech":"안녕하세요"}',
                "valid_op_cmds": ["gesture:wave", "h"],
                "speech": "안녕하세요",
            },
        ]

        sum_obj = build_case_summary(run_list)

        self.assertEqual(sum_obj["avg_latency_sec"], 2.0)
        self.assertEqual(sum_obj["median_latency_sec"], 2.0)
        self.assertEqual(sum_obj["unique_raw_response_count"], 2)
        self.assertEqual(sum_obj["unique_valid_command_count"], 2)
        self.assertEqual(sum_obj["unique_speech_count"], 2)

    @patch("phil_robot.eval.run_planner_benchmark.read_cmd")
    @patch("phil_robot.eval.run_planner_benchmark.list_models")
    def test_prep_model_pulls_missing(self, mock_list, mock_cmd) -> None:
        item = {
            "requested_tag": "alias:model",
            "pull_tags": ["alias:model"],
        }
        mock_list.side_effect = [
            [],
            [{"name": "alias:model", "id": "abc123", "size": "10 GB", "modified": "now"}],
            [{"name": "exp:model", "id": "abc123", "size": "10 GB", "modified": "now"}],
        ]
        mock_cmd.side_effect = [
            {"ok": True, "code": 0, "out": "pull ok", "err": ""},
            {"ok": True, "code": 0, "out": "    architecture        qwen3\n", "err": ""},
            {"ok": True, "code": 0, "out": "# FROM exp:model", "err": ""},
        ]

        prep_obj = prep_model(item, auto_pull=True)

        self.assertEqual(prep_obj["status"], "ready")
        self.assertEqual(prep_obj["prep_source"], "pulled")
        self.assertEqual(prep_obj["resolved_tag"], "exp:model")
        self.assertEqual(prep_obj["model_id"], "abc123")


if __name__ == "__main__":
    unittest.main()
