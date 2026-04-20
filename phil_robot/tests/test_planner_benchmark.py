import os
import tempfile
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
from phil_robot.pipeline.brain_pipeline import BrainTurnResult, run_brain_turn
from phil_robot.pipeline.skills import expand_skills
from phil_robot.pipeline.validator import ValidatedPlan, build_validated_plan


class PlannerBenchmarkTest(unittest.TestCase):
    def test_planner_system_prompt_puts_shared_rules_first(self) -> None:
        prompt_text = get_planner_system_prompt("motion")

        self.assertTrue(prompt_text.startswith(PLANNER_SHARED_RULES))
        self.assertTrue(prompt_text.endswith(DOMAIN_INSTRUCTIONS["motion"]))
        self.assertIn("look_forward skill 이나 look:0,90 명령을 추가하지 않는다", prompt_text)
        self.assertIn("긍정은 nod_yes, 부정은 shake_no", prompt_text)
        self.assertIn("준비 자세", prompt_text)
        self.assertIn("unrelated social skill", prompt_text)

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
        state_idx = input_json.find('"robot_state"')
        motion_idx = input_json.find('"needs_motion"')
        user_idx = input_json.find('"user_text"')

        self.assertEqual(input_json.find('"response_schema"'), -1)
        self.assertEqual(input_json.find('"intent_result"'), -1)
        self.assertLess(planner_idx, state_idx)
        self.assertLess(state_idx, motion_idx)
        self.assertLess(motion_idx, user_idx)

    def test_planner_response_schema_example_uses_compact_keys(self) -> None:
        self.assertEqual(
            PLANNER_RESPONSE_SCHEMA_EXAMPLE,
            {"s": [], "c": [], "t": "안녕하세요!", "r": "simple greeting"},
        )

    def test_wave_hi_skill_no_longer_forces_forward_look(self) -> None:
        op_cmds, warnings = expand_skills(["wave_hi"])

        self.assertEqual(op_cmds, ["gesture:wave"])
        self.assertEqual(warnings, [])

    def test_expand_skills_keeps_direct_wait_command_in_order(self) -> None:
        op_cmds, warnings = expand_skills(["arm_up", "wait:3", "arm_down"])

        self.assertEqual(
            op_cmds,
            [
                "move:R_arm2,58",
                "move:L_arm2,58",
                "move:R_arm3,95",
                "move:L_arm3,95",
                "move:R_wrist,0",
                "move:L_wrist,0",
                "wait:3",
                "move:R_arm2,0",
                "move:L_arm2,0",
                "move:R_arm3,20",
                "move:L_arm3,20",
            ],
        )
        self.assertEqual(warnings, [])

    def test_filter_skills_keeps_direct_wait_command(self) -> None:
        from phil_robot.pipeline.skills import filter_skills_by_allowed_categories

        filtered = filter_skills_by_allowed_categories(
            ["arm_up", "wait:3", "arm_down"],
            {"posture"},
        )

        self.assertEqual(filtered, ["arm_up", "wait:3", "arm_down"])

    def test_build_validated_plan_overrides_relative_motion_speech(self) -> None:
        plan_obj = build_validated_plan(
            user_text="오른 쪽 손목 들어봐",
            robot_state={
                "state": 0,
                "is_lock_key_removed": True,
                "is_fixed": True,
                "current_angles": {"R_wrist": 20.0},
                "last_action": "move:R_wrist,20",
            },
            classifier_result={
                "intent": "motion_request",
                "needs_motion": True,
                "needs_dialogue": True,
                "risk_level": "medium",
            },
            planner_result={
                "skills": ["wave_hi"],
                "op_cmd": [],
                "speech": "안녕하세요!",
                "reason": "fallback greeting",
            },
        )

        self.assertEqual(plan_obj.valid_op_cmds, ["move:R_wrist,35"])
        self.assertIn("오른쪽 손목", plan_obj.speech)
        self.assertIn("15도", plan_obj.speech)

    def test_build_validated_plan_preserves_arm_wait_sequence(self) -> None:
        plan_obj = build_validated_plan(
            user_text="양팔 올렸다가 3초 뒤에 양팔 내려",
            robot_state={
                "state": 0,
                "is_lock_key_removed": True,
                "is_fixed": True,
                "current_angles": {},
            },
            classifier_result={
                "intent": "motion_request",
                "needs_motion": True,
                "needs_dialogue": True,
                "risk_level": "medium",
            },
            planner_result={
                "skills": ["arm_up", "wait:3", "arm_down"],
                "op_cmd": [],
                "speech": "양팔을 올렸다가 3초 뒤에 내립니다.",
                "reason": "arm sequence",
            },
        )

        self.assertEqual(
            plan_obj.valid_op_cmds,
            [
                "move:R_arm2,58",
                "move:L_arm2,58",
                "move:R_arm3,95",
                "move:L_arm3,95",
                "move:R_wrist,0",
                "move:L_wrist,0",
                "wait:3",
                "move:R_arm2,0",
                "move:L_arm2,0",
                "move:R_arm3,20",
                "move:L_arm3,20",
            ],
        )

    def test_build_validated_plan_resolves_relative_wait_sequence(self) -> None:
        plan_obj = build_validated_plan(
            user_text="손목 30도 내리고 1초 뒤에 10도 더 내려",
            robot_state={
                "state": 0,
                "is_lock_key_removed": True,
                "is_fixed": True,
                "current_angles": {"R_wrist": 50.0},
                "last_action": "move:R_wrist,50",
            },
            classifier_result={
                "intent": "motion_request",
                "needs_motion": True,
                "needs_dialogue": False,
                "risk_level": "medium",
            },
            planner_result={
                "skills": [],
                "op_cmd": ["move:L_wrist,30", "wait:1", "move:L_wrist,40"],
                "speech": "손목을 30도 내리고 1초 후에 10도 더 내립니다.",
                "reason": "relative wait sequence",
            },
        )

        self.assertEqual(plan_obj.valid_op_cmds, ["move:R_wrist,20", "wait:1", "move:R_wrist,10"])
        self.assertIn("30도", plan_obj.speech)
        self.assertIn("1초", plan_obj.speech)
        self.assertIn("10도", plan_obj.speech)

    def test_build_validated_plan_resolves_relative_repeat_sequence(self) -> None:
        plan_obj = build_validated_plan(
            user_text="손목 30도씩 두번 내려.",
            robot_state={
                "state": 0,
                "is_lock_key_removed": True,
                "is_fixed": True,
                "current_angles": {"R_wrist": 70.0},
                "last_action": "move:R_wrist,70",
            },
            classifier_result={
                "intent": "motion_request",
                "needs_motion": True,
                "needs_dialogue": False,
                "risk_level": "medium",
            },
            planner_result={
                "skills": [],
                "op_cmd": ["move:L_wrist,30", "wait:1", "move:L_wrist,30", "wait:1"],
                "speech": "손목을 30도씩 두 번 내립니다.",
                "reason": "relative repeat sequence",
            },
        )

        self.assertEqual(plan_obj.valid_op_cmds, ["move:R_wrist,40", "move:R_wrist,10"])
        self.assertIn("두번", plan_obj.speech)
        self.assertIn("30도", plan_obj.speech)
        self.assertIn("60도", plan_obj.speech)

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

    @patch("phil_robot.pipeline.brain_pipeline.call_json_llm")
    def test_run_brain_turn_shortcuts_repertoire_question(self, mock_call) -> None:
        mock_call.return_value = '{"i":"P","m":1,"d":1,"r":"M"}'

        turn_obj = run_brain_turn(
            "너 무슨 노래 연주할 수 있니?",
            {
                "state": 0,
                "is_fixed": True,
                "current_song": "None",
                "progress": "0/1",
                "last_action": "None",
                "is_lock_key_removed": False,
                "error_message": "None",
                "current_angles": {"waist": 0.0},
            },
        )

        mock_call.assert_called_once()
        self.assertEqual(turn_obj.classifier_result["intent"], "chat")
        self.assertFalse(turn_obj.classifier_result["needs_motion"])
        self.assertEqual(turn_obj.planner_domain, "chat")
        self.assertEqual(turn_obj.planner_input_json, "")
        self.assertEqual(turn_obj.planner_raw_response_text, "")
        self.assertEqual(turn_obj.validated_plan.valid_op_cmds, [])
        self.assertIn("This Is Me", turn_obj.validated_plan.speech)
        self.assertIn("Test Beat", turn_obj.validated_plan.speech)
        self.assertIn("그대에게", turn_obj.validated_plan.speech)
        self.assertIn("Baby I Need You", turn_obj.validated_plan.speech)

    @patch("phil_robot.pipeline.brain_pipeline.call_json_llm")
    def test_run_brain_turn_shortcuts_identity_confirmation_negative(self, mock_call) -> None:
        mock_call.return_value = '{"i":"C","m":1,"d":1,"r":"M"}'

        turn_obj = run_brain_turn(
            "너의 이름은 모펫이니?",
            {
                "state": 0,
                "is_fixed": True,
                "current_song": "None",
                "progress": "0/1",
                "last_action": "None",
                "is_lock_key_removed": True,
                "error_message": "None",
                "current_angles": {"waist": 0.0},
            },
        )

        mock_call.assert_called_once()
        self.assertEqual(turn_obj.classifier_result["intent"], "motion_request")
        self.assertEqual(turn_obj.planner_domain, "motion")
        self.assertEqual(turn_obj.planner_input_json, "")
        self.assertEqual(turn_obj.planner_raw_response_text, "")
        self.assertEqual(turn_obj.validated_plan.valid_op_cmds, ["gesture:shake"])
        self.assertEqual(turn_obj.validated_plan.speech, "아니요, 제 이름은 필이에요.")

    @patch("phil_robot.pipeline.brain_pipeline.call_json_llm")
    def test_run_brain_turn_shortcuts_wave_then_play_song(self, mock_call) -> None:
        mock_call.return_value = '{"i":"M","m":1,"d":1,"r":"M"}'

        turn_obj = run_brain_turn(
            "손흔들고 This Is Me 연주해줘.",
            {
                "state": 0,
                "is_fixed": True,
                "current_song": "None",
                "progress": "0/1",
                "last_action": "None",
                "is_lock_key_removed": True,
                "error_message": "None",
                "current_angles": {"waist": 0.0, "R_wrist": 20.0, "L_wrist": 20.0},
            },
        )

        mock_call.assert_called_once()
        self.assertEqual(turn_obj.classifier_result["intent"], "play_request")
        self.assertEqual(turn_obj.planner_domain, "play")
        self.assertEqual(turn_obj.planner_input_json, "")
        self.assertEqual(turn_obj.planner_raw_response_text, "")
        self.assertEqual(turn_obj.validated_plan.valid_op_cmds, ["gesture:wave", "r", "p:TIM"])
        self.assertIn("This Is Me", turn_obj.validated_plan.speech)

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

    def test_run_eval_summary_includes_avg_median_p95(self) -> None:
        row_list = [
            {
                "id": "case_a",
                "passed": True,
                "checks": [{"name": "classifier.intent", "passed": True}],
                "durations_sec": {
                    "classifier": 0.5,
                    "planner": 2.0,
                    "total": 2.5,
                },
            },
            {
                "id": "case_b",
                "passed": False,
                "checks": [{"name": "classifier.intent", "passed": False}],
                "durations_sec": {
                    "classifier": 0.7,
                    "planner": 4.0,
                    "total": 4.7,
                },
            },
        ]

        sum_obj = run_eval.summarize_results(row_list)

        self.assertEqual(sum_obj["latency_summary"]["classifier"]["avg_sec"], 0.6)
        self.assertEqual(sum_obj["latency_summary"]["classifier"]["median_sec"], 0.6)
        self.assertEqual(sum_obj["latency_summary"]["classifier"]["p95_sec"], 0.7)
        self.assertEqual(sum_obj["latency_summary"]["planner"]["avg_sec"], 3.0)
        self.assertEqual(sum_obj["latency_summary"]["planner"]["median_sec"], 3.0)
        self.assertEqual(sum_obj["latency_summary"]["planner"]["p95_sec"], 4.0)
        self.assertEqual(sum_obj["latency_summary"]["total"]["slowest_case"], "case_b")

    def test_build_doc_path_maps_report_into_eval_docs(self) -> None:
        json_path = os.path.join(run_eval.CURRENT_DIR, "reports", "smoke_report_test.json")
        md_path = run_eval.build_doc_path(json_path)

        self.assertEqual(
            md_path,
            os.path.join(run_eval.CURRENT_DIR, "eval_docs", "reports", "smoke_report_test.md"),
        )

    def test_save_report_md_includes_pass_fail_and_actual_speech(self) -> None:
        row_list = [
            {
                "id": "pass_case",
                "user_text": "안녕",
                "passed": True,
                "checks": [{"name": "classifier.intent", "passed": True}],
                "failed_checks": [],
                "actual": {
                    "valid_op_cmds": [],
                    "speech": "안녕하세요!",
                    "planner_is_fallback": False,
                },
                "durations_sec": {
                    "classifier": 0.4,
                    "planner": 1.0,
                    "total": 1.4,
                },
            },
            {
                "id": "fail_case",
                "user_text": "고개 끄덕여봐",
                "passed": False,
                "checks": [{"name": "validator.valid_op_cmds_exact", "passed": False}],
                "failed_checks": [
                    {
                        "name": "validator.valid_op_cmds_exact",
                        "expected": [],
                        "actual": ["gesture:nod"],
                    }
                ],
                "actual": {
                    "valid_op_cmds": ["gesture:nod"],
                    "speech": "지금은 연주 중입니다.",
                    "planner_is_fallback": False,
                },
                "durations_sec": {
                    "classifier": 0.6,
                    "planner": 1.8,
                    "total": 2.4,
                },
            },
        ]
        sum_obj = run_eval.summarize_results(row_list)
        meta_obj = {
            "generated_at": "2026-04-06T14:28:00+09:00",
            "suite": "smoke",
            "cases_path": "/tmp/cases_smoke.json",
            "classifier_model": "clf:test",
            "planner_model": "plan:test",
            "capture_llm_metrics": False,
        }

        with tempfile.TemporaryDirectory() as tmp_dir:
            json_path = os.path.join(tmp_dir, "reports", "smoke_report_test.json")
            run_eval.save_report(json_path, row_list, sum_obj, meta_obj)
            md_path = run_eval.save_report_md(json_path)
            self.assertTrue(os.path.exists(md_path))

            with open(md_path, "r", encoding="utf-8") as file:
                md_text = file.read()

        self.assertIn("총 2건 중 1건 통과, 1건 실패", md_text)
        self.assertIn("통과한 케이스", md_text)
        self.assertIn("pass_case", md_text)
        self.assertIn("fail_case", md_text)
        self.assertIn("안녕하세요!", md_text)
        self.assertIn("지금은 연주 중입니다.", md_text)
        self.assertIn("바로 고쳐야 할 항목", md_text)
        self.assertIn("실패 항목", md_text)
        self.assertIn("기대한 것", md_text)
        self.assertIn("실제로 나온 것", md_text)
        self.assertIn("명령 불일치", md_text)
        self.assertIn("명령 불일치: 없음", md_text)
        self.assertIn("명령 불일치: gesture:nod", md_text)
        self.assertIn("1/2 (50.0%)", md_text)
        self.assertIn("p95", md_text)

    def test_evaluate_actual_supports_speech_contains_all(self) -> None:
        checks, failed_checks, passed = run_eval.evaluate_actual(
            {"speech_contains_all": ["30도", "1초", "10도"]},
            {
                "intent": None,
                "needs_motion": None,
                "needs_dialogue": None,
                "planner_domain": None,
                "skills": [],
                "valid_op_cmds": [],
                "speech": "손목을 30도 내리고 1초 후에 10도 더 내립니다.",
            },
        )

        self.assertTrue(passed)
        self.assertEqual(len(failed_checks), 0)
        self.assertEqual(checks[0]["name"], "e2e.speech_contains_all")

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
