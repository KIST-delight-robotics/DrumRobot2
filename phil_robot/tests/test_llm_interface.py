import importlib
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

from phil_robot.config import (
    CLASSIFIER_MODEL,
    CLASSIFIER_NUM_CTX,
    OLLAMA_KEEP_ALIVE,
    OLLAMA_THINK,
)
from phil_robot.pipeline.llm_interface import build_chat_metrics, call_json_llm


class LlmInterfaceTest(unittest.TestCase):
    def test_top_level_pipeline_import_uses_fallback(self) -> None:
        phil_dir = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(phil_dir))
        try:
            sys.modules.pop("pipeline.llm_interface", None)
            module_obj = importlib.import_module("pipeline.llm_interface")
            self.assertTrue(hasattr(module_obj, "call_json_llm"))
        finally:
            if sys.path and sys.path[0] == str(phil_dir):
                sys.path.pop(0)

    def test_build_chat_metrics_includes_load_breakdown(self) -> None:
        resp = {
            "load_duration": 500_000_000,
            "prompt_eval_count": 40,
            "prompt_eval_duration": 300_000_000,
            "eval_count": 20,
            "eval_duration": 900_000_000,
        }

        metrics = build_chat_metrics(resp, wall_sec=2.0)

        self.assertEqual(metrics["prompt_tokens"], 40)
        self.assertEqual(metrics["eval_tokens"], 20)
        self.assertAlmostEqual(metrics["load_sec"], 0.5)
        self.assertAlmostEqual(metrics["prompt_sec"], 0.3)
        self.assertAlmostEqual(metrics["eval_sec"], 0.9)
        self.assertAlmostEqual(metrics["infer_sec"], 1.2)
        self.assertAlmostEqual(metrics["meta_sec"], 1.7)
        self.assertAlmostEqual(metrics["overhead_sec"], 0.3)

    @patch("phil_robot.pipeline.llm_interface.ollama.chat")
    def test_call_json_llm_uses_latency_controls(self, mock_chat) -> None:
        mock_chat.return_value = {
            "message": {"content": '{"ok": true}'},
            "load_duration": 100_000_000,
            "prompt_eval_count": 10,
            "prompt_eval_duration": 200_000_000,
            "eval_count": 6,
            "eval_duration": 300_000_000,
        }

        raw_text, metrics = call_json_llm(
            model_name=CLASSIFIER_MODEL,
            system_prompt="sys",
            user_input_json="{}",
            capture_metrics=True,
        )

        self.assertEqual(raw_text, '{"ok": true}')
        self.assertAlmostEqual(metrics["load_sec"], 0.1)

        call_kwargs = mock_chat.call_args.kwargs
        self.assertEqual(call_kwargs["model"], CLASSIFIER_MODEL)
        self.assertEqual(call_kwargs["format"], "json")
        self.assertEqual(call_kwargs["think"], OLLAMA_THINK)
        self.assertEqual(call_kwargs["keep_alive"], OLLAMA_KEEP_ALIVE)
        self.assertEqual(call_kwargs["options"]["num_ctx"], CLASSIFIER_NUM_CTX)
        self.assertNotIn("num_predict", call_kwargs["options"])


if __name__ == "__main__":
    unittest.main()
