import unittest
from unittest.mock import Mock

from phil_robot.runtime.melo_engine import TTS_Engine


class MeloEngineTest(unittest.TestCase):
    def build_engine(self) -> TTS_Engine:
        engine = TTS_Engine.__new__(TTS_Engine)
        engine.model = object()
        engine.preprocess = lambda text: text.strip()
        engine._speak_file = Mock()
        engine._render_audio = lambda text: f"audio::{text}"
        engine._play_audio = Mock()
        return engine

    def test_split_stream_text_strips_chunks(self) -> None:
        engine = self.build_engine()

        chunk_list = engine._split_stream_text(" 첫 문장입니다. 둘째 문장입니다.  ")

        self.assertEqual(chunk_list, ["첫 문장입니다", "둘째 문장입니다"])

    def test_speak_stream_plays_each_chunk(self) -> None:
        engine = self.build_engine()

        engine.speak("첫 문장입니다. 둘째 문장입니다.", stream=True)

        self.assertEqual(
            [call.args[0] for call in engine._play_audio.call_args_list],
            ["audio::첫 문장입니다", "audio::둘째 문장입니다"],
        )
        engine._speak_file.assert_not_called()

    def test_speak_stream_falls_back_when_stream_fails(self) -> None:
        engine = self.build_engine()
        engine._speak_stream = Mock(side_effect=RuntimeError("stream failed"))

        engine.speak("테스트", stream=True)

        engine._speak_file.assert_called_once_with("테스트", output_path=None, play=True)


if __name__ == "__main__":
    unittest.main()
