#!/bin/bash

echo "⚡️ [1/2] 젯슨 풀파워 가동 (Jetson Clocks)..."
# 성능 봉인 해제 (비밀번호 입력 필요)
sudo jetson_clocks

echo "🧠 [2/2] LLM 모델 메모리에 알박기 (Keep-Alive)..."
# 실행 없이 모델만 메모리에 올리고 끝냄 (응답을 기다리지 않고 바로 종료하려면 -s 옵션 추가)
curl -s http://localhost:11434/api/generate -d '{"model": "qwen3:30b-a3b-instruct-2507-q4_K_M", "keep_alive": -1}' > /dev/null

echo "✅ LLM 준비 완료!"