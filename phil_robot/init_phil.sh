#!/bin/bash

set -euo pipefail

PY_BIN="${PYTHON_BIN:-python}"

mapfile -t MODEL_LIST < <("$PY_BIN" - <<'PY'
from config import CLASSIFIER_MODEL, PLANNER_MODEL

print(CLASSIFIER_MODEL)
print(PLANNER_MODEL)
PY
)

preload_model() {
    local model_name="$1"

    echo "   - ${model_name}"
    curl -s http://localhost:11434/api/generate \
        -d "{\"model\":\"${model_name}\",\"keep_alive\":-1}" > /dev/null
}

echo "⚡️ [1/2] 젯슨 풀파워 가동 (Jetson Clocks)..."
# 성능 봉인 해제 (비밀번호 입력 필요)
sudo jetson_clocks

echo "🧠 [2/2] LLM 모델 메모리에 알박기 (Keep-Alive)..."
# classifier와 planner를 모두 올려 첫 턴 cold load를 줄인다.
for model_name in "${MODEL_LIST[@]}"; do
    preload_model "$model_name"
done

echo "📦 현재 Ollama 상주 상태:"
ollama ps

echo "✅ LLM 준비 완료!"
