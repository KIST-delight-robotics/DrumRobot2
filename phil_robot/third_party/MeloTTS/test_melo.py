import torch
from melo.api import TTS
import time
import os

# 1. GPU(CUDA) 잘 잡혔는지 확인
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"--------------------------------------------------")
print(f"▶ 장치 확인: {device}")
if device == 'cuda':
    print(f"▶ GPU 이름: {torch.cuda.get_device_name(0)}")
else:
    print("WARNING: CPU로 돌아가고 있습니다.")
print(f"--------------------------------------------------")

# 2. 모델 로드
try:
    print("▶ 모델 로딩 중... (잠시만 기다려주세요)")
    # 한국어(KR) 모델 로드
    model = TTS(language='KR', device=device)
    speaker_ids = model.hps.data.spk2id
    print("▶ 모델 로드 완료!")
except Exception as e:
    print(f"❌ 모델 로드 실패: {e}")
    print("팁: torchaudio 버전 문제일 수 있습니다.")
    exit()

# 3. 추론 테스트
text = "안녕하세요. 저는 드럼로봇 필이에요. 목소리 잘 들리시나요? 젯슨 오린 보드에서 돌아가는 멜로 티티에스입니다. GPU 가속이 잘 되는지 확인해 보세요. 저의 나이는 3살이고요. 드럼로봇 치는것도 좋아해요. 뚱치땅치뚱뚱뚱 wow 재밌지 않나요?"
output_path = "test_output.wav"
speed = 1.0

print(f"▶ 음성 생성 시작: '{text}'")
start_time = time.time()

# 파일로 저장
model.tts_to_file(text, speaker_ids['KR'], output_path, speed=speed)

end_time = time.time()
print(f"--------------------------------------------------")
print(f"▶ 생성 완료! 걸린 시간: {end_time - start_time:.4f}초")
print(f"▶ 저장 위치: {output_path}")
print(f"--------------------------------------------------")

# 4. 바로 재생 (aplay 사용)
print(f"▶ 재생 중... 🔊")
# -q: 로그 숨기기, 만약 소리가 안 나면 -q를 빼고 에러를 확인하세요
exit_code = os.system(f"aplay -q {output_path}")

if exit_code != 0:
    print("❌ 재생 실패: 스피커가 연결되어 있는지, 소리 설정(Sound Settings)을 확인하세요.")
else:
    print("▶ 테스트 종료.")