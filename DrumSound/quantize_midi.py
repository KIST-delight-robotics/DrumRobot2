import mido
from mido import MidiFile, MidiTrack, MetaMessage
import os

# --- 1. MIDI 파일 <-> 절대 시간 메시지 리스트 변환 함수 ---

def get_midi_bpm(mid_file, default_bpm = 120):
    """MIDI 파일에서 BPM 정보를 추출합니다."""
    for msg in mid_file.tracks[0]:
        if msg.is_meta and msg.type == 'set_tempo':
            return mido.tempo2bpm(msg.tempo)
    return default_bpm

def midi_to_absolute_messages(mid_file):
    """MidiFile 객체를 절대 시간(초) 기반의 메시지 리스트로 변환합니다."""
    messages = []
    abs_time = 0.0
    ticks_per_beat = mid_file.ticks_per_beat
    print(f"{ticks_per_beat}\n")
    # [수정] BPM을 한 번만 조회하도록 tempo 변수 위치 변경
    tempo = mido.bpm2tempo(get_midi_bpm(mid_file))
     
    for msg in mid_file:
        abs_time += mido.tick2second(msg.time, ticks_per_beat/1000, tempo)
        if msg.type == 'note_on' and msg.velocity > 0:
            messages.append(msg.copy(time=abs_time))
    print(f"{messages}")

    return messages

def messages_to_midi_file(messages: list, original_mid: MidiFile):
    """절대 시간 메시지 리스트를 MidiFile 객체로 변환합니다."""
    mid = MidiFile(ticks_per_beat=original_mid.ticks_per_beat)
    track = MidiTrack()
    mid.tracks.append(track)
    
    bpm = get_midi_bpm(original_mid)
    tempo = mido.bpm2tempo(bpm)
    track.append(MetaMessage('set_tempo', tempo=tempo, time=0))

    messages.sort(key=lambda x: x.time)
    
    last_abs_time = 0
    for msg in messages:
        delta_sec = msg.time - last_abs_time
        delta_ticks = mido.second2tick(delta_sec, original_mid.ticks_per_beat, tempo)
        track.append(msg.copy(time=max(0, int(round(delta_ticks)))))
        last_abs_time = msg.time
        
    return mid

# --- 2. 전처리 파이프라인 각 단계 함수 ---

def apply_debouncing(messages, threshold):
    """디바운싱 필터. 악기별로 잔타격을 제거합니다."""
    if not messages: return []
    print(f"[1/3] 디바운싱 적용 (임계값: {threshold*1000:.0f}ms)...")
    filtered_notes = []
    last_hit_times = {}

    for msg in messages:
        note_id = msg.note
        if note_id not in last_hit_times or msg.time - last_hit_times[note_id] > threshold:
            filtered_notes.append(msg)
            last_hit_times[note_id] = msg.time
    print(f"    > {len(messages)}개 노트 -> {len(filtered_notes)}개 노트")
    return filtered_notes

def apply_clustering(messages, threshold):
    """노트 클러스터링. 동시 타격 노트를 그룹화합니다."""
    if not messages: return []
    print(f"[2/3] 클러스터링 적용 (임계값: {threshold*1000:.0f}ms)...")
    clusters = []
    current_cluster = [messages[0]]
    clusters.append(current_cluster)
    for i in range(1, len(messages)):
        if messages[i].time - messages[i-1].time < threshold:
            current_cluster.append(messages[i])
        else:
            current_cluster = [messages[i]]
            clusters.append(current_cluster)
    print(f"    > {len(messages)}개 노트 -> {len(clusters)}개 클러스터")
    print(f"{clusters}")
    return clusters

def apply_grid_quantization(clusters, bpm, subdivisions):
    """그리드 기반 양자화. 클러스터를 그리드에 맞춥니다."""
    if not clusters: return []
    print(f"[3/3] 그리드 양자화 적용 (BPM: {bpm}, 단위: 1/{subdivisions*4}음표)...")
    quantized_messages = []
    seconds_per_beat = 60.0 / bpm
    quantize_step_duration = seconds_per_beat / subdivisions
    
    for cluster in clusters:
        if not cluster: continue
        average_time = sum(msg.time for msg in cluster) / len(cluster)
        quantized_steps = round(average_time / quantize_step_duration)
        quantized_time = quantized_steps * quantize_step_duration
        for msg in cluster:
            quantized_messages.append(msg.copy(time=quantized_time))
    
    print(f"{quantized_messages}")
    print(f"    > {len(clusters)}개 클러스터 양자화 완료")
    return quantized_messages

# --- 3. 메인 실행 함수 ---

def main():
    
    # --- 설정값 (여기서 수정) ---
    params = {
        "input_file": "/home/shy/DrumRobot/DrumSound/input_00.mid",
        "output_file": "/home/shy/DrumRobot/DrumSound/output_00.mid",
        "bpm": 100,
        "subdivisions": 2,
        "debounce_threshold": 0.04,
        "cluster_threshold": 0.02
    }

    # 입력 파일 존재 여부 확인
    if not os.path.exists(params["input_file"]):
        print(f"오류: 입력 파일 '{params['input_file']}'을 찾을 수 없습니다.")
        return

    print(f"'{params['input_file']}' 파일 전처리를 시작합니다...")
    
    # 1. MIDI 파일 로드
    input_mid = mido.MidiFile(params["input_file"])
    
    # 2. BPM 결정 (파일에 BPM 정보가 있으면 그 값을 사용, 없으면 설정값 사용)
    bpm_to_use = get_midi_bpm(input_mid, default_bpm=params["bpm"])
    print(f"기준 BPM: {bpm_to_use}")

    # 3. 파이프라인 실행
    # MIDI -> 절대 시간 메시지 리스트
    messages = midi_to_absolute_messages(input_mid)
    
    # 3-1. 디바운싱
    debounced_msgs = apply_debouncing(messages, params["debounce_threshold"])
    
    # 3-2. 클러스터링
    clusters = apply_clustering(debounced_msgs, params["cluster_threshold"])
    
    # 3-3. 그리드 양자화
    quantized_msgs = apply_grid_quantization(clusters, bpm_to_use, params["subdivisions"])
    
    # 4. 결과 메시지 리스트 -> MIDI 파일 객체
    output_mid = messages_to_midi_file(quantized_msgs, input_mid)
    
    # 5. 새로운 MIDI 파일로 저장
    output_mid.save(params["output_file"])
    print(f"\n✅ 전처리 완료! 결과가 '{params['output_file']}'에 저장되었습니다.")


if __name__ == '__main__':
    main()