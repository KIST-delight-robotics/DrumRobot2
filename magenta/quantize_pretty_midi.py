"""mido 패키지로 midi 분석하는 파이썬 코드(함수화 안함)"""
import pretty_midi
import os
import numpy as np

# --- 1. 전처리 파이프라인 각 단계 함수 (pretty_midi 버전) ---

def apply_debouncing(notes: list, threshold: float) -> list:
    """디바운싱 필터. 악기별로 잔타격을 제거합니다."""
    if not notes: return []
    print(f"[1/3] 디바운싱 적용 (임계값: {threshold*1000:.0f}ms)...")
    
    filtered_notes = []
    last_hit_times = {}  # 각 pitch별 마지막 유효 타격 시간 저장
    
    for note in notes:
        pitch = note.pitch
        # pretty_midi의 note.start는 절대 시간(초)입니다.
        if pitch not in last_hit_times or note.start - last_hit_times[pitch] > threshold:
            filtered_notes.append(note)
            last_hit_times[pitch] = note.start
            
    print(f"    > {len(notes)}개 노트 -> {len(filtered_notes)}개 노트")
    print(f"{filtered_notes}")
    return filtered_notes

def apply_clustering(notes: list, threshold: float) -> list:
    """노트 클러스터링. 동시 타격 노트를 그룹화합니다."""
    if not notes: return []
    print(f"[2/3] 클러스터링 적용 (임계값: {threshold*1000:.0f}ms)...")
    
    clusters = []
    # 첫 노드로 첫 클러스터를 생성하며 시작
    clusters.append([notes[0]])
    
    for note in notes[1:]:
        # 현재 노트의 시간을 '가장 마지막 클러스터의 첫 번째 노트' 시간과 비교
        if note.start - clusters[-1][0].start < threshold:
            clusters[-1].append(note)
        else:
            clusters.append([note]) # 새 클러스터 시작
            
    print(f"    > {len(notes)}개 노트 -> {len(clusters)}개 클러스터")
    return clusters

def apply_grid_quantization(clusters: list, bpm: float, subdivisions: int) -> list:
    """그리드 기반 양자화. 클러스터를 그리드에 맞춥니다."""
    if not clusters: return []
    print(f"[3/3] 그리드 양자화 적용 (BPM: {bpm}, 단위: 1/{subdivisions*4}음표)...")
    
    quantized_notes = []
    seconds_per_beat = 60.0 / bpm
    quantize_step_duration = seconds_per_beat / subdivisions
    
    for cluster in clusters:
        if not cluster: continue
        
        # 클러스터의 평균 시작 시간 계산
        average_start_time = sum(note.start for note in cluster) / len(cluster)
        
        # 가장 가까운 그리드 시간으로 양자화
        quantized_steps = round(average_start_time / quantize_step_duration)
        quantized_start_time = quantized_steps * quantize_step_duration
        
        # 클러스터 내 모든 노트의 타이밍을 보정하여 새 노트 객체 생성
        for note in cluster:
            duration = note.end - note.start  # 원래 노트의 길이 유지
            new_note = pretty_midi.Note(
                velocity=note.velocity,
                pitch=note.pitch,
                start=quantized_start_time,
                end=quantized_start_time + duration
            )
            quantized_notes.append(new_note)
            
    print(f"    > {len(clusters)}개 클러스터 양자화 완료")
    return quantized_notes

def analyze_drum_patterns(notes: list, bpm: float):
    """    MIDI 노트 리스트 전체를 분석하여 'Fill-in'인지 'Drum Beat'인지 분류합니다.    """
    if not notes or bpm <= 0:
        return "Drum Beat" # 노트가 없으면 기본값으로 'Drum Beat' 반환

    # 필 인 판별의 기준이 되는 노트 번호 (주로 탐과 스네어)
    target_notes = {41, 38, 45, 47, 48, 50}
    
    # MIDI 전체에서 target_notes에 해당하는 악기 종류 수 계산
    found_target_pitches = set()
    for note in notes:
        if note.pitch in target_notes:
            found_target_pitches.add(note.pitch)
            
    match_count = len(found_target_pitches)
    
    # 분류 규칙 적용
    if match_count >= 2:
        classification = 'Fill-in'
    else:
        classification = 'Drum Beat'
        
    return classification

# --- 2. 메인 실행 함수 ---

def quantize_midi(input_filename: str, **kwargs):
    """
    지정된 MIDI 파일을 전처리하고, 패턴을 분석한 후, 'output_...' 이름으로 저장합니다.
    분석 결과도 함께 반환합니다.
    """
    
    # --- 기본 설정값 ---
    params = {
        "base_path": "/home/shy/DrumRobot/DrumSound/",
        "bpm": 100,
        "subdivisions": 2,
        "debounce_threshold": 0.04,
        "cluster_threshold": 0.02
    }
    params.update(kwargs)

    input_filepath = os.path.join(params["base_path"], input_filename)
    
    if not input_filename.startswith("input_"):
        print(f"오류: 입력 파일 이름은 'input_'으로 시작해야 합니다. (현재: {input_filename})")
        return None
    output_filename = input_filename.replace("input_", "output_", 1)
    output_filepath = os.path.join(params["base_path"], output_filename)

    if not os.path.exists(input_filepath):
        print(f"오류: 입력 파일 '{input_filepath}'을 찾을 수 없습니다.")
        return None

    print(f"--- '{input_filename}' 파일 전처리 시작 ---")
    
    pm = pretty_midi.PrettyMIDI(input_filepath)
    
    try:
        bpm_to_use = pm.get_tempo_changes()[1][0]
    except IndexError:
        bpm_to_use = params["bpm"]
    print(f"기준 BPM: {bpm_to_use:.1f}")

    all_notes = []
    for instrument in pm.instruments:
        all_notes.extend(instrument.notes)
    all_notes.sort(key=lambda x: x.start)

    # 파이프라인 실행
    debounced_notes = apply_debouncing(all_notes, params["debounce_threshold"])
    clusters = apply_clustering(debounced_notes, params["cluster_threshold"])
    quantized_notes = apply_grid_quantization(clusters, bpm_to_use, params["subdivisions"])
    
    # [수정] 파일 저장 전, 양자화된 노트를 기준으로 패턴 분석
    analysis_results = analyze_drum_patterns(quantized_notes, bpm_to_use)

    # 새로운 MIDI 파일 객체 생성 및 결과 저장
    output_pm = pretty_midi.PrettyMIDI()
    drum_instrument = pretty_midi.Instrument(program=0, is_drum=True)
    drum_instrument.notes.extend(quantized_notes)
    output_pm.instruments.append(drum_instrument)
    
    output_pm.write(output_filepath)
    print(f"✅ 전처리 완료! 결과가 '{output_filename}'에 저장되었습니다.\n")
    
    # [수정] 분석 결과를 반환
    print(f"{analysis_results}")
    return analysis_results

# --- 3. 스크립트 실행 예시 ---
if __name__ == '__main__':
    # 이 스크립트를 직접 실행할 때 아래 코드가 동작합니다.
    # 다른 파이썬 파일에서 import하여 사용할 때는 실행되지 않습니다.
    
    # 예시 1: 기본 설정으로 "input_00.mid" 파일 처리
    quantize_midi("input_00.mid")
    
    # # 예시 2: "input_01.mid" 파일을 16분음표(subdivisions=4) 기준으로 처리
    # quantize_midi("input_01.mid", subdivisions=4)
    
    # # 예시 3: BPM을 120으로 강제 설정하여 처리
    # quantize_midi("input_02.mid", bpm=120)