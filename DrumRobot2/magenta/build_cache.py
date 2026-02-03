#!/usr/bin/env python3
import os
import numpy as np
from mido import MidiFile

BAR_LEN_BEATS = 4.0
STEPS_PER_BAR = 16
TARGET_BARS   = 2

# GM Drum (Kick, Snare만 사용)
KICK_NOTES  = {35, 36}
SNARE_NOTES = {38, 40}
# HIHAT_NOTES = {42, 44, 46}  # 사용 안 함

def midi_to_grids_01(midi_path, velocity_min=1, max_bars=TARGET_BARS):
    """MIDI -> [{kick:[0/1x16], snare:[0/1x16]}, ...] (bar 단위)"""
    mid = MidiFile(midi_path)
    tpq = mid.ticks_per_beat

    events = []  # (abs_tick, note, vel)
    for track in mid.tracks:
        cur = 0
        for msg in track:
            cur += msg.time
            if msg.type == 'note_on' and msg.velocity > 0:
                events.append((cur, msg.note, msg.velocity))

    events.sort(key=lambda x: x[0])

    grids = [{"kick":[0]*STEPS_PER_BAR, "snare":[0]*STEPS_PER_BAR}
             for _ in range(max_bars)]

    for abs_tick, note, vel in events:
        if vel < velocity_min:
            continue
        beat = abs_tick / tpq
        if beat < 0:
            continue

        bar_idx = int(beat // BAR_LEN_BEATS)
        if bar_idx >= max_bars:
            break

        pos = beat - bar_idx * BAR_LEN_BEATS
        step = int(round((pos / BAR_LEN_BEATS) * STEPS_PER_BAR)) % STEPS_PER_BAR

        if note in KICK_NOTES:
            grids[bar_idx]["kick"][step] = 1
        elif note in SNARE_NOTES:
            grids[bar_idx]["snare"][step] = 1
        else:
            pass  # 다른 악기는 무시
    return grids

def ensure_two_bars(grids):
    """그리드 길이를 정확히 2마디로 맞춤"""
    if len(grids) >= 2:
        return grids[:2]
    if len(grids) == 1:
        g0 = grids[0]
        g1 = {"kick": g0["kick"][:], "snare": g0["snare"][:]}
        return [g0, g1]
    Z = {"kick":[0]*STEPS_PER_BAR, "snare":[0]*STEPS_PER_BAR}
    return [Z, Z]

def concat_2bars_to_vector(g2):
    """2마디 그리드 -> 벡터(길이 64 = kick32 + snare32)"""
    def flat(inst): return g2[0][inst] + g2[1][inst]
    vec = flat("kick") + flat("snare")
    return np.array(vec, dtype=np.uint8)

def main():
    library_dir = input("라이브러리 MIDI 폴더 경로 입력: ").strip()
    out_file = input("출력 캐시 파일명(.npz) [기본=library_cache.npz]: ").strip() or "library_cache.npz"
    try:
        velocity_min = int(input("무시할 최소 velocity (기본=1): ").strip() or "1")
    except ValueError:
        velocity_min = 1

    if not os.path.isdir(library_dir):
        print("❌ 폴더가 존재하지 않습니다.")
        return

    files = [f for f in sorted(os.listdir(library_dir))
             if f.lower().endswith((".mid", ".midi"))]
    if not files:
        print("❌ 폴더에 MIDI가 없습니다.")
        return

    names, vectors = [], []
    for f in files:
        path = os.path.join(library_dir, f)
        try:
            grids = midi_to_grids_01(path, velocity_min, max_bars=TARGET_BARS)
            g2 = ensure_two_bars(grids)
            vec = concat_2bars_to_vector(g2)  # (64,)
            names.append(f)
            vectors.append(vec)
            print("OK", f)
        except Exception as e:
            print(f"⚠️  {f} 스킵: {e}")

    if not vectors:
        print("❌ 파싱 성공한 파일이 없습니다.")
        return

    vectors = np.stack(vectors, axis=0)  # (N,64)
    meta = {
        "velocity_min": velocity_min,
        "bar_len_beats": BAR_LEN_BEATS,
        "steps_per_bar": STEPS_PER_BAR,
        "target_bars": TARGET_BARS,
        "instruments": ["kick", "snare"],
        "vector_layout": "kick32|snare32"
    }

    np.savez_compressed(
        out_file,
        names=np.array(names, dtype=object),
        vectors=vectors,
        meta=np.array([meta], dtype=object)
    )
    print(f"\n✅ 캐시 저장 완료: {os.path.abspath(out_file)}")
    print(f"   항목 수: {len(names)} / 벡터 shape: {vectors.shape}")

if __name__ == "__main__":
    main()
