#!/usr/bin/env python3
import os
import numpy as np
from mido import MidiFile

BAR_LEN_BEATS = 4.0
STEPS_PER_BAR = 16
TARGET_BARS   = 2

KICK_NOTES  = {35, 36}
SNARE_NOTES = {38, 40}

def midi_to_grids_01(midi_path, velocity_min=1, max_bars=TARGET_BARS):
    mid = MidiFile(midi_path)
    tpq = mid.ticks_per_beat

    events = []
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
    return grids

def ensure_two_bars(g):
    if len(g) >= 2:
        return g[:2]
    if len(g) == 1:
        g0 = g[0]
        g1 = {"kick": g0["kick"][:], "snare": g0["snare"][:]}
        return [g0, g1]
    Z = {"kick":[0]*STEPS_PER_BAR, "snare":[0]*STEPS_PER_BAR}
    return [Z, Z]

def concat_2bars_to_vector(g2):
    def flat(inst): return g2[0][inst] + g2[1][inst]
    return np.array(flat("kick") + flat("snare"), dtype=np.uint8)  # (64,)

def hamming_sim_ks(vecA, vecB, weights=(0.6, 0.4)):
    """킥/스네어 구간별 해밍 유사도 가중 평균"""
    assert vecA.shape == (64,) and vecB.shape == (64,)
    kW, sW = weights
    kA, sA = vecA[0:32], vecA[32:64]
    kB, sB = vecB[0:32], vecB[32:64]
    sim = lambda a, b: float(np.sum(a == b)) / a.size
    kSim, sSim = sim(kA, kB), sim(sA, sB)
    return (kSim * kW + sSim * sW) / (kW + sW)

def main():
    cache_file = input("캐시 파일 경로 [기본=library_cache.npz]: ").strip() or "library_cache.npz"
    if not os.path.isfile(cache_file):
        print("❌ 캐시 파일을 찾을 수 없습니다."); return

    input_midi = input("비교할 MIDI 경로 [기본=input.mid]: ").strip() or "input.mid"
    if not os.path.isfile(input_midi):
        print("❌ 입력 MIDI를 찾을 수 없습니다."); return

    try:
        w_str = input("가중치 (kick snare) [기본=0.6 0.4]: ").strip() or "0.6 0.4"
        weights = tuple(float(x) for x in w_str.split())
        if len(weights) != 2: raise ValueError
    except:
        weights = (0.6, 0.4)

    try:
        topk = int(input("상위 몇 개 출력? [기본=3]: ").strip() or "3")
    except:
        topk = 3

    npz = np.load(cache_file, allow_pickle=True)
    names   = npz["names"]
    vectors = npz["vectors"]  # (N,64)
    meta    = npz["meta"][0]  # dict

    # 입력 MIDI → 벡터(64)
    g = midi_to_grids_01(input_midi, velocity_min=meta.get("velocity_min", 1), max_bars=meta.get("target_bars", 2))
    qvec = concat_2bars_to_vector(ensure_two_bars(g))

    # 유사도 계산 및 정렬
    results = [(str(n), hamming_sim_ks(qvec, v, weights)) for n, v in zip(names, vectors)]
    results.sort(key=lambda x: x[1], reverse=True)

    print("\n=== 매칭 결과 (Top {}) ===".format(topk))
    for i, (fname, sim) in enumerate(results[:topk], start=1):
        print(f"{i:2d}. {fname:30s}  {sim*100:6.2f}%")

if __name__ == "__main__":
    main()
