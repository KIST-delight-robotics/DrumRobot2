#!/usr/bin/env python3

def match_best_from_cache(midi_path: str,
                          cache_path: str = "library_cache.npz",
                          weights: tuple[float, float] = (0.6, 0.4),
                          return_score: bool = False):
    """
    입력 MIDI(항상 2마디 기준으로 처리)를 캐시(library_cache.npz)에 들어있는 베이직 패턴들과 비교하여
    가장 유사한 파일명을 반환한다.

    Params
    -------
    midi_path : str
        비교할 MIDI 파일 경로.
    cache_path : str, default "library_cache.npz"
        미리 생성한 캐시 파일 경로. 킥/스네어 전용(64차원)이어야 함.
    weights : (float, float), default (0.6, 0.4)
        (kick_weight, snare_weight). 킥/스네어 해밍 유사도 가중 평균에 사용.
    return_score : bool, default False
        True면 (best_filename, best_similarity_float) 튜플을 반환.

    Returns
    -------
    str | (str, float)
        가장 유사한 파일명 문자열 또는 (파일명, 유사도[0~1]) 튜플.

    Raises
    ------
    FileNotFoundError : cache_path 또는 midi_path가 없을 때
    ValueError        : 캐시 레이아웃이 킥/스네어(64차원)과 다를 때
    """
    # 내부에서만 쓰는 의존성 import (외부 네임스페이스 오염 방지)
    import os
    import numpy as np
    from mido import MidiFile

    # ---------- 내부 상수/셋업 ----------
    BAR_LEN_BEATS = 4.0
    STEPS_PER_BAR = 16
    TARGET_BARS   = 2
    KICK_NOTES    = {35, 36}
    SNARE_NOTES   = {38, 40}

    # ---------- 내부 유틸 ----------
    def _midi_to_grids_01(_midi_path: str, velocity_min: int = 1, max_bars: int = TARGET_BARS):
        """MIDI -> [{kick:[0/1x16], snare:[0/1x16]}, ...] (bar 리스트)"""
        mid = MidiFile(_midi_path)
        tpq = mid.ticks_per_beat

        events = []  # (abs_tick, note, vel)
        for track in mid.tracks:
            cur = 0
            for msg in track:
                cur += msg.time
                if msg.type == "note_on" and msg.velocity > 0:
                    events.append((cur, msg.note, msg.velocity))
        events.sort(key=lambda x: x[0])

        grids = [{"kick":[0]*STEPS_PER_BAR, "snare":[0]*STEPS_PER_BAR} for _ in range(max_bars)]
        for abs_tick, note, vel in events:
            if vel < velocity_min:
                continue
            beat = abs_tick / tpq
            if beat < 0:
                continue
            bar_idx = int(beat // BAR_LEN_BEATS)
            if bar_idx >= max_bars:
                break
            pos  = beat - bar_idx * BAR_LEN_BEATS
            step = int(round((pos / BAR_LEN_BEATS) * STEPS_PER_BAR)) % STEPS_PER_BAR

            if note in KICK_NOTES:
                grids[bar_idx]["kick"][step] = 1
            elif note in SNARE_NOTES:
                grids[bar_idx]["snare"][step] = 1
        return grids

    def _ensure_two_bars(g):
        if len(g) >= 2:
            return g[:2]
        if len(g) == 1:
            g0 = g[0]
            g1 = {"kick": g0["kick"][:], "snare": g0["snare"][:]}
            return [g0, g1]
        Z = {"kick":[0]*STEPS_PER_BAR, "snare":[0]*STEPS_PER_BAR}
        return [Z, Z]

    def _concat_2bars_to_vector(g2):
        """2마디 -> 64차원 벡터(kick32 + snare32)"""
        import numpy as np
        def flat(inst): return g2[0][inst] + g2[1][inst]
        return np.array(flat("kick") + flat("snare"), dtype=np.uint8)

    def _hamming_sim_ks(vecA, vecB, w=(0.6, 0.4)) -> float:
        """킥/스네어 구간별 해밍 유사도 가중 평균"""
        import numpy as np
        if vecA.shape != (64,) or vecB.shape != (64,):
            raise ValueError("유사도 계산은 64차원 벡터만 지원합니다. (kick32|snare32)")
        kW, sW = w
        kA, sA = vecA[0:32],  vecA[32:64]
        kB, sB = vecB[0:32],  vecB[32:64]
        sim = lambda a, b: float(np.sum(a == b)) / a.size
        kSim, sSim = sim(kA, kB), sim(sA, sB)
        return (kSim * kW + sSim * sW) / (kW + sW)

    # ---------- 입력/캐시 확인 ----------
    if not os.path.isfile(cache_path):
        raise FileNotFoundError(f"캐시 파일이 없습니다: {cache_path}")
    if not os.path.isfile(midi_path):
        raise FileNotFoundError(f"입력 MIDI를 찾을 수 없습니다: {midi_path}")

    import numpy as np
    npz  = np.load(cache_path, allow_pickle=True)
    names   = npz["names"]          # object array of str
    vectors = npz["vectors"]        # (N, 64)
    meta    = npz["meta"][0]        # dict

    if vectors.ndim != 2 or vectors.shape[1] != 64:
        raise ValueError("캐시 레이아웃이 킥/스네어 전용(64차원)과 다릅니다. 캐시를 다시 빌드하세요.")

    # ---------- 입력 MIDI -> 64차원 벡터 ----------
    g    = _midi_to_grids_01(midi_path,
                             velocity_min=meta.get("velocity_min", 1),
                             max_bars=meta.get("target_bars", 2))
    qvec = _concat_2bars_to_vector(_ensure_two_bars(g))

    # ---------- 라이브러리와 유사도 계산 → 최고 점수 ----------
    best_idx = -1
    best_sim = -1.0
    for i, v in enumerate(vectors):
        sim = _hamming_sim_ks(qvec, v, w=weights)
        if sim > best_sim:
            best_sim = sim
            best_idx = i

    best_name = str(names[best_idx])
    return (best_name, float(best_sim)) if return_score else best_name



if __name__ == "__main__":
    try:
        # 필요시 파일명만 바꿔 테스트
        result = match_best_from_cache("input_1.mid", return_score=True)
        print("Best match:", result[0])
        print("Similarity:", f"{result[1]*100:.2f}%")
    except Exception as e:
        print("ERROR:", e)
