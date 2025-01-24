import numpy as np
import pandas as pd

# 드럼 위치 좌표 정의
DRUM_POSITIONS = {
    0: (0, 0, 0),  # 연주하지 않음
    1: (0, 1, 0.5),  # 스네어
    2: (1, 0, 0.4),  # 플로어 탐
    3: (0.5, 1.5, 0.7),  # 미드 탐
    4: (0.5, 2, 0.8),  # 하이 탐
    5: (-0.5, 2, 1.0),  # 하이햇
    6: (-1, 2, 1.2),  # 라이드 벨
    7: (-1.5, 1.5, 1.1),  # 라이트 크래쉬
    8: (-1.5, 0.5, 0.9),  # 레프트 크래쉬
}

# 로봇 팔 제한 조건
ARM_LENGTH = 1.5  # 최대 팔 길이
MIN_SAFE_DISTANCE = 0.3  # 손 간 최소 거리

# 드럼 악보 데이터를 읽기
def read_drum_data(file_path):
    columns = ["Measure", "Time", "RightHandPos", "LeftHandPos", "Dummy1", "Dummy2", "Dummy3"]
    drum_data = pd.read_csv(file_path, sep="\t", names=columns, skiprows=1)
    return drum_data[["Measure", "Time", "RightHandPos", "LeftHandPos"]]

# 안전성 판단 함수
def check_safety(right_pos, left_pos):
    right_coords = np.array(DRUM_POSITIONS.get(right_pos, (0, 0, 0)))
    left_coords = np.array(DRUM_POSITIONS.get(left_pos, (0, 0, 0)))
    
    # 1) 팔 길이 초과 확인
    if np.linalg.norm(right_coords) > ARM_LENGTH or np.linalg.norm(left_coords) > ARM_LENGTH:
        return False, "팔 길이 초과"
    
    # 2) 충돌 위험 확인
    distance_between_hands = np.linalg.norm(right_coords - left_coords)
    if distance_between_hands < MIN_SAFE_DISTANCE:
        return False, f"충돌 위험: 손 간 거리 {distance_between_hands:.2f}m"
    
    return True, "안전"

# 위험한 부분 수정 함수
def adjust_drum_data(drum_data):
    modified_data = drum_data.copy()
    for i in range(len(modified_data)):
        right_pos = modified_data.loc[i, "RightHandPos"]
        left_pos = modified_data.loc[i, "LeftHandPos"]
        
        # 현재 동작이 위험한지 확인
        is_safe, message = check_safety(right_pos, left_pos)
        if not is_safe:
            print(f"Measure {modified_data.loc[i, 'Measure']}, Time {modified_data.loc[i, 'Time']}: {message}")
            # 위험한 경우 해당 동작을 0으로 변경
            modified_data.loc[i, "RightHandPos"] = 0
            modified_data.loc[i, "LeftHandPos"] = 0
            
    return modified_data

# 수정된 악보 저장 함수
def save_modified_drum_data(modified_data, output_path):
    modified_data.to_csv(output_path, sep="\t", index=False, header=False)

# 메인 실행 함수
def main():
    input_path = "./include/codes/codeMOY_easy0.txt"  # 입력 파일 경로
    output_path = "modified_drum_score.txt"  # 수정된 파일 저장 경로
    
    # 드럼 악보 데이터 읽기
    drum_data = read_drum_data(input_path)
    
    # 위험한 동작 수정
    modified_data = adjust_drum_data(drum_data)
    
    # 수정된 악보 저장
    save_modified_drum_data(modified_data, output_path)
    print(f"수정된 드럼 악보가 {output_path}에 저장되었습니다.")

if __name__ == "__main__":
    main()
