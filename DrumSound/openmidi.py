import os

def view_midi_as_hex(midi_filepath, bytes_per_line=16):
    """
    MIDI 파일을 16진수 형식으로 읽어 출력합니다.

    Args:
        midi_filepath (str): MIDI 파일의 경로.
        bytes_per_line (int): 한 줄에 표시할 16진수 바이트 수.
    """
    if not os.path.exists(midi_filepath):
        print(f"오류: '{midi_filepath}' 파일을 찾을 수 없습니다.")
        return

    print(f"--- MIDI 파일 '{midi_filepath}' 내용 (16진수) ---")
    print(f"한 줄에 {bytes_per_line} 바이트씩 표시됩니다.\n")

    try:
        with open(midi_filepath, 'rb') as f:
            offset = 0
            while True:
                # 지정된 바이트 수만큼 읽어옵니다.
                chunk = f.read(bytes_per_line)
                if not chunk:
                    break # 파일의 끝에 도달하면 종료

                # 오프셋 (파일 시작부터의 위치)을 16진수 8자리로 포맷팅
                hex_offset = f"{offset:08X}"

                # 바이트들을 16진수 문자열로 변환하고 공백으로 구분
                # 각 바이트를 2자리 16진수로 포맷팅합니다.
                hex_bytes = ' '.join([f"{byte:02X}" for byte in chunk])

                # ASCII 문자열 표현 (출력 가능한 문자만 표시)
                # 출력 가능한 ASCII 문자는 그대로, 아니면 '.'으로 표시
                ascii_chars = ''.join([chr(byte) if 32 <= byte <= 126 else '.' for byte in chunk])

                # 최종 출력 형식: 오프셋 | 16진수 바이트 | ASCII 문자
                print(f"{hex_offset} | {hex_bytes.ljust(bytes_per_line * 3 - 1)} | {ascii_chars}")

                offset += len(chunk)

    except IOError as e:
        print(f"파일을 읽는 중 오류가 발생했습니다: {e}")
    except Exception as e:
        print(f"예상치 못한 오류가 발생했습니다: {e}")

    print("\n----- 출력 끝 -----")

# --- 사용 예시 ---
# 실제 MIDI 파일 경로로 변경해주세요.
midi_file_to_view = "/home/shy/DrumRobot/DrumSound/input.mid"
#midi_file_to_view = "/home/shy/DrumRobot/DrumSound/output.mid" # 다른 파일로 변경 가능
view_midi_as_hex(midi_file_to_view)

