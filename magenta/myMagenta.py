import os
from magenta.models.music_vae import TrainedModel
from magenta.models.music_vae import configs
import note_seq

class MyMagenta:
    
    def __init__(self, config_name, checkpoint_path):
        # 사용할 모델 이름 설정
        self.config_name = config_name
        # 모델 체크포인트 파일 경로 설정
        self.checkpoint_path = checkpoint_path

        # 모델 설정 로드
        self.config = configs.CONFIG_MAP[self.config_name]

        # 사전 학습된 MusicVAE 모델 로드
        self.model = TrainedModel(self.config, batch_size=1, checkpoint_dir_or_path=self.checkpoint_path)

        # 파라미터 초기화
        self.num_samples = 5
        self.length = 32 # 2마디, 마디당 16단계
        self.temperature = 0.2

        # 저장 경로 (폴더 이름)
        self.output_dir = 'generated'

    def reload_model(self, config_name, checkpoint_path):
        # 사용할 모델 이름 설정
        self.config_name = config_name
        # 모델 체크포인트 파일 경로 설정
        self.checkpoint_path = checkpoint_path

        # 모델 설정 로드
        self.config = configs.CONFIG_MAP[self.config_name]

        # 사전 학습된 MusicVAE 모델 로드
        self.model = TrainedModel(self.config, batch_size=1, checkpoint_dir_or_path=self.checkpoint_path)

    def set_parameter(self, num_samples, length, temperature):
        # 파라미터 설정
        self.num_samples = num_samples
        self.length = length
        self.temperature = temperature

    def set_save_path(self, output_dir):
        self.output_dir = output_dir

    def print_model(self):
        print(f"\nModel Name: {self.config_name}")
        print(f"Model Checkpoint Path: {self.checkpoint_path}\n")
    
    def generate_music(self, output_filename):
        # 생성
        generated_sequences = self.model.sample(n=self.num_samples, length=self.length, temperature=self.temperature)

        # 생성된 MIDI 파일 저장할 경로 설정
        os.makedirs(self.output_dir, exist_ok=True)

        # 생성된 드럼 패턴을 MIDI 파일로 저장
        for i, sequence in enumerate(generated_sequences):
            midi_filename = output_filename + f"{i+1}.mid"
            output_path = os.path.join(self.output_dir, midi_filename)
            note_seq.sequence_proto_to_midi_file(sequence, output_path)
            print(f"Generated MIDI file: {midi_filename}")

    def generate_music_from_input(self, input_midi, output_filename):
        # 입력 MIDI 파일 변환
        input_sequence = note_seq.midi_file_to_sequence_proto(input_midi)

        # 생성
        generated_sequences = self.model.sample(n=self.num_samples, length=self.length, temperature=self.temperature, c_input=input_sequence)

        # 생성된 MIDI 파일 저장할 경로 설정
        os.makedirs(self.output_dir, exist_ok=True)

        # 생성된 드럼 패턴을 MIDI 파일로 저장
        for i, sequence in enumerate(generated_sequences):
            midi_filename = output_filename + f"{i+1}.mid"
            output_path = os.path.join(self.output_dir, midi_filename)
            note_seq.sequence_proto_to_midi_file(sequence, output_path)
            print(f"Generated MIDI file: {midi_filename}")

    def interpolate_music(self, input_midi1, input_midi2, output_filename):
        # 입력 MIDI 파일 변환
        input_sequence1 = note_seq.midi_file_to_sequence_proto(input_midi1)
        input_sequence2 = note_seq.midi_file_to_sequence_proto(input_midi2)

        # 보간
        generated_sequences = self.model.interpolate(start_sequence=input_sequence1, end_sequence=input_sequence2, num_steps=self.num_samples, length=self.length, temperature=self.temperature, assert_same_length=False)

        # 생성된 MIDI 파일 저장할 경로 설정
        os.makedirs(self.output_dir, exist_ok=True)

        # 생성된 드럼 패턴을 MIDI 파일로 저장
        for i, sequence in enumerate(generated_sequences):
            midi_filename = output_filename + f"{i+1}.mid"
            output_path = os.path.join(self.output_dir, midi_filename)
            note_seq.sequence_proto_to_midi_file(sequence, output_path)
            print(f"Generated MIDI file: {midi_filename}")

# 모델 이름 - 모델 체크포인트 파일 경로
# 'cat-drums_2bar_small' - '/home/shy/IW_test/magenta/model/cat-drums_2bar_small.lokl.tar'
# 'cat-drums_2bar_small' - '/home/shy/IW_test/magenta/model/cat-drums_2bar_small.hikl.tar' (interpolate)
# 'hierdec-trio_16bar' - '/home/shy/IW_test/magenta/model/hierdec-trio_16bar.tar'
# 'groovae_2bar_humanize' - '/home/shy/IW_test/magenta/model/groovae_2bar_humanize.tar'

# 명령어로 실행
# music_vae_generate \
#   --config=cat-drums_2bar_small \
#   --checkpoint_file=/home/shy/IW_test/magenta/model/cat-drums_2bar_small.lokl.tar \
#   --mode=sample \
#   --num_outputs=5 \
#   --output_dir=/home/shy/IW_test/magenta/generated
  
# music_vae_generate \
# --config=cat-drums_2bar_small \
# --checkpoint_file=/home/shy/IW_test/magenta/model/cat-drums_2bar_small.lokl.tar \
# --mode=interpolate \
# --num_outputs=5 \
# --input_midi_1=/home/shy/IW_test/magenta/generated/0.mid \
# --input_midi_2=/home/shy/IW_test/magenta/generated/1.mid \
# --output_dir=/home/shy/IW_test/magenta/generated