# Python 3.7.17 가상 환경 설정 가이드
이 문서는 새 Linux 환경에서 Python 3.7.17 버전의 개발 환경을 설정하는 방법을 안내합니다. pyenv를 사용하여 여러 Python 버전을 관리하고, venv 모듈을 사용하여 프로젝트별 가상 환경을 생성합니다.

## 1. pyenv 설치 및 설정
pyenv는 여러 Python 버전을 유연하게 설치하고 관리할 수 있도록 해주는 도구입니다.

### 1.1. 필수 시스템 라이브러리 설치
Python을 소스 코드로부터 빌드하는 데 필요한 시스템 라이브러리들을 설치합니다. 사용하는 Linux 배포판에 따라 명령어가 다를 수 있습니다.

Ubuntu/Debian 기반 시스템:

```bash
sudo apt update
sudo apt install -y build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev
```

### 1.2. pyenv 설치
Git 저장소를 클론하여 pyenv를 설치합니다.

```bash
git clone https://github.com/pyenv/pyenv.git ~/.pyenv
```

### 1.3. 쉘 환경 설정
pyenv가 올바르게 작동하도록 환경 설정 파일에 경로를 추가합니다.

Bash 쉘 (~/.bashrc):

```bash
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init --path)"' >> ~/.bashrc
echo 'eval "$(pyenv init -)"' >> ~/.bashrc
```

### 1.4. 쉘 설정 적용
변경 사항을 적용하려면 터미널을 닫고 다시 열거나 다음 명령어를 실행합니다.

```bash
source ~/.bashrc # 또는 source ~/.zshrc
```

## 2. Python 3.7.17 설치
pyenv를 사용하여 Python 3.7.17을 설치합니다.

```bash
pyenv install 3.7.17
```

이 과정은 Python 소스를 다운로드하고 컴파일하므로 시스템 성능에 따라 몇 분 정도 소요될 수 있습니다.

## 3. Python 3.7.17 가상 환경 생성 및 활성화
프로젝트 디렉토리로 이동하여 Python 3.7.17을 기반으로 하는 새로운 가상 환경을 생성합니다.

### 3.1. 프로젝트 디렉토리로 이동

```bash
cd /path/to/your/project # 실제 프로젝트 디렉토리 경로로 변경하세요.
```

### 3.2. pyenv로 Python 버전 설정
현재 디렉토리에서 Python 3.7.17을 사용하도록 설정합니다. 이렇게 하면 이 디렉토리 안에서는 python 명령어가 3.7.17을 가리키게 됩니다.

```bash
pyenv local 3.7.17
```

### 3.3. 가상 환경 생성
venv 모듈을 사용하여 새로운 가상 환경을 생성합니다. 

```bash
python -m venv magenta-env
```

### 3.4. 가상 환경 활성화
생성된 가상 환경을 활성화합니다.

```bash
source magenta-env/bin/activate
```

터미널 프롬프트가 (magenta-env)와 같이 변경되면 가상 환경이 성공적으로 활성화된 것입니다.

### 3.5. Python 버전 확인
활성화된 가상 환경의 Python 버전이 3.7.17인지 확인합니다.

```bash
python --version
```

출력은 Python 3.7.17이어야 합니다.

## 4. 필요한 라이브러리 설치
이제 이 새로운 Python 3.7.17 가상 환경에 필요한 라이브러리들을 설치합니다. requirements.txt 파일이 있다면 편리하게 설치할 수 있습니다. 

(DrumSound 폴더 내의 requirement.txt)

```bash
pip install -r requirements.txt
```

참고: requirements.txt에 포함된 특정 라이브러리가 Python 3.7.17과 호환되지 않는 버전이라면 설치 오류가 발생할 수 있습니다. 이 경우 해당 라이브러리의 버전을 조절하거나 대안을 찾아야 합니다.


ALSA or JACK Audio Connenction Kit 관련 오류 발생 시 :

```bash
sudo apt update
sudo apt install libasound2-dev
sudo apt install libjack-dev
```


## 5. Python 코드 실행
작업을 마쳤다면 다음 명령으로 Python 코드를 실행할 수 있습니다.

(getMIDI_input.py는 다른 원하는 파일 이름으로 변경 가능)

```bash
/home/shy/DrumRobot/DrumSound/magenta-env/bin/python /home/shy/DrumRobot/DrumSound/getMIDI_input.py
```


## 6. 가상 환경 비활성화
작업을 마쳤다면 다음 명령으로 가상 환경을 비활성화할 수 있습니다.

```bash
deactivate
```



