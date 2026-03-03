# AI Coding Agent Instructions for Phil Robot Codebase

## Project Overview
**Phil Robot** is a drum-playing robot with AI conversation capabilities, combining a C++ motor control system (DrumRobot2) with a Python AI/TTS interface (phil_robot). The system uses Jetson AGX Orin for real-time performance.

### Core Architecture
- **C++ Backend** (`DrumRobot2/`): Motor control, CAN communication, state machine (~9 motors via TMotor & Maxon drivers)
- **Python Brain** (`phil_robot/`): Speech recognition (Whisper), LLM reasoning (Ollama), text-to-speech (MeloTTS)
- **IPC Bridge**: TCP sockets on `127.0.0.1:9999` for command delivery and state updates (JSON)

### State Machine (Critical)
The robot operates in explicit states (`ROBOT_STATE["state"]`):
- `0` = Idle/Ready
- `2` = Playing (drum sequence executing)
- `4` = Error (joint collision/motor issue)

LLM responses must adapt behavior based on state context (see phil_brain.py lines 88-94).

## Build & Run

### DrumRobot2 (C++)
```bash
cd DrumRobot2
make clean && make           # Compiles with OpenCV, SFML, RealSense, ArUco
./bin/main.out               # Interactive: 'p' play, 'r' record, 's' stop, 'o' wait for Python connection
```
**Key Build Files**: `Makefile` (lines 1-80) specifies Jetson arm64 libraries, ArUco 3rdparty, OpenCV via pkg-config.

### Phil Robot (Python)
```bash
cd phil_robot
conda env create -f environment.yml
conda activate drum4
python phil_brain.py         # Runs interaction loop
```
**Pre-flight**: 
- Run `/init_phil.sh` to unlock Jetson performance & preload Ollama model
- Ensure C++ backend is running and accepts connection ('o' prompt)

## Key Patterns

### 1. LLM Response Protocol
LLM must respond in format: `[CMD:action:params][CMD:...] >> speech text`
- Commands extracted by `parse_llm_response()` (response_parser.py)
- Multiple commands per response supported
- Speech text (after `>>`) sent to MeloTTS for audio playback

Example: `[CMD:look:0,0][CMD:gesture:wave] >> 안녕! 반가워.`

### 2. State-Aware Context Injection
Before querying LLM, inject robot state context (phil_brain.py lines 88-94):
- Playing state: Refuse non-urgent requests
- Error state: Apologize for physical limitations
- Moving state: Indicate busy with positioning
- Idle state: Accept any command

### 3. Model Warm-up (GPU Initialization)
Both STT (Whisper) and TTS (MeloTTS) require dummy-run initialization to avoid GPU stalls:
- Whisper: Feed 2-second zero-padded audio before first transcription
- MeloTTS: Call `.speak()` with empty text on startup
This prevents latency spikes in real-time loops.

### 4. Motor Architecture (C++)
Motors mapped by name: `waist`, `R_arm1`, `L_arm1`, `R_arm2`, `R_arm3`, `L_arm2`, `L_arm3`, `R_wrist`, `L_wrist`, `R_foot`, `L_foot`
- **TMotor**: Joint motors with PWM control (AK10_9, AK70_10)
- **MaxonMotor**: Wrist/foot motors with EC-90 drivers
- CAN IDs: 0x00–0x0B mapped in DrumRobot.cpp:initializeMotors()
- Range limits: Injected via `jointRangeMin/Max` arrays

### 5. Multi-threaded C++ Execution
Main.cpp spawns 6+ threads with priority levels:
1. `sendThread` (priority 5): CAN message transmission
2. `receiveThread` (priority 4): CAN message reception
3. `stateThread` (priority 3): State machine logic
4. `musicThread`: Music playback coordination
5. `pythonThread`: Bridges to Python (TCP client)
6. `broadcastThread`: Sends JSON state updates to Python

Do not modify thread priorities without RTOS knowledge (Jetson real-time scheduling).

## Hardware Integration

### Port Detection
```bash
# PCAN-USB (CAN interface)
sudo uhubctl

# Dynamixel USB serial (motor control)
ls /dev/ttyUSB*

# Arduino/sensors
ls /dev/ttyACM*

# E-drum MIDI
aconnect -l
```

### USB Latency Tuning
For Dynamixel responsiveness, reduce latency timer:
```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## Data Flows

### Request Pipeline (User Speech → Robot Action)
1. **Capture**: `record_audio()` → 16kHz PCM via sounddevice
2. **Transcribe**: Whisper.transcribe() → Korean text
3. **LLM**: Ollama `phil-bot` model (requires `ollama serve` running on localhost:11434)
4. **Parse**: `parse_llm_response()` extracts [CMD:...] + speech
5. **Send**: `RobotClient.send_command(cmd)` → C++ via TCP
6. **Execute**: C++ state machine processes single-char commands
7. **Feedback**: C++ broadcasts state JSON → updates ROBOT_STATE global

### State Update Pipeline (C++ → Python)
C++ `broadcastStateThread` → JSON serialization (state, bpm, is_fixed) → TCP → Python receives via daemon thread in `_receive_loop()`.

## Common Workflows

### Testing Motor Response
1. Run `./bin/main.out`, press 'o' to enter wait-mode
2. Start Python `phil_brain.py` in another terminal
3. Speak command like "손을 올려" (raise hand)
4. LLM generates [CMD:action:params] → C++ processes → motor actuates

### Tests & Simulators
Several lightweight scripts are provided for exercising subsystems without the full robot.

* **Python unit-style tests** live alongside `phil_robot/`:
	- `test_speech.py` – threaded STT/LLM/TTS loop used during early development. Run with `python test_speech.py`.
	- `test_drum*.py` – variants that exercise the RobotClient and command parsing without audio; useful for mocking the C++ server.
	- `phil_robot/MeloTTS/test/` contains standard MeloTTS library tests.
	These scripts are run directly with the workspace Python environment (`conda activate drum4`).

* **C++ socket stub** lives in `DrumRobot2/tests/server_test.cpp`.  Build with `make` or manually compile and run to emulate the 127.0.0.1:9999 command port.  The program prints incoming single‑char commands (`r`, `p`, `s`, etc.) and is handy for debugging the TCP handshake.

There is no formal test framework; treat the above as examples and manual verifiers.

### Debugging Latency
- MeloTTS: Check inference + playback times printed in log
- Whisper: Compare timestamp at `record_audio()` vs result completion
- CAN: Monitor `receiveThread` loop timing (should be <10ms)

### Adding New Motor Action
1. Define command parser in C++ (CommandParser.cpp)
2. Add handler in AgentAction.cpp
3. Update Python LLM prompt to include new [CMD:...] pattern
4. Test with manual command injection in phil_brain.py

## Language & Conventions
- **Korean comments throughout** (user's native language)
- **Emoji markers** for state/action clarity (🎤 input, 🧠 thinking, 📡 transmission, ❌ error)
- **Config constants at module top** (SAMPLE_RATE, LLM_MODEL, HOST, PORT)
- **Exception handling**: Log + graceful degradation (never crash input loop)

## Critical Gotchas
1. **TCP Connection**: C++ must be running *before* Python connects (blocking retry loop)
2. **Model Loading**: LLM & TTS models require VRAM; preload with init_phil.sh
3. **JSON Parsing**: C++ expects newline-terminated JSON; Python must add '\n' after JSON.dumps()
4. **Dummy Audio Length**: Must be ≥1 sample; use 16000 samples (1 sec @ 16kHz) for safety
5. **Korean TTS**: MeloTTS preprocessor converts English acronyms (GPU→지피유); add custom replacements if needed

6. **Python test scripts** should be executed from the `phil_robot` directory so relative imports work.
