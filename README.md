# 🥁 Phil Robot: AI-Powered Humanoid Drummer
> **Jetson AGX Orin 기반의 로컬 AI 에이전트와 실시간 C++ 모터 제어 시스템의 통합 아키텍처**

![Phil Robot Preview](https://www.youtube.com/watch?v=SHXWN2f2ou8)
*(https://www.youtube.com/watch?v=SHXWN2f2ou8)*

## 📖 Project Overview
**Phil Robot**은 KIST에서 개발된 휴머노이드 드럼 로봇 '필(Phil)'에게 AI 기반의 인지/대화 능력을 부여하는 프로젝트입니다. 
외부 클라우드 API에 의존하지 않고, 엣지 디바이스(Jetson AGX Orin) 내에서 독립적으로 구동되는 **On-Device AI 파이프라인(Whisper ↔ Qwen 30B ↔ MeloTTS)**과 실시간성이 요구되는 **C++ 하드웨어 제어 시스템(CAN 통신)**을 이기종 소켓 통신으로 완벽하게 통합했습니다.

## 🏗️ System Architecture (Level 1)
Python(AI Brain)과 C++(Robot Body)을 철저히 분리하여, AI 연산이 실시간 모터 제어 루프를 방해하지 않도록 비동기 IPC 구조를 채택했습니다.

```mermaid
---
config:
  layout: dagre
---
graph LR
    %% Styling Definitions
    classDef pythonBox fill:#E1F5FE,stroke:#0288D1,stroke-width:2px,color:#000;
    classDef cppBox fill:#E8F5E9,stroke:#388E3C,stroke-width:2px,color:#000;
    classDef hardware fill:#FFF3E0,stroke:#F57C00,stroke-width:2px,color:#000;
    classDef highlight fill:#FCE4EC,stroke:#C2185B,stroke-width:2px,stroke-dasharray: 5 5,color:#000;

    subgraph Brain ["🧠 AI Agent Process (Python / Jetson GPU)"]
        direction TB
        Mic((Mic Input)) --> STT["Whisper<br/>(Local STT)"]
        STT --> LLM{"Qwen-3 30B (Q4_K_M)<br/>via Ollama Engine<br/>w/ Harness Engineering"}
        LLM --> TTS["MeloTTS<br/>(Local TTS)"]
        LLM --> PySocket["TCP Client Socket"]
        
        class Mic,STT,LLM,TTS,PySocket pythonBox;
    end

    subgraph Body ["🦾 Robot Control Process (C++ / Real-time CPU)"]
        direction TB
        CppSocket["Agent Socket Server"] --> MutexQueue[/"Command Queue<br/>(Protected by std::mutex)"/]
        MutexQueue --> ActionDispatcher["AgentAction<br/>(Regex Command Parser)"]
        ActionDispatcher --> CanManager["CAN Manager"]
        CanManager --> Motors(("Robot Joints<br/>(Maxon / T-Motor)"))
        
        class CppSocket,MutexQueue,ActionDispatcher,CanManager cppBox;
        class Motors hardware;
    end

    %% IPC Connection (Asymmetric Communication)
    PySocket ==>|"1. Command (Low Latency)<br/>⚡ Regex Parsed Text"| CppSocket
    CppSocket -.->|"2. State Feedback<br/>📦 JSON Format"| PySocket
    
    class PySocket,CppSocket highlight;

    %% TTS Output
    TTS -.->|"Audio Feedback"| Speaker((Speaker))
    class Speaker hardware;