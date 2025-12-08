#include <thread>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <memory>
#include <map>
#include <atomic>

#include "../include/motors/Motor.hpp"
#include "../include/managers/PathManager.hpp"
#include "../include/managers/CanManager.hpp"
#include "../include/managers/TestManager.hpp"
#include "../include/tasks/DrumRobot.hpp"
#include "../include/tasks/SystemState.hpp"
#include "../include/managers/GuiManager.hpp"
#include "../include/USBIO_advantech/USBIO_advantech.hpp"
#include "../include/tasks/Functions.hpp"

using namespace std;

// 스레드 우선순위 설정 함수
bool setThreadPriority(std::thread &th, int priority, int policy = SCHED_FIFO)
{
    sched_param sch_params;
    sch_params.sched_priority = priority;
    if (pthread_setschedparam(th.native_handle(), policy, &sch_params))
    {
        std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char *argv[])
{
    // Create Share Resource
    State state;
    std::map<std::string, std::shared_ptr<GenericMotor>> motors;
    USBIO usbio;

    Functions func(motors);
    CanManager canManager(motors, func, usbio);
    PathManager pathManager(state, canManager, motors, usbio, func);
    TestManager testManager(state, canManager, motors, usbio, func);
    DrumRobot drumRobot(state, canManager, pathManager, testManager, motors, usbio, func);
    GuiManager guiManager(state, canManager, motors);
    
    // sf::Music warmup;
    // warmup.setVolume(50);
    
    // 포트를 비활성화하고 다시 활성화
    func.restCanPort();
    // 드럼 로봇 초기화
    drumRobot.initializeDrumRobot();

    // Create Threads
    std::thread stateThread(&DrumRobot::stateMachine, &drumRobot);
    std::thread sendThread(&DrumRobot::sendLoopForThread, &drumRobot);
    std::thread receiveThread(&DrumRobot::recvLoopForThread, &drumRobot);
    std::thread musicThread(&DrumRobot::musicMachine, &drumRobot);
    std::thread pythonThread(&DrumRobot::runPythonInThread, &drumRobot);
    //std::thread guiThread(&GuiManager::guiThread, &guiManager);
    
    // Threads Priority Settings
    if (!setThreadPriority(sendThread, 5))
    {
        std::cerr << "Error setting priority for sendCanFrame" << std::endl;
        return -1;
    }
    if (!setThreadPriority(receiveThread, 4))
    {
        std::cerr << "Error setting priority for receiveCanFrame" << std::endl;
        return -1;
    }
    if (!setThreadPriority(stateThread, 3))
    {
        std::cerr << "Error setting priority for stateMachine" << std::endl;
        return -1;
    }
    if (!setThreadPriority(musicThread, 2))
    {
        std::cerr << "Error setting priority for watchThread" << std::endl;
        return -1;
    }
    if (!setThreadPriority(pythonThread, 1))
    {
        std::cerr << "Error setting priority for watchThread" << std::endl;
        return -1;
    }
    //if (!setThreadPriority(guiThread, 1))
    //{
    //    std::cerr << "Error setting priority for stateMachine" << std::endl;
    //    return -1;
    //}

    // Wait Threads
    stateThread.join();
    sendThread.join();
    receiveThread.join();
    musicThread.join();
    pythonThread.join();
    //guiThread.join();
}