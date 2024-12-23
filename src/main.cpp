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

    Functions fun(motors);
    CanManager canManager(motors, fun);
    PathManager pathManager(state, canManager, motors, fun);
    TestManager testManager(state, canManager, motors, usbio, fun);

    DrumRobot drumRobot(state, canManager, pathManager, testManager, motors, usbio, fun);
    GuiManager guiManager(state, canManager, motors);

    //shy-desktop -> 1반환
    //shy-MINIPC-VC66-C2 -> 2반환
    // int com_number = canManager.get_com_number_by_hostname();
    int com_number = fun.get_com_number_by_hostname();

    // 포트를 비활성화하고 다시 활성화
    // canManager.restCanPort(com_number);
    fun.restCanPort(com_number);

    // Create Threads
    std::thread stateThread(&DrumRobot::stateMachine, &drumRobot);
    std::thread sendThread(&DrumRobot::sendLoopForThread, &drumRobot);
    std::thread receiveThread(&DrumRobot::recvLoopForThread, &drumRobot);
    //std::thread guiThread(&GuiManager::guiThread, &guiManager);
    
    // Threads Priority Settings
    if (!setThreadPriority(sendThread, 3))
    {
        std::cerr << "Error setting priority for sendCanFrame" << std::endl;
        return -1;
    }
    if (!setThreadPriority(receiveThread, 2))
    {
        std::cerr << "Error setting priority for receiveCanFrame" << std::endl;
        return -1;
    }
    if (!setThreadPriority(stateThread, 1))
    {
        std::cerr << "Error setting priority for stateMachine" << std::endl;
        return -1;
    }
    //if (!setThreadPriority(guiThread, 4))
    //{
    //    std::cerr << "Error setting priority for stateMachine" << std::endl;
    //    return -1;
    //}


    // Wait Threads
    stateThread.join();
    sendThread.join();
    receiveThread.join();
    //guiThread.join();
}
