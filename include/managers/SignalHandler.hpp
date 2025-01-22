#ifndef SIGNAL_HANDLER_HPP
#define SIGNAL_HANDLER_HPP

#include <signal.h>
#include <iostream>

class SignalHandler {
public:
    // Initialize the signal handler
    static void initialize();

private:
    // SIGINT signal handler
    static void handleSignal(int signal); // ^C 신호를 핸들해주는 전체 함수

    // Define actions for the signal
    static void clear_brake();
    static void shut_down();
};

#endif // SIGNAL_HANDLER_HPP
