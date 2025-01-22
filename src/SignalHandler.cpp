#include "SignalHandler.hpp"

// Initialize the signal handler
void SignalHandler::initialize() {
    struct sigaction sa;
    sa.sa_handler = SignalHandler::handleSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (sigaction(SIGINT, &sa, nullptr) == -1) {
        std::cerr << "Failed to set up SIGINT handler" << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Signal handling logic
void SignalHandler::handleSignal(int signal) {
    if (signal == SIGINT) {
        clear_brake();
        shut_down();
        std::cout << "Signal SIGINT handled. Exiting..." << std::endl;
        exit(0); // Gracefully exit
    }
}

// Define clear_brake function
void SignalHandler::clear_brake() {
    std::cout << "Clearing brake..." << std::endl;
    // Add logic to clear brake here
}

// Define shut_down function
void SignalHandler::shut_down() {
    std::cout << "Shutting down system..." << std::endl;
    // Add logic to shut down resources here
}
