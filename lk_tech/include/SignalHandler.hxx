#ifndef __SIGNAL_HANDLER_HXX__
#define __SIGNAL_HANDLER_HXX__


#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>

class SignalHandler {
public:
    static std::atomic<bool> stopRequested;

    static void signalHandler(int signal) {
        if (signal == SIGINT) {
            stopRequested = true;
        }
    }

    template <typename F>
    void run(F&& func) {
        signal(SIGINT, signalHandler);
        
        while (!stopRequested) {
            func();
        }
        std::cout << "Stopped." << std::endl;
    }
};

std::atomic<bool> SignalHandler::stopRequested(false); // Initialize outside the class definition



#endif