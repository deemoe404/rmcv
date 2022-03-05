#include <iostream>

#include <thread>
#include "rmcv/rmcv.h"

int main() {
    rm::SerialPort sp, sp2;
    sp.Initialize();
    sp2.Initialize();

    rm::Response response = {20, 20, 20};
    rm::Request request;

    std::thread th1([&]() {
        std::cout << sp.Receive(request) << std::endl;
    });

    std::cout << sp.Send(response) << std::endl;

    th1.join();

    sp.Destroyed();
    sp2.Destroyed();

    return 0;
}
