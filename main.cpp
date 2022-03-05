#include <iostream>

#include "rmcv/rmcv.h"

int main() {
    rm::SerialPort sp;
    std::cout << sp.Initialize() << std::endl;
    rm::Response response = {20, 20, 20};
    std::cout << sp.Send(response) << std::endl;
    rm::Request request;
    std::cout << sp.Receive(request) << std::endl;
    sp.Destroyed();

    return 0;
}
