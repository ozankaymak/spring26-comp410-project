#include "app.h"

#include <exception>
#include <iostream>

int main() {
    try {
        hyper::App app;
        app.run();
    } catch (const std::exception& error) {
        std::cerr << "hyperbolica failed: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
