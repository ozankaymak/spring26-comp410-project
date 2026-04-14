#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace hyper::test {

constexpr double kDefaultTolerance = 1.0e-9;

inline void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

inline void require_close(double actual, double expected, const std::string& message,
                          double tolerance = kDefaultTolerance) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + ": expected " + std::to_string(expected) +
                                 ", got " + std::to_string(actual));
    }
}

template <typename Exception, typename Fn>
void require_throws(Fn&& fn, const std::string& message) {
    bool threw = false;
    try {
        std::forward<Fn>(fn)();
    } catch (const Exception&) {
        threw = true;
    }

    require(threw, message);
}

template <typename Fn>
int run(const char* suite_name, Fn&& fn) {
    try {
        std::forward<Fn>(fn)();
    } catch (const std::exception& error) {
        std::cerr << suite_name << " failed: " << error.what() << '\n';
        return 1;
    }

    return 0;
}

} // namespace hyper::test

