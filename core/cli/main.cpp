// iai-actuator-driver-cpp: the command line of the C++ edition (see cli.hpp). Unofficial: not an IAI product.
#include <iostream>
#include <string>
#include <vector>

#include "cli.hpp"

int main(int argc, char** argv) {
    iai_actuator_driver::cli::install_interrupt_handlers();  // Ctrl+C / Ctrl+Break: stop the axis, exit with 130
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);
    return iai_actuator_driver::cli::run(args, std::cout, std::cerr);
}
