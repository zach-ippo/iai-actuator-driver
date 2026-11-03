// iai-actuator-driver-cpp: the command line of the C++ edition (see cli.hpp). Unofficial: not an IAI product.
#include <csignal>
#include <iostream>
#include <string>
#include <vector>

#include "cli.hpp"

namespace {

extern "C" void ec_cli_on_sigint(int) {
    iai_actuator_driver::cli::request_interrupt();   // the running motion command stops its axis, then exits with 130
    std::signal(SIGINT, SIG_DFL);           // a second Ctrl+C ends the program at once
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, ec_cli_on_sigint);
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);
    return iai_actuator_driver::cli::run(args, std::cout, std::cerr);
}
