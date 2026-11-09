// tdr_static: command-line front end of the static reproduction (M8). All logic lives in runner/cli.hpp.
#include <iostream>
#include <string>
#include <vector>

#include "runner/cli.hpp"

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    return tdr::runner::cli_main(args, std::cout, std::cerr);
}
