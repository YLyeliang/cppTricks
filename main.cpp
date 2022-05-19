#include "library.h"
#include <argparse/argparse.hpp>
#include <iostream>

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("main");
    program.add_argument("foo");
    program.add_argument("square")
            .help("display the square of a given integer")
            .scan<'i', int>();
    program.add_argument("-v", "--verbose");// parameter packing

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    auto input = program.get<int>("square");
    std::cout << (input * input) << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
