#include "config.h"
#include "library.h"
#include <argparse/argparse.hpp>
#include <fstream>
#include <iostream>
#ifdef USE_MYMATH
#include "MathFunctions.h"
#else
#include <cmath>
#endif
#include "nlohmann/json.hpp"

using namespace std;


void jsonPractice(const string input, const string output) {
    nlohmann::json j;
    // add a number that is stored as double (note the implicit conversion of j to an object)
    j["pi"] = 3.141;

    // add a Boolean that is stored as bool
    j["happy"] = true;

    // add a string that is stored as std::string
    j["name"] = "Niels";

    // add another null object by passing nullptr
    j["nothing"] = nullptr;

    // add an object inside the object
    j["answer"]["everything"] = 42;

    // add an array that is stored as std::vector (using an initializer list)
    j["list"] = {1, 0, 2};

    // add another object (using an initializer list of pairs)
    j["object"] = {{"currency", "USD"}, {"value", 42.99}};

    // instead, you could also write (which looks very similar to the JSON above)
    nlohmann::json j2 = {
            {"pi", 3.141},
            {"happy", true},
            {"name", "Niels"},
            {"nothing", nullptr},
            {"answer", {{"everything", 42}}},
            {"list", {1, 0, 2}},
            {"object", {{"currency", "USD"}, {"value", 42.99}}}};

    // Serialization / Deserialization
    // To/from strings
    // by appending _json

    // create object from string literal
    nlohmann::json j3 = "{ \"happy\": true, \"pi\": 3.141 }"_json;

    // or even nicer with a raw string literal
    auto j4 = R"(
        {
        "happy": true,
            "pi": 3.141
          }
        )"_json;

    string s = j.dump();
    cout << "dump to string: " << s << endl;

    // read from file
    ifstream i(input);
    nlohmann::json j5;
    i >> j;
    ofstream o(output);
    // the setw manipulator was overloaded to set the indentation for pretty printing
    o << setw(4) << j << endl;
    cout << "write json to file:" << output << endl;
}

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("main");
    program.add_argument("square")
            .help("display the square of a given integer")
            .scan<'i', int>();
    //    program.add_argument("-v", "--verbose");// parameter packing
    program.add_argument("-i", "--input");
    program.add_argument("-o", "--output");

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

    std::cout << "math power:" << std::endl;
    double base = 2;
    int exponent = 4;
#ifdef USE_MYMATH
    cout << "Use our own math library" << endl;
    double result = power(base, exponent);
#else
    cout << "Use the standard library" << endl;
    double result = pow(base, exponent);
#endif
    cout << base << "^" << exponent << "=" << result << endl;


    // json test
    cout << "Json practice" << endl;
    auto f_in = program.get("i");
    auto f_out = program.get("o");
    jsonPractice(f_in, f_out);


    return 0;
}
