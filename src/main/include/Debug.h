#include <fstream>
#include <iostream>
#include <string>

class dbg {
    public:
        std::ofstream file;
        void out(std::string x);
        bool initialize();
        bool end();
        int iterations;
};