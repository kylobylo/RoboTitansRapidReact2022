#include <fstream>
#include <iostream>
#include <string>

class dbg {
    public:
        std::ofstream file;
        void out(std::string x);
        bool initialize(std::string y = "/home/lvuser/DEBUG.txt");
        bool end();
        int iterations;
        ~dbg();
};