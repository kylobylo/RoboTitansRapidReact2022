
#pragma once 

#include <fstream>
#include <iostream>
#include <string>

class dbg {
    public:
        std::ofstream file;
        void out(std::string x);
        dbg(std::string y = "home/lvuser/DEBUG.txt");
        bool end();
        int iterations;
        ~dbg();
};