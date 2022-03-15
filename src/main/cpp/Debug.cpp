#include "Debug.h"
#include <fstream>
#include <iostream>
#include <string>
using namespace std;

dbg internalDebug;
bool dbg::initialize(std::string y = "/home/lvuser/DEBUG.txt") {
    internalDebug.file.open(y);
    internalDebug.iterations = 0;
    if(internalDebug.file.is_open()) {
        return true;
    } else {
        return false;
    }
}
void dbg::out(string x) {
    internalDebug.iterations++;
    internalDebug.file << "[" << internalDebug.iterations << "]" << x << endl;
}
bool dbg::end() {
    internalDebug.file << "[ROBOT SHUT OFF]";
    internalDebug.file.close();
    if(!internalDebug.file.is_open()) {
        return true;
    } else {
        return false;
    }
}
dbg::~dbg() {
    internalDebug.end();
}