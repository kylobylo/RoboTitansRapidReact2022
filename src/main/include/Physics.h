#pragma once

#include "Game.h"

class Physics{
public:
    static double getVelocity(double distance, double goal);
    static double getMinDistance(double goal);
    static double getHeight(double velocity);
    static double getTime(double height, double goal, double velocity);
    static double getShotRPM(double velocity);
};