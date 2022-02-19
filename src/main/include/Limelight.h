#pragma once

#include "Game.h"

class Limelight{
public:
    static void toggleCamera();
    static void toggleCamera(int mode);
    static double getDistance();
    static bool visibleTarget();
};
