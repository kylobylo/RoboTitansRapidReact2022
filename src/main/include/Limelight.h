#pragma once

#include "Game.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <string>

class Limelight{
public:
    static auto getTable();
    static void toggleCamera();
    static void toggleCamera(int);
    static void putInfo(const std::string&, double);
    static double getDistance();
    static double getHorizontalOffset();
    static double getInfo(const std::string&);
    static bool visibleTarget();
};
