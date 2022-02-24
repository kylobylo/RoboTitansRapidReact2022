#pragma once

#include "Game.h"
#include "Physics.h"
#include "Limelight.h"

#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <iostream>

class Shooter{
public:
    static double findTarget(double);
    static void feedCargo(const rev::CANSparkMax& motor);
    static void alignTarget(const frc::DifferentialDrive&);
    static void shootHigh(const frc::DifferentialDrive&, const rev::CANSparkMax&, bool);
    static void shootLow(const frc::DifferentialDrive&, const rev::CANSparkMax&, bool);
};
