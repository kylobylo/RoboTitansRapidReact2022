#pragma once

#include "Game.h"
#include "Physics.h"
#include "Limelight.h"

#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>
#include <iostream>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/Joystick.h"
#include <chrono>
#include <thread>

#define motorMaxRPM 2000 //Motor max RPM (The fastest the motor can spin)
#define rpmThreshold 200

class Shooter{
public:
    static double findTarget(double);
    static void shoot(rev::SparkMaxPIDController&, rev::CANSparkMax&, double);
    static void feedCargo(frc::Joystick&, rev::CANSparkMax&, int);
    static void alignTarget(frc::DifferentialDrive&);
    static void shootHigh(frc::DifferentialDrive&, rev::SparkMaxPIDController&, rev::CANSparkMax&, bool);
    static void shootLow(frc::DifferentialDrive&, rev::SparkMaxPIDController&, rev::CANSparkMax&, bool);
    static void updateDashboard();
};
