#pragma once

#include <iostream>

#include "Game.h"
#include "Physics.h"
#include "Limelight.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>

#include <rev/CANPIDController.h>
#include <rev/CANSparkMax.h>

#define motorMaxRPM 4000 //Motor max RPM (The fastest the motor can spin)
#define rpmThreshold 100
#define driveProportion 0.2
#define minAdjustment -0.5
#define maxAdjustment 0.5
#define shotThreshold 1
#define minDrivePower 0.5

#define shotDistance 270 // in cm
#define distanceThreshold 10 // in cm
#define alignThreshold 1 //in degrees

#define alignSpeed 0.3

#define rpm_drop 1000

class Shooter{
public:
    static void shoot(rev::SparkMaxPIDController&, double);
    static double alignTarget();
};
