#pragma once

#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <fstream>
#include "Debug.h"

class tankDrive {
    public:
        tankDrive(rev::CANSparkMax* leftSide, rev::CANSparkMax* rightSide);
        tankDrive(frc::Spark* leftSide, frc::Spark* rightSide);
        void drive(float x, float y);

    protected:
        rev::CANSparkMax* rightMaxDrive;
        rev::CANSparkMax* leftMaxDrive;
        frc::Spark* rightSparkDrive;
        frc::Spark* leftSparkDrive;


};