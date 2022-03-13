#include "Robot.h"
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include "Climb.h"
#include <fstream>

void climb::startClimb() {

}

bool climb::checkIfLine() {
    climb climbing;
    if (climb::input.Get()) {
        climbing.startClimb();
    }
}