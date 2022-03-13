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
#include <fstream>

class climb {
    public: 
    frc::DigitalInput input{0};
    bool checkIfLine();

    private:
    
    void startClimb();
};