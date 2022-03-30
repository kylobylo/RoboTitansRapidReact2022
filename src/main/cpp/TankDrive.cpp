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
#include "TankDrive.h"


tankDrive::tankDrive(rev::CANSparkMax* leftSide, rev::CANSparkMax* rightSide) {
    rightMaxDrive = rightSide;
    leftMaxDrive = leftSide;
}

tankDrive::tankDrive(frc::Spark* leftSide, frc::Spark* rightSide) {
    rightSparkDrive = rightSide;
    leftSparkDrive = leftSide;
}


void tankDrive::drive(float x, float y) {
    //Needs to add x and y together to turn and drive
    if (x < 0.1 && x > -0.1) {
        x = 0;
    }
    if (y < 0.1 && y > -0.1) {
        y = 0;
    }
    float leftDrive = y + x;
    float rightDrive = y - x;
    if (leftDrive > 1) {
        leftDrive = 1;
    } else if (leftDrive < -1) {
        leftDrive = -1;
    }
    if (rightDrive > 1) {
        rightDrive = 1;
    } else if (rightDrive < -1) {
        rightDrive = -1;
    }

    rightMaxDrive->Set(rightDrive);
    leftMaxDrive->Set(leftDrive);

}