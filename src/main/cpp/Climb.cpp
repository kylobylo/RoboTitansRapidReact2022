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
#include <frc/DigitalInput.h>
#include "Climb.h"
#include <fstream>
#include "Debug.h"


void climb::prepareClimb(bool* climbingBool) {

    m_grab1.Set(true);
    m_grab2.Set(true);
    m_armRelease.Set(false);
    m_armRetract.Set(false);

    frc::SmartDashboard::PutBoolean("Arm One Is Grabbed", m_grab1.Get());
    frc::SmartDashboard::PutBoolean("Arm Two Is Grabbed", m_grab2.Get());

    //MUST come at the end
    m_armRelease.Toggle();
    *climbingBool = true;
}

//Must be used in TeleopPeriodic()


//Need to make driver for lamprey absulute encoder
void climb::doClimb(frc::Joystick* joy) {
    //False == grab1 while true == grab2
    if (joy->GetRawButtonPressed(3)) {
        m_armRetract.Toggle();
    }
    frc::SmartDashboard::PutBoolean("Arm One Is Grabbed", m_grab1.Get());
    frc::SmartDashboard::PutBoolean("Arm Two Is Grabbed", m_grab2.Get());
    if (grabs < 3) {
        if (m_grab1.Get() || m_grab2.Get() && !joy->GetRawButtonPressed(6)) {
            m_climbingMotor.Set(0.5);
        }
        if (handOne.Get()) {
            m_grab1.Set(true);
        }
        if (handTwo.Get()) {
            m_grab2.Set(true);
        }
        if (joy->GetRawButtonPressed(5) && m_grab2.Get() && m_grab1.Get()) {
            if(whichSwitch) {
                m_grab2.Set(false);
                whichSwitch = false;
                grabs++;
            } else {
                m_grab1.Set(false);
                whichSwitch = true;
                grabs++;
            }
        }
    } else {
        //Use encoder to get perpendicular to the ground.
    }
}
void climb::doManualClimb(frc::Joystick* joy) {
    if (joy->GetRawButtonPressed(3)) {
        m_armRelease.Toggle();
    }
    if (joy->GetRawButtonPressed(4) && armGo == false) {
        armGo = true;
        m_climbingMotor.Set(0.1);
    }
    if (joy->GetRawButtonPressed(4) && armGo == true) {
        armGo = false;
        m_climbingMotor.Set(0.0);
    }
    if (joy->GetRawButtonPressed(5)) {
        m_grab1.Toggle();
    }
    if (joy->GetRawButtonPressed(6)) {
        m_grab2.Toggle();
    }

    frc::SmartDashboard::PutBoolean("Arm One Is Grabbed", !m_grab1.Get());
    frc::SmartDashboard::PutBoolean("Arm Two Is Grabbed", !m_grab2.Get());

}
void climb::climbTest(frc::Joystick* joy) {
    if(joy->GetRawButton(3)){
        m_climbingMotor.Set(0.1);
    }
    if(joy->GetRawButton(4)) {
        m_armRelease.Toggle();
    }
    if(joy->GetRawButtonPressed(5)){
        m_grab1.Toggle();
    }
    if(joy->GetRawButtonPressed(6)) {
        m_grab2.Toggle();
    }
}