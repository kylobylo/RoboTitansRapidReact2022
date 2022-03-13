/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include "Limelight.h"
#include "Shooter.h"
#include "Game.h"

#include <iostream>

#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/StartEndCommand.h>
#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>

//Pin Variables
unsigned const short controlJoystickID = 0;
unsigned const short driveJoystickID = 1;

//Spark Variables
unsigned const short intakeMotorID = 0;
unsigned const short shootMotorID = 3;
unsigned const short indexMotorID = 6;

//The Lead motor is the one in front of the lagging motor.
static unsigned const short leadRightSparkID = 1;
static unsigned const short leadLeftSparkID = 2;

//Button Variables
unsigned const short shootHighButtonID = 4;
unsigned const short shootLowButtonID = 5;
unsigned const short cameraButtonID = 6;

//Define Spark Max Objects
rev::CANSparkMax m_shootMotor{shootMotorID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_indexMotor{indexMotorID, rev::CANSparkMax::MotorType::kBrushless};

rev::CANSparkMax m_leftLeadingMotor{leadLeftSparkID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightLeadingMotor{leadRightSparkID, rev::CANSparkMax::MotorType::kBrushless};

frc::DifferentialDrive m_robotDrive{m_leftLeadingMotor, m_rightLeadingMotor};

//PID controller
rev::SparkMaxPIDController m_pidController = m_shootMotor.GetPIDController();
rev::SparkMaxRelativeEncoder m_encoder = m_shootMotor.GetEncoder();

//Default PID coefficientss
double proportionalPIDConstant = 0;
double intergralPIDConstant = 0;
double derivativePIDConstant = 0;
double intergralZonePIDConstant = 0;
double feedForwardPIDConstant = 0.000015;
double kMaxOutput = 1.0;
double kMinOutput = -1.0;

//Instantiate the buttons
frc::Joystick m_controlStick{controlJoystickID};
frc::Joystick m_driveStick{driveJoystickID};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Assign the PID variables
  m_pidController.SetP(proportionalPIDConstant);
  m_pidController.SetI(intergralPIDConstant);
  m_pidController.SetD(derivativePIDConstant);
  m_pidController.SetIZone(intergralZonePIDConstant);
  m_pidController.SetFF(feedForwardPIDConstant);
  m_pidController.SetOutputRange(kMinOutput, kMaxOutput);
  
  //Update the variables to the smartdashboard
  frc::SmartDashboard::PutNumber("P Gain", proportionalPIDConstant);
  frc::SmartDashboard::PutNumber("I Gain", intergralPIDConstant);
  frc::SmartDashboard::PutNumber("D Gain", derivativePIDConstant);
  frc::SmartDashboard::PutNumber("I Zone", intergralZonePIDConstant);
  frc::SmartDashboard::PutNumber("Feed Forward", feedForwardPIDConstant);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
}

void Robot::RobotPeriodic() {
  
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    
  }
}

void Robot::TeleopInit() {}


void onShotRequest(double goal_height){
  Limelight::toggleCamera(0);
  double shot_rpm = Shooter::findTarget(goal_height);
  if (shot_rpm > 0) {

    Shooter::shoot(m_pidController, m_indexMotor, shot_rpm);
    if (m_encoder.GetVelocity() > (shot_rpm - rpmThreshold)) {
      m_indexMotor.Set(0.5);
    } else {
      m_indexMotor.Set(0);
    }
  }
}

void Robot::TeleopPeriodic() {
  // read PID coefficients from SmartDashboard
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double mx = frc::SmartDashboard::GetNumber("Max Output", 0);
  double mn = frc::SmartDashboard::GetNumber("Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != proportionalPIDConstant)) { m_pidController.SetP(p); proportionalPIDConstant = p;}
  if((i != intergralPIDConstant)) { m_pidController.SetI(i); intergralPIDConstant = i;}
  if((d != derivativePIDConstant)) { m_pidController.SetD(d); derivativePIDConstant = d;}
  if((iz != intergralZonePIDConstant)) { m_pidController.SetIZone(iz); intergralZonePIDConstant = iz;}
  if((ff != feedForwardPIDConstant)) { m_pidController.SetFF(ff); feedForwardPIDConstant = ff;}
  if((mx != kMaxOutput) || (mn != kMinOutput)) { 
    m_pidController.SetOutputRange(mn, mx);
    kMinOutput = mn;
    kMaxOutput = mx;
  }

  if (m_controlStick.GetRawButton(shootHighButtonID)){
    onShotRequest(top_goal);
  } else if (m_controlStick.GetRawButton(shootLowButtonID)){    
    onShotRequest(low_goal);
  }

  if (m_controlStick.GetRawButtonPressed(shootLowButtonID) || m_controlStick.GetRawButtonPressed(shootHighButtonID)){
    Shooter::alignTarget(m_robotDrive);
  }

  if (m_controlStick.GetRawButtonReleased(shootLowButtonID) || m_controlStick.GetRawButtonReleased(shootHighButtonID)){
    Shooter::shoot(m_pidController, m_indexMotor, 0);
    m_indexMotor.Set(0);
    Limelight::toggleCamera(1);
  }

  if (m_controlStick.GetRawButtonPressed(cameraButtonID)){
    Limelight::toggleCamera();
  }
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
