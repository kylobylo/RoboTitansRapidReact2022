/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


  //Put pin numbers as variables here.
  unsigned const short driveStickID = 0;
  unsigned const short controlStickID = 1;

  //The Lead motor is the one in front of the lagging motor.
  static unsigned const short leadRightSparkID = 1;
  static unsigned const short laggingRightSparkID = 2;
  static unsigned const short leadLeftSparkID = 3;
  static unsigned const short laggingLeftSparkID = 4;
  //Define Spark Max objects
  rev::CANSparkMax m_leftLeadingMotor{leadLeftSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftLaggingMotor{laggingLeftSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadingMotor{leadRightSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLaggingMotor{laggingRightSparkID, rev::CANSparkMax::MotorType::kBrushless};
  //Only need to pass the leading motors to differential drive because the lagging motors
  //will follow the leading motors.
  frc::DifferentialDrive m_robotDrive{m_leftLeadingMotor, m_rightLeadingMotor};
  //Fine control differential drive object is needed for other joystick
  frc::DifferentialDrive m_robotControl{m_leftLeadingMotor, m_rightLeadingMotor};
  //Instantiate the left joystick
  frc::Joystick m_driveStick{driveStickID};
  //Instantiate the control joystick
  frc::Joystick m_controlStick{controlStickID};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_leftLeadingMotor.RestoreFactoryDefaults();
  m_leftLaggingMotor.RestoreFactoryDefaults();
  m_rightLeadingMotor.RestoreFactoryDefaults();
  m_rightLaggingMotor.RestoreFactoryDefaults();


  m_leftLaggingMotor.Follow(m_leftLeadingMotor);
  m_rightLaggingMotor.Follow(m_rightLeadingMotor);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
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

void Robot::TeleopInit() {
  //Drive with arcade style
  

}

void Robot::TeleopPeriodic() {
  
  if (m_driveStick.GetY()>=0.1 ||m_driveStick.GetY()<=-0.1|| m_driveStick.GetX()>=0.1 || m_driveStick.GetX()<=-0.1) {
    m_robotDrive.ArcadeDrive(m_driveStick.GetY(), m_driveStick.GetX());
  }

  if (m_controlStick.GetY()>=0.1 || m_controlStick.GetY()<=0.1 || m_controlStick.GetX()>=0.1 || m_controlStick.GetX()<=-0.1) {
    m_robotControl.ArcadeDrive(m_controlStick.GetY(), m_controlStick.GetX());
  }
}

void Robot::TestPeriodic() {
  std::cout << m_driveStick.GetY();
  std::cout << m_driveStick.GetX();
  m_leftLeadingMotor.RestoreFactoryDefaults();
  m_leftLaggingMotor.RestoreFactoryDefaults();
  m_rightLeadingMotor.RestoreFactoryDefaults();
  m_rightLaggingMotor.RestoreFactoryDefaults();
  m_robotDrive.ArcadeDrive(m_driveStick.GetY(), m_driveStick.GetX());

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
