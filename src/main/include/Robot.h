/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>


class Robot : public frc::TimedRobot {
 public:


  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledPeriodic() override;

  //Put pin numbers as variables here.
  unsigned const short driveStickID = 0;
  unsigned const short controlStickID = 1;
  unsigned const short controlTriggerID = 0;
  unsigned const short sideButtonID = 2;
  unsigned const short modeButtonID = 7;
  //The Lead motor is the one in front of the lagging motor.
  static unsigned const short leadRightSparkID = 1;
  static unsigned const short leadLeftSparkID = 2;
  static unsigned const short shooterMotorID = 3;
  static unsigned const short intakeMotorID = 6;
  static unsigned const short indexBeltID = 4;
  //Define Spark and Spark Max objects
  rev::CANSparkMax m_leftLeadingMotor{leadLeftSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadingMotor{leadRightSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_shooterMotor{shooterMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intakeMotor{intakeMotorID, rev::CANSparkMax::MotorType::kBrushless};
  frc::Spark m_indexBelt{0};
  //Only need to pass the leading motors to differential drive because the lagging motors
  //will follow the leading motors.
  //frc::DifferentialDrive m_robotDrive{m_leftLeadingMotor, m_rightLeadingMotor};
  //Fine control differential drive object is needed for other joystick
  frc::DifferentialDrive m_robotControl{m_leftLeadingMotor, m_rightLeadingMotor};
  //Instantiate the left joystick
  frc::Joystick m_driveStick{driveStickID};
  //Instantiate the control joystick
  frc::Joystick m_controlStick{controlStickID};
  //Instantiate shooter speed variable for manipulation in Robot Periodic
  float shooterSpeed = 1.0;
  //create a Digital Input object to get grip detection
  frc::DigitalInput m_gripButton1{0};
  frc::DigitalInput m_gripButton2{1};
  //Create a mode toggle for manual climb control and a mode toggle for intake direction
  bool climbing;
  bool intake;

  //Instantiate a Smart dashboard object for displaying data
  frc::SmartDashboard display;
  //Make a robot object

  dbg debug;

 private:

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
