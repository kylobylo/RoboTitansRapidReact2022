/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <fstream>
#include <cscore_oo.h>
#include "Debug.h"
#include "Climb.h"
#include "TankDrive.h"

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
  //Define Spark and Spark Max objects
  rev::CANSparkMax m_leftLeadingMotor{leadLeftSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadingMotor{leadRightSparkID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_shooterMotor{shooterMotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intakeMotor{intakeMotorID, rev::CANSparkMax::MotorType::kBrushless};
  frc::Spark m_indexBelt{1};
  //will follow the leading motors.
  frc::DifferentialDrive m_robotDrive{m_leftLeadingMotor, m_rightLeadingMotor};
  //Fine control differential drive object is needed for other joystick
  tankDrive m_robotControl(&m_leftLeadingMotor, &m_rightLeadingMotor);
  //Instantiate the left joystick
  frc::Joystick m_climbStick{driveStickID};
  //Instantiate the control joystick
  frc::Joystick m_controlStick{controlStickID};
  //Instantiate shooter speed variable for manipulation in Robot Periodic
  float shooterSpeed = 1.0;
  //create a Digital Input object to get grip detection
  frc::DigitalInput testInput{6};

  //create compressor
  frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};

  //Create a mode toggle for manual climb control and a mode toggle for intake direction
  bool buttonEightPressed = false;
  bool testBool = false;
  bool intake;
  bool climbing;

  climb climbObject;

  //Previous accelerometer values
  double prevXAccel = 0;
  double prevYAccel = 0;

  //frc::SendableChooser testChoices;




void Robot::RobotInit() {

  frc::CameraServer::StartAutomaticCapture();
  
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);m_robotDrive.ArcadeDrive(m_controlStick.GetTwist(), -1 * m_controlStick.GetY());
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


  //Custom debug class records debug log in /home/lvuser/DEBUG.txt
  //initialize and end are only run once and debug.out can be run as many times as you want.
  //debug.initialize("/home/lvuser/DEBUG.txt");
  //debug.out("test");
  //debug.end();
  dbg debug;

  //set climbing to false so that we can shoot
  climbing = false;
  intake = true;

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

  cs::CvSink cvSink = frc::CameraServer::GetVideo();

  cs::CvSource outputStream = frc::CameraServer::PutVideo("Bottom_Of_Bot", 480, 480);
}

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
    m_robotControl.drive(-0.1, 0);
  } else {
    m_robotControl.drive(-0.1, 0);
  }
}

void Robot::TeleopInit() {
  //Shooter speed is set so we don't start with a spinning motor.
  shooterSpeed = 0.0;
  frc::SmartDashboard::PutNumber("Shooter Speed", shooterSpeed);
}

void Robot::DisabledPeriodic() {

}

void Robot::TeleopPeriodic() {

  //climber.prepareClimb(climbing);

//NOT-YET-TESTED tankdrive class.  Easily replacible using frc::arcadedrive(might not be exaxt function call).
  if (m_controlStick.GetY()>=0.1 || m_controlStick.GetY()<=-0.1 || m_controlStick.GetTwist()>=0.1 || m_controlStick.GetTwist()<=-0.1) {
    m_robotDrive.ArcadeDrive(m_controlStick.GetTwist(),-1 * m_controlStick.GetY());
  } 
//Toggles intake to reject balls
  if (m_controlStick.GetRawButtonReleased(6) && intake == true) {
    intake = false;
  }
  if (m_controlStick.GetRawButtonReleased(6) && intake == false) {
    intake = true;
  }
  if (intake == true) {
    m_intakeMotor.Set(1);
  }

  //Sets shooter speed with (hopefully) working limits and output to smart dashboard.
  if (m_climbStick.GetRawButtonPressed(3) && shooterSpeed != 0.0) {
    shooterSpeed = shooterSpeed - 0.1; 
    frc::SmartDashboard::PutNumber("Shooter Speed", shooterSpeed);
  }
  if (m_climbStick.GetRawButtonPressed(4) && shooterSpeed != 1.0) {
    shooterSpeed = shooterSpeed + 0.1;
    frc::SmartDashboard::PutNumber("Shooter Speed", shooterSpeed);
  }
  m_shooterMotor.Set(shooterSpeed);
  if (m_controlStick.GetRawButton(4)) {
    m_indexBelt.Set(1.0);
  } else {
    m_indexBelt.Set(0.0);
  }
}


void Robot::TestPeriodic() {
  climbObject.climbTest(&m_climbStick);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
