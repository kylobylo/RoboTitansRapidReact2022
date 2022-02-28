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
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>



//Put pin numbers as variables here.
unsigned const short driveStickID = 0;
unsigned const short controlStickID = 1;
unsigned const short controlTriggerID = 0;
unsigned const short sideButtonID = 2;
unsigned const short modeButtonID = 7;
unsigned const short ballInputDirection = 8;
//Instantiate the solenoids for pneumatic control
frc::Solenoid m_grab1{frc::PneumaticsModuleType::CTREPCM, 1};
frc::Solenoid m_grab2{frc::PneumaticsModuleType::CTREPCM, 2};
frc::Solenoid m_armRelease{frc::PneumaticsModuleType::CTREPCM, 3};
frc::Solenoid m_armRetract{frc::PneumaticsModuleType::CTREPCM, 4};
//The Lead motor is the one in front of the lagging motor.
static unsigned const short leadRightSparkID = 1;
static unsigned const short leadLeftSparkID = 2;
static unsigned const short shooterMotorID = 3;
//static unsigned const short climbingMotorID = 4;
static unsigned const short intakeMotorID = 6;
static unsigned const short indexBeltID = 4;
//Define Spark and Spark Max objects
rev::CANSparkMax m_leftLeadingMotor{leadLeftSparkID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightLeadingMotor{leadRightSparkID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_shooterMotor{shooterMotorID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_indexBelt{indexBeltID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_intakeMotor{intakeMotorID, rev::CANSparkMax::MotorType::kBrushless};
//VictorSPX m_indexBelt = {indexBeltID};
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
//Make a robot object
Robot robot;



void Robot::RobotInit() {
 
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

/*   m_shooterMotor.RestoreFactoryDefaults();
  m_climbingMotor.RestoreFactoryDefaults();
  m_intakeMotor.RestoreFactoryDefaults();
  m_leftLeadingMotor.RestoreFactoryDefaults();
  m_rightLeadingMotor.RestoreFactoryDefaults(); */

  //set climbing to false so that we can shoot
  climbing = false;
  intake = true;
  
  //m_robotControl.SetMaxOutput(0.50);

  m_grab1.Set(true);
  m_grab2.Set(true);
  m_armRelease.Set(false);
  m_armRetract.Set(false);


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
    m_robotControl.ArcadeDrive(0.0, -0.1);
  } else {
    m_robotControl.ArcadeDrive(0.0, -0.1);
  }
}

void Robot::TeleopInit() {
  //Drive with arcade style
  shooterSpeed = 0.0;

}

void Robot::TeleopPeriodic() {


  if (m_controlStick.GetY()>=0.1 || m_controlStick.GetY()<=-0.1 || m_controlStick.GetX()>=0.1 || m_controlStick.GetX()<=-0.1) {
    m_robotControl.ArcadeDrive(m_controlStick.GetTwist(), -1 * m_controlStick.GetY());
  } 
  /*if (m_controlStick.GetY()>=0.1 || m_controlStick.GetY()<=-0.1 || m_controlStick.GetTwist()>=0.1 || m_controlStick.GetTwist()<=-0.1) {
    m_robotControl.ArcadeDrive(m_controlStick.GetX(), -1 * m_controlStick.GetY());
  } */

 /* if (m_controlStick.GetRawAxis(2) >= 0.1 || m_controlStick.GetRawAxis(2) <= -0.1 && climbing == true) {
    m_climbingMotor.Set(m_controlStick.GetRawAxis(2));
  }*/

  if (m_controlStick.GetRawButtonPressed(8)) {
    intake = false;
  }
  if (m_controlStick.GetRawButtonReleased(8)) {
    intake = true;
  }
  
  if (intake == true) {
    m_intakeMotor.Set(0.4);
  }
  if (m_driveStick.GetRawButtonPressed(3) && shooterSpeed != 0) {
    shooterSpeed = shooterSpeed - 0.1; 
  }
  if (m_driveStick.GetRawButtonPressed(4) && shooterSpeed != 1) {
    shooterSpeed = shooterSpeed + 0.1;
  }

/*   if (m_controlStick.GetRawButtonPressed(modeButtonID) || climbing == true) {
    robot.startManualClimb();
  } */
  if (m_controlStick.GetRawButtonPressed(4)) {
      m_indexBelt.Set(-0.5);
    }
  if (m_controlStick.GetRawButtonReleased(4)) {
    m_indexBelt.Set(0.0);
  }
  m_shooterMotor.Set(shooterSpeed);
  

/*
  I am going to come back to this.  I wanted to get somehing working with no esxtra sensors so I shelved this part
  for the time being so as to facilitate quick coding and be able to test the robot sooner before scrimage

  if (m_controlStick.GetRawButtonPressed(controlTriggerID)) {
    //use to shoot ball.
    bool speed = true;
    while(speed == true) {
      m_shooterMotor.Set(shooterSpeed);
      if(m_controlStick.GetRawButtonReleased(controlTriggerID)) {
        speed = false;
        m_indexBelt.Set(0.1);
        
      }
    }
    */
}
//Function for beggining climb phase
/*void Robot::startClimb() {
  if (climbing == false) {
    m_armRelease.Toggle();
  }
  climbing = true;
  if(m_gripButton1.Get()) {
    m_grab1.Toggle();
    m_climbingMotor.Set(1);
  }
  if (m_gripButton2.Get()) {
    m_grab2.Toggle();
    m_climbingMotor.Set(1);
  }
}*/

//Function for beggining manual climb phase
/*void Robot::startManualClimb() {
  m_armRelease.Toggle();
  climbing = true;
  if(m_gripButton1.Get()) {
    m_grab1.Toggle();
  }
  if (m_gripButton2.Get()) {
    m_grab2.Toggle();
  }
}*/


void Robot::TestPeriodic() {
  m_indexBelt.Set(-1.0);
  m_shooterMotor.Set(1.0);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
