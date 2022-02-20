/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "Limelight.h"
#include "Game.h"
#include "Physics.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

void alignRobotGoal(){ //Aligns the robot with the goal/target.
  if (Limelight::visibleTarget()){
    double horizontal_offset = Limelight::getInfo("tz");//-29.8 to +29.8 degrees;

    //TODO: Turn to align the robot with the goal.
  };
}

void onShotTriggered(double goal_height){ //Shoots the ball (Should be called when shot is asked).
  //TODO: Determine which goal to shoot at (High or low).
  alignRobotGoal();

  if (Limelight::visibleTarget()){
    double goal_distance = Limelight::getDistance();

    if (goal_distance > Physics::getMinDistance(goal_height)){
      double shot_velocity = Physics::getVelocity(goal_distance, goal_height);

      double shot_rpm = Physics::getShotRPM(shot_velocity);
      //TODO: Spin the shooter to this speed.

    } else {
      std::cout << "Robot is too close!" << std::endl;
    };
    
  };
};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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
  // std::cout << Physics::getVelocity(100, top_goal) << std::endl;

  onShotTriggered(300);
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
  // m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
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

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
