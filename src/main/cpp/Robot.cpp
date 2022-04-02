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
#include "Climb.h"

//Default PID coefficientss
double shotRPM = 2500;//NOTE: The rpm drops by ~500 when ball goes in. (Increase shot speed by 500rpm)
double proportionalPIDConstant = 0.0003;
double intergralPIDConstant = 0.000001;
double derivativePIDConstant = 0;
double intergralZonePIDConstant = 0;
double feedForwardPIDConstant = 0;
double kMaxOutput = 1.0;
double kMinOutput = -1.0;

//Align Variables
double horizontalOffset = 0;
double turnPosition = 0; //In degrees
double leftTurnGoal = 0;
double rightTurnGoal = 0;

//Boolean Variables
bool indexActive = false;
bool intakeActive = false;
bool shootActive = false;

double autoSteps = 0;

double steeringAdjust = 0;

void Robot::onShotRequest(double goalHeight){
  Limelight::toggleCamera(0);
  if (Limelight::visibleTarget()){
    double goalDistance = Limelight::getDistance();

    steeringAdjust = Shooter::alignTarget();
    m_robotDrive.ArcadeDrive(steeringAdjust, 0);

    // double shotVelocity = Physics::getVelocity(goalDistance, goalHeight);
    // shotRPM = Physics::getShotRPM(shotVelocity);

    shotRPM = 2800;

    if (shotRPM > motorMaxRPM) {
      shotRPM = motorMaxRPM;
    };
    
    horizontalOffset = Limelight::getInfo("tx");
    
    frc::SmartDashboard::PutNumber("Steering Adjust", 0);
    std::cout << steeringAdjust << std::endl;

    if (shotRPM > 0 && Physics::farEnough(goalDistance, goalHeight) && abs(horizontalOffset) < alignThreshold) {
      frc::SmartDashboard::PutNumber("Shot RPM", shotRPM);
      std::cout << shotRPM << std::endl;

      if (abs(goalDistance - shotDistance) > distanceThreshold) {
        if (goalDistance > shotDistance) {
          m_robotDrive.ArcadeDrive(0, minDrivePower); 
        } else if (goalDistance < shotDistance) {
          m_robotDrive.ArcadeDrive(0, -minDrivePower);
        }
      }

      Shooter::shoot(m_pidController, shotRPM);
      double currentRPM = m_encoder.GetVelocity();
      bool speedReached = currentRPM > (shotRPM - rpmThreshold) && currentRPM < (shotRPM + rpmThreshold);
      if (speedReached) {
        indexActive = true;
      }
    }
  }
}

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
  frc::SmartDashboard::PutBoolean("Far Enough", false);
  frc::SmartDashboard::PutNumber("Shot RPM", shotRPM);
  frc::SmartDashboard::PutNumber("Shooter Velocity", 0);
  frc::SmartDashboard::PutNumber("P Gain", proportionalPIDConstant);
  frc::SmartDashboard::PutNumber("I Gain", intergralPIDConstant);
  frc::SmartDashboard::PutNumber("D Gain", derivativePIDConstant);
  frc::SmartDashboard::PutNumber("I Zone", intergralZonePIDConstant);
  frc::SmartDashboard::PutNumber("Feed Forward", feedForwardPIDConstant);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

  frc::SmartDashboard::PutNumber("Distance", Limelight::getDistance());

  frc::SmartDashboard::PutBoolean("Target Visible", false);
  frc::SmartDashboard::PutBoolean("Toggled Camera", false);

  frc::SmartDashboard::PutBoolean("intakeActive", intakeActive);
  frc::SmartDashboard::PutBoolean("indexActive", indexActive);
  frc::SmartDashboard::PutNumber("Steering Adjust", 0);
  frc::SmartDashboard::PutNumber("Left Motor Postion", m_leftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Motor Postion", m_rightEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Minimum Distance", Physics::getMinDistance(top_goal));
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  Limelight::toggleCamera(0);
  if (autoSteps < 20) {
    m_robotDrive.ArcadeDrive(0.2, 0);
  } else if (autoSteps < 30) {

    m_robotDrive.ArcadeDrive(-0.2, 0);
    m_intakeMotor.Set(intakeMotorSpeed);
  } else if (autoSteps > 0) {
    if (!Limelight::visibleTarget()) {
      m_robotDrive.ArcadeDrive(0, 0.05);
    } else {
      Robot::onShotRequest(top_goal);
    }
  }

  autoSteps += 1;
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  //Read PID coefficients from SmartDashboard
  double s = frc::SmartDashboard::GetNumber("Shot RPM", 0);
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double mx = frc::SmartDashboard::GetNumber("Max Output", 0);
  double mn = frc::SmartDashboard::GetNumber("Min Output", 0);


  //Update the Values
  frc::SmartDashboard::PutNumber("Distance", Limelight::getDistance());
  frc::SmartDashboard::PutBoolean("intakeActive", intakeActive);
  frc::SmartDashboard::PutBoolean("indexActive", indexActive);

  frc::SmartDashboard::PutNumber("Shooter Velocity", m_encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Left Motor Postion", m_leftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Motor Postion", m_rightEncoder.GetPosition());

  //If PID coefficients on SmartDashboard have changed, write new values to controller
  if((s != shotRPM)) { shotRPM = s;}
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

  //Shooting
  if (m_controlStick.GetRawButton(shootHighButtonID)){
    std::cout << "Shoot High" << std::endl;
    Robot::onShotRequest(top_goal);

  } else if (m_controlStick.GetRawButton(shootLowButtonID)){    
    std::cout << "Shoot Low" << std::endl;
    Robot::onShotRequest(low_goal);

  } else {
    //Control Stick Drive Robot
    if (abs(m_driveStick.GetTwist()) >= joyStickDeadzone || abs(m_driveStick.GetY()) >= joyStickDeadzone) {
      m_robotDrive.ArcadeDrive(m_driveStick.GetTwist(), -m_driveStick.GetY());
    } else if (abs(m_controlStick.GetTwist()) >= joyStickDeadzone || abs(m_controlStick.GetY()) >= joyStickDeadzone) {
      m_robotDrive.ArcadeDrive(m_controlStick.GetTwist()*controlStickSensitivty, -m_controlStick.GetY()*controlStickSensitivty);
    }

    indexActive = false;
    shootActive = false;
    Shooter::shoot(m_pidController, 0);
    // Limelight::toggleCamera(1);
  }
  

  //Limelight
  if (m_controlStick.GetRawButtonPressed(cameraButtonID)){
    Limelight::toggleCamera();
  }

  //Index
  if (m_controlStick.GetRawButtonPressed(indexButtonID)) {
    indexActive = !indexActive;
  }

  if (indexActive) {
    m_indexMotor.Set(indexMotorSpeed);
  } else {
    m_indexMotor.Set(0.0);
  }

  //Intake
  if (m_controlStick.GetRawButtonPressed(intakeButtonID)) {
    intakeActive = !intakeActive;
  }

  if (intakeActive) {
    m_intakeMotor.Set(intakeMotorSpeed);
  } else {
    m_intakeMotor.Set(0.0);
  }

  double ve = Physics::getShotRPM(Physics::getVelocity(300, top_goal));

  // std::cout << ve << std::endl;
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
