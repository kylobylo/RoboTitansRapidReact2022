#include "Shooter.h"

double min(double a, double b){
    if (a < b){
        return a;
    }
    return b;
};

//Finds the target.
double Shooter::findTarget(double goal_height){
    if (Limelight::visibleTarget()){
        double goal_distance = Limelight::getDistance();

        if (Physics::checkDistance(goal_distance, goal_height)){
            double shot_velocity = Physics::getVelocity(goal_distance, goal_height);

            double shot_rpm = Physics::getShotRPM(shot_velocity);

            std::cout << shot_velocity << "m/s" << std::endl;

            return shot_rpm;
        };
    };
};

//Shoots the balls at the desired rpm
void Shooter::shoot(rev::SparkMaxPIDController& pidController, rev::CANSparkMax& indexMotor, double rpm){
    double setPoint = min(motorMaxRPM, rpm);// = motorMaxRPM*m_stick.GetY();

    pidController.SetReference(setPoint, rev::ControlType::kVelocity);
};

//Aligns the robot with the goal.
void Shooter::alignTarget(frc::DifferentialDrive& robotDrive){
    if (Limelight::visibleTarget()){
    double horizontal_angle_offset = Limelight::getInfo("tz");//-29.8 to +29.8 degrees;

    robotDrive.ArcadeDrive(0, horizontal_angle_offset / 360);//---!!!Need to check if this works!!!---
  };
};

//Shoots the ball into the high goal.
void Shooter::shootHigh(frc::DifferentialDrive& robotDrive, rev::SparkMaxPIDController& pidController, rev::CANSparkMax& indexMotor, bool lookForTarget = true){
    Shooter::alignTarget(robotDrive);
    if (lookForTarget){
        double shot_rpm = Shooter::findTarget(top_goal);
        shoot(pidController, indexMotor, shot_rpm);
    } else {
        shoot(pidController, indexMotor, 0);
    };
};

//Shoots the ball into the low goal.
void Shooter::shootLow(frc::DifferentialDrive& robotDrive, rev::SparkMaxPIDController& pidController, rev::CANSparkMax& indexMotor, bool lookForTarget = false){
    Shooter::alignTarget(robotDrive);
    if (lookForTarget){
        double shot_rpm = Shooter::findTarget(low_goal);
        shoot(pidController, indexMotor, shot_rpm);
    } else {
        shoot(pidController, indexMotor, 30);
    };
};