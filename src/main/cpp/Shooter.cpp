#include "Shooter.h"

double Shooter::findTarget(double goal_height){//Finds the target.
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

void Shooter::feedCargo(const rev::CANSparkMax& motor){//Feeds the cargo into the shooter.
    //TODO: Feed the balls in.
};

void Shooter::alignTarget(const frc::DifferentialDrive& drive){//Aligns the robot with the goal.
    if (Limelight::visibleTarget()){
    double horizontal_angle_offset = Limelight::getInfo("tz");//-29.8 to +29.8 degrees;
    //TODO: Turn to align the robot with the goal.
  };
};

void Shooter::shootHigh(const frc::DifferentialDrive& drive, const rev::CANSparkMax& motor, bool lookForTarget = true){//Shoots the ball into the high goal.
    Shooter::alignTarget(drive);
    if (lookForTarget){
        double shot_rpm = Shooter::findTarget(top_goal);
        //TODO: Spin shooter to the rpm speed.
        
    } else {
        //TODO: Spin shooter to the high goal speed.
        //TODO: Feed the balls in.
    };
};

void Shooter::shootLow(const frc::DifferentialDrive& drive, const rev::CANSparkMax& motor, bool lookForTarget = false){
    Shooter::alignTarget(drive);
    if (lookForTarget){
        double shot_rpm = Shooter::findTarget(low_goal);
        //TODO: Spin shooter to the rpm speed.
        //TODO: Feed the balls in.
    } else {
        //TODO: Spin shooter to the low goal speed.
        //TODO: Feed the balls in.
    };
};