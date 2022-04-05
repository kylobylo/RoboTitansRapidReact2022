#include <iostream>

#include "Physics.h"
#include "math.h"

bool Physics::checkDistance(double distance, double height){//Returns if the robot is too close to make the shot by using height distance and height.
    bool far_enough = distance > Physics::getMinDistance(height);

    if (!far_enough) {
        std::cout << "Robot is too close!" << std::endl;
        frc::SmartDashboard::PutNumber("Far Enough", 0);
    } else {
        frc::SmartDashboard::PutNumber("Far Enough", 1);
    };

    return far_enough;
};

double Physics::getVelocity(double distance, double height){//Returns the shot velocity by taking in distance and goal height.
    height += cam_height - exit_height + offset_height;
    distance += exit_distance - cam_distance + offset_distance;

    double velocity = sqrt(-(gravity * pow(distance, 2) / (2 * (height - distance * tan(exit_angle)) * pow(cos(exit_angle),2) )));

    return velocity;
};

double Physics::getMinDistance(double height){//Returns minimum distance that you can be from the goal to make a successful shot using the goal height.
    double min_distance = height/tan(exit_angle) * 2;

    return min_distance;
};

double Physics::getHeight(double velocity){//Returns the highest point of the shot with the velocity.
    double height = pow(velocity * sin(exit_angle), 2) / (gravity * 2);

    height += exit_height - cam_height;

    return height;
};

double Physics::getTime(double max_height, double goal_height, double velocity){//Returns the time taken to make the shot using the max height, goal height, and velocity.
    double drop_time = sqrt((max_height - goal_height) / gravity);
    double climb_time = velocity * sin(exit_angle) / gravity;

    double total_time = drop_time + climb_time;

    return total_time;
};

double Physics::getShotRPM(double velocity){//Returns the needed rpm of the shooter needed to reach the desired ball velocity.
    double radians_per_second = (2 * velocity) / (wheel_diameter/2);
    double rotaions_per_minute = radians_per_second * (120 / (2 * PI));

    return rotaions_per_minute;
};