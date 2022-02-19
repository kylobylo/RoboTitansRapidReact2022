#include <iostream>

#include "Physics.h"
#include "math.h"

double Physics::getVelocity(double distance, double goal){//Returns the shot velocity by taking in distance and goal height.
    double velocity = sqrt(-(gravity * pow(distance, 2) / (2 * (goal - distance * tan(shot_angle)) * pow(cos(shot_angle),2) )));

    return velocity;
};

double Physics::getMinDistance(double goal){//Returns minimum distance that you can be from the goal to make a successful shot using the goal height.
    double min_distance = goal/tan(shot_angle) * 2;

    return min_distance;
};

double Physics::getHeight(double velocity){//Returns the highest point of the shot with the velocity.
    double height = pow(velocity * sin(shot_angle), 2) / (gravity * 2);

    return height;
};

double Physics::getTime(double height, double goal, double velocity){//Returns the time taken to make the shot using the max height, goal height, and velocity.
    double drop_time = sqrt((height - goal) / gravity);
    double climb_time = velocity * sin(shot_angle) / gravity;

    double total_time = drop_time + climb_time;

    return total_time;
};

double Physics::getShotRPM(double velocity){//Returns the needed rpm of the shooter needed to reach the desired ball velocity.
    double radians_per_second = (2 * velocity) / (wheel_diameter/2);
    double rotaions_per_minute = radians_per_second * (60 / (2 * PI));

    return rotaions_per_minute;
};