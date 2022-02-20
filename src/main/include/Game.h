#pragma once

#define offset_height 0.00//The vertical offset of the goal(cm)
#define offset_distance 0.00//The horizontal offset of the goal(cm)

#define shot_height 0.00//Height of the shot origin from the floor(cm)
#define cam_height 0.00//Height of camera from the floor(cm)

#define cam_distance 0.00//Distance from the back of the bot to the camera(cm)
#define shot_distance 0.00//Distance from the back of the bot to the shot exit(cm)

#define top_goal 264.00/*Height of the UPPER goal from the floor(cm)*/  + offset_height + (shot_height-cam_height)
#define low_goal 104.00/*Height of the LOWER goal from the floor(cm)*/  + offset_height + (cam_distance-shot_distance)

#define cam_angle 0.00/*Angle of camera(degrees)*/ * (PI/180)
#define shot_angle 60.00 /*Angle of the shooter(degrees)*/ * (PI/180)

#define ball_diameter 24.00//Diameter of the ball(cm)
#define wheel_diameter 15.24//Diameter of the shooting wheel(cm)

#define gravity 980.7//Gravity(cm/s/s)
#define PI 3.141592653589793//PI
