#include "Limelight.h"
#include <iostream>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

auto network_instance = nt::NetworkTableInstance::GetDefault();
auto variable_table = network_instance.GetTable("Preferences");
auto cam_table = network_instance.GetTable("limelight");

void Limelight::toggleCamera(){//Turns on/off the limelight and camera.
    int current_cam = cam_table->GetNumber("camMode", 0); // 0 tracking, 1 driving vision.
    int current_led = cam_table->GetNumber("ledMode", 0); // 1 off, 3 on.

    cam_table->PutNumber("camMode", (current_cam + 1) % 2);

    if (current_led == 3){
        cam_table->PutNumber("ledMode", 1);
    } else {
        cam_table->PutNumber("ledMode", 3);
    }
};

void Limelight::toggleCamera(int mode){//0 is automatic, 1 is driver.
    if (mode == 0) {
        cam_table->PutNumber("ledMode", 3);
        cam_table->PutNumber("camMode", 0);
    } else {
        cam_table->PutNumber("ledMode", 1);
        cam_table->PutNumber("camMode", 1);
    }
};

bool Limelight::visibleTarget(){
    int target_visible = cam_table->GetNumber("tv", 0.0);//1 and 0

    if (target_visible == 0){
        std::cout << "Target Not Visible!" << std::endl;
    };

    return target_visible;
};

double Limelight::getDistance(){//Calculates the horizontal distance with the limelight variables.
    double vertical_angle_offset = cam_table->GetNumber("ty", 0.0);//-24.85 to +24.85 degrees

    double height = top_goal - cam_height;//The difference of height between the goal and the camera.(The leg of the right triangle.)
    double angle = cam_angle + (vertical_angle_offset * (PI/180));//Total angle of the mount and camera in degrees.(The angle of the right triangle.)

    double distance = height/tan(angle) + offset_distance;//Horizontal distance from the goal to the shooter

    return distance;
};