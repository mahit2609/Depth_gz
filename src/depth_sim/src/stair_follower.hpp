#pragma once

#include <px4_msgs/msg/vehicle_local_position.hpp>

class StairFollower
{
public:
    StairFollower(double cruise_height, double max_step_height);

    void updateFromLocalPosition(const px4_msgs::msg::VehicleLocalPosition &msg);

    double computeTargetZ(double terrain_z) const;
    double getTerrainZ() const;
    bool isTerrainValid() const;

private:
    double cruise_height_;
    double max_step_height_;
    double terrain_z_;
    bool terrain_valid_;
};
