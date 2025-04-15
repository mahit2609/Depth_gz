#include<stair_follower.hpp>
#include <cmath>

StairFollower::StairFollower(double cruise_height, double max_step_height)
    : cruise_height_(cruise_height),
      max_step_height_(max_step_height),
      terrain_z_(0.0),
      terrain_valid_(false) {}

void StairFollower::updateFromLocalPosition(const px4_msgs::msg::VehicleLocalPosition &msg)
{
    if (msg.z_valid && msg.dist_bottom_valid) {
        terrain_z_ = msg.z + msg.dist_bottom;
        terrain_valid_ = true;
    } else {
        terrain_valid_ = false;
    }
}

bool StairFollower::isTerrainValid() const
{
    return terrain_valid_;
}

double StairFollower::computeTargetZ(double terrain_z) const
{
    return terrain_z + cruise_height_;  // NED: target height above terrain
}

double StairFollower::getTerrainZ() const
{
    return terrain_z_;
}
