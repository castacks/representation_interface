/* Copyright 2014 Sankalp Arora
 * nea_obstacle_grid_representation_interface.h
 *
 *  Created on: Sep 12, 2014
 *      Author: Sankalp Arora
 */
#include "representation_interface/nea_obstacle_grid_representation.h"
#include <boost/assert.hpp>

using namespace ca::representation_interface

CollisionCheckReturn NEAObstacleGridRepresentation::CollisionCheck(const std::vector<Eigen::VectorXd> &query,
                                            bool greater, const double threshold)
{
  CollisionCheckReturn collision_check;
  if(greater)
  {
    for(auto it=query.begin();it<query.end();it++)
    {
      int occupancy_level=0;
      if(getOccupancyLevel(*it, occupancy_level))
      {
        collision_check.first.push_back(occupancy_level);
        if(occupancy_level>=threshold)
        {
          collision_check.second = true;
          break;
        }
      }
      else
      {
        collision_check.second = true;
        break;
      }
    }
  }
  else
  {
    for(auto it=query.begin();it<query.end();it++)
    {
      int occupancy_level=0;
      if(getOccupancyLevel(*it, occupancy_level))
      {
        collision_check.first.push_back(occupancy_level);
        if(occupancy_level<=threshold)
        {
          collision_check.second = true;
          break;
        }
      }
      else
      {
        collision_check.second = true;
        break;
      }
    }
  }
}

CollisionCheckReturn CollisionCheck(const std::vector<Eigen::VectorXi> &query,
                                              bool greater, const double threshold)
{
  BOOST_ASSERT_MSG(0, "CollisionCheck with INT queries is not defined for nea occupancy grid");
}
CollisionCheckReturn CollisionCheckLine(const Eigen::VectorXd &start,const Eigen::VectorXd &end,
                                             const bool greater,const double threshold)
{

}


std::vector<Eigen::VectorXd> GetGradient(const std::vector<Eigen::VectorXd> &query)
{
  BOOST_ASSERT_MSG(0, "Gradient is not defined for nea occupancy grid");
}
Eigen::VectorXd GetGradient(const Eigen::VectorXd &query)
{
  BOOST_ASSERT_MSG(0, "Gradient is not defined for nea occupancy grid");
}







#endif  // REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_INTERFACE_BASE_H
