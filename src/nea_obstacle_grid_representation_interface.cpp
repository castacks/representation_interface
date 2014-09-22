/* Copyright 2014 Sankalp Arora
 * nea_obstacle_grid_representation_interface.cpp
 *
 *  Created on: Sep 12, 2014
 *      Author: Sankalp Arora
 */
#include "representation_interface/nea_obstacle_grid_representation_interface.h"
#include <boost/assert.hpp>

using namespace ca::representation_interface;


bool NEAObstacleGridRepresentationInterface::GetValue(const Eigen::Vector3d &query, double &value)
{
  int occupancy_level;
  return mapping_client_->getOccupancyLevel(query, occupancy_level);
};

bool NEAObstacleGridRepresentationInterface::GetValue(const Eigen::Vector3f &query, double &value)
{
  BOOST_ASSERT_MSG(0, "float input GetValue cannot be defined for nea occupancy grid");
  return false;
};

bool NEAObstacleGridRepresentationInterface::GetValue(const Eigen::Vector3i  &query, double &value)
{
  BOOST_ASSERT_MSG(0, "int input GetValue cannot be defined for nea occupancy grid");
  return false;
}


std::vector<std::pair<double, bool> > NEAObstacleGridRepresentationInterface::GetValue(const std::vector<Eigen::Vector3d> &query)
{
  std::vector<std::pair<double, bool> > return_vector;
  for(auto it=query.begin();it<query.end();it++)
  {
    int occupancy_level=0;
    std::pair<double,bool> return_pair;
    return_pair.second = mapping_client_->getOccupancyLevel(*it, occupancy_level);
    return_pair.first = occupancy_level;
    return_vector.push_back(return_pair);
  }
  return return_vector;
}

std::vector<std::pair<double, bool>> NEAObstacleGridRepresentationInterface::
GetValue(const std::vector<Eigen::Vector3i> &query)
{
  BOOST_ASSERT_MSG(0, "int input GetValue cannot be defined for nea occupancy grid");
  std::vector<std::pair<double, bool>> return_vector;
  return return_vector;
}

bool NEAObstacleGridRepresentationInterface::IsValid(const Eigen::Vector3d  &query)
{
 // Hugh this needs a maximum and minimum limit
  BOOST_ASSERT_MSG(0, "Isvalid with Vector3d is not implemented for NEAObstacleGridRepresentationInterface");
  return false;
}

bool NEAObstacleGridRepresentationInterface::IsValid(const Eigen::Vector3i  &query)
{
 // Hugh this needs a maximum and minimum limit
  BOOST_ASSERT_MSG(0, "Isvalid with Vector3i is not implemented for NEAObstacleGridRepresentationInterface");
  return false;
}

Eigen::Vector3d NEAObstacleGridRepresentationInterface::GetMinQuery()
{
  // Hugh can we get this?
  BOOST_ASSERT_MSG(0, "GetMinQuery() is not implemented for NEAObstacleGridRepresentationInterface");
  Eigen::Vector3d v;
  return v;
}


Eigen::Vector3d NEAObstacleGridRepresentationInterface::GetMaxQuery()
{
  // Hugh can we get this?
  BOOST_ASSERT_MSG(0, "GetMaxQuery() is not implemented for NEAObstacleGridRepresentationInterface");
  Eigen::Vector3d v;
  return v;
}

Eigen::Vector3d NEAObstacleGridRepresentationInterface::GetResolution(const Eigen::Vector3d &query)
{
  // Hugh can we get this?
    BOOST_ASSERT_MSG(0, "GetResolution() is not implemented for NEAObstacleGridRepresentationInterface");
    Eigen::Vector3d v;
    return v;
}

Eigen::Vector3d NEAObstacleGridRepresentationInterface::GetResolution(const Eigen::Vector3i &query)
{
  // Hugh can we get this?
    BOOST_ASSERT_MSG(0, "GetResolution() is not implemented for NEAObstacleGridRepresentationInterface");
    Eigen::Vector3d v;
    return v;
}


CollisionCheckReturn NEAObstacleGridRepresentationInterface::CollisionCheck(const std::vector<Eigen::Vector3d> &query,
                                            bool greater, const double threshold)
{
  CollisionCheckReturn collision_check;
  if(greater)
  {
    for(auto it=query.begin();it<query.end();it++)
    {
      int occupancy_level=0;
      if(mapping_client_->getOccupancyLevel(*it, occupancy_level))
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
      if(mapping_client_->getOccupancyLevel(*it, occupancy_level))
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

CollisionCheckReturn NEAObstacleGridRepresentationInterface::CollisionCheck(const std::vector<Eigen::Vector3i> &query,
                                              bool greater, const double threshold)
{
  BOOST_ASSERT_MSG(0, "CollisionCheck with INT queries is not defined for nea occupancy grid");
}

CollisionCheckReturn NEAObstacleGridRepresentationInterface::CollisionCheckLine(const Eigen::Vector3d &start,const Eigen::Vector3d &end,
                                             const bool greater,const double threshold)
{
 // hugh -- getOccupancyResolution(); does it give us the spatial resolution?
 // hugh -- class functors
  /*
   * bool getOccupancyLevel(const Vector3D &start_pose,
                           const std::vector<Vector3I> &cell_offsets,
                           std::vector<int> &occupancy_levels);
     Do the offsets mean the next cell when the resolution changes or the resolution stays constant and next cell is decided accordingly ?
   */
}


std::vector<std::pair<Eigen::Vector3d,bool> > NEAObstacleGridRepresentationInterface::GetGradient(const std::vector<Eigen::Vector3d> &query)
{
  BOOST_ASSERT_MSG(0, "Gradient is not defined for nea occupancy grid");
}


bool NEAObstacleGridRepresentationInterface::GetGradient(const Eigen::Vector3d &query, Eigen::Vector3d &value)
{
  BOOST_ASSERT_MSG(0, "Gradient is not defined for nea occupancy grid");
}

