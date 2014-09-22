/* Copyright 2014 Sankalp Arora
 * nea_obstacle_grid_representation_interface.cpp
 *
 *  Created on: Sep 12, 2014
 *      Author: Sankalp Arora
 */
#include "representation_interface/nea_distance_map_representation_interface.h"
#include <boost/assert.hpp>

using namespace ca::representation_interface;


bool NEADistanceMapRepresentationInterface::GetValue(const Eigen::Vector3d &query, double &value)
{
  NEA::MappingClient::NearestObstacle distance;
  bool return_bool = mapping_client_->getObstacleDistance(query, distance);
  value = distance.distance;
  return return_bool;
};

bool NEADistanceMapRepresentationInterface::GetValue(const Eigen::Vector3f &query, double &value)
{
  BOOST_ASSERT_MSG(0, "float input GetValue cannot be defined for nea occupancy grid");
  return false;
};

bool NEADistanceMapRepresentationInterface::GetValue(const Eigen::Vector3i  &query, double &value)
{
  BOOST_ASSERT_MSG(0, "int input GetValue cannot be defined for nea occupancy grid");
  return false;
}


std::vector<std::pair<double, bool> > NEADistanceMapRepresentationInterface::GetValue(const std::vector<Eigen::Vector3d> &query)
{
  std::vector<std::pair<double, bool> > return_vector;
  NEA::MappingClient::NearestObstacle distance;
  std::pair<double,bool> return_pair;
  for(auto it=query.begin();it<query.end();it++)
  {
    return_pair.second = mapping_client_->getObstacleDistance(*it, distance);
    return_pair.first = distance.distance;
    return_vector.push_back(return_pair);
  }
  return return_vector;
}

std::vector<std::pair<double, bool>> NEADistanceMapRepresentationInterface::
GetValue(const std::vector<Eigen::Vector3i> &query)
{
  BOOST_ASSERT_MSG(0, "int input GetValue cannot be defined for nea occupancy grid");
  std::vector<std::pair<double, bool>> return_vector;
  return return_vector;
}

bool NEADistanceMapRepresentationInterface::IsValid(const Eigen::Vector3d  &query)
{
 // Hugh this needs a maximum and minimum limit
  BOOST_ASSERT_MSG(0, "Isvalid with Vector3d is not implemented for NEAObstacleGridRepresentationInterface");
  return false;
}

bool NEADistanceMapRepresentationInterface::IsValid(const Eigen::Vector3i  &query)
{
 // Hugh this needs a maximum and minimum limit
  BOOST_ASSERT_MSG(0, "Isvalid with Vector3i is not implemented for NEAObstacleGridRepresentationInterface");
  return false;
}

Eigen::Vector3d NEADistanceMapRepresentationInterface::GetMinQuery()
{
  // Hugh can we get this?
  BOOST_ASSERT_MSG(0, "GetMinQuery() is not implemented for NEAObstacleGridRepresentationInterface");
  Eigen::Vector3d v;
  return v;
}


Eigen::Vector3d NEADistanceMapRepresentationInterface::GetMaxQuery()
{
  // Hugh can we get this?
  BOOST_ASSERT_MSG(0, "GetMaxQuery() is not implemented for NEAObstacleGridRepresentationInterface");
  Eigen::Vector3d v;
  return v;
}

Eigen::Vector3d NEADistanceMapRepresentationInterface::GetResolution(const Eigen::Vector3d &query)
{
  // Hugh can we get this?
    BOOST_ASSERT_MSG(0, "GetResolution() is not implemented for NEAObstacleGridRepresentationInterface");
    Eigen::Vector3d v;
    return v;
}

Eigen::Vector3d NEADistanceMapRepresentationInterface::GetResolution(const Eigen::Vector3i &query)
{
  // Hugh can we get this?
    BOOST_ASSERT_MSG(0, "GetResolution() is not implemented for NEAObstacleGridRepresentationInterface");
    Eigen::Vector3d v;
    return v;
}


CollisionCheckReturn NEADistanceMapRepresentationInterface::CollisionCheck(const std::vector<Eigen::Vector3d> &query,
                                            bool greater, const double threshold)
{
  CollisionCheckReturn collision_check;
  if(greater)
  {
    NEA::MappingClient::NearestObstacle distance;
    for(auto it=query.begin();it<query.end();it++)
    {
      if(mapping_client_->getObstacleDistance(*it, distance))
      {
        collision_check.first.push_back(distance.distance);
        if(distance.distance>=threshold)
        {
          collision_check.second = true;
          break;
        }
      }
      else
      {
        collision_check.second = false;
        break;
      }
    }
  }
  else
  {
    NEA::MappingClient::NearestObstacle distance;
    for(auto it=query.begin();it<query.end();it++)
    {
      int occupancy_level=0;
      if(mapping_client_->getObstacleDistance(*it, distance))
      {
        collision_check.first.push_back(distance.distance);
        if(distance.distance<=threshold)
        {
          collision_check.second = true;
          break;
        }
      }
      else
      {
        collision_check.second = false;
        break;
      }
    }
  }
}

CollisionCheckReturn NEADistanceMapRepresentationInterface::CollisionCheck(const std::vector<Eigen::Vector3i> &query,
                                              bool greater, const double threshold)
{
  BOOST_ASSERT_MSG(0, "CollisionCheck with INT queries is not defined for nea occupancy grid");
}

CollisionCheckReturn NEADistanceMapRepresentationInterface::CollisionCheckLine(const Eigen::Vector3d &start,const Eigen::Vector3d &end,
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


std::vector<std::pair<Eigen::Vector3d,bool> > NEADistanceMapRepresentationInterface::GetGradient(const std::vector<Eigen::Vector3d> &query)
{
  std::vector<std::pair<Eigen::Vector3d,bool> > return_vector;
  NEA::MappingClient::NearestObstacle distance;
  std::pair<Eigen::Vector3d,bool> return_pair;

  for(auto it=query.begin();it<query.end();it++)
  {
    return_pair.second = mapping_client_->getObstacleDistance(*it, distance);
    return_pair.first = distance.vec2obstacle;
    return_vector.push_back(return_pair);
  }
  return return_vector;
}


bool NEADistanceMapRepresentationInterface::GetGradient(const Eigen::Vector3d &query, Eigen::Vector3d &value)
{
  NEA::MappingClient::NearestObstacle distance;
  bool return_bool = mapping_client_->getObstacleDistance(query, distance);
  value = distance.vec2obstacle;
  return return_bool;
}

