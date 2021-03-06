/** * @author: AirLab / Field Robotics Center
 *
 * @attention Copyright (C) 2016
 * @attention Carnegie Mellon University
 * @attention All rights reserved
 *
 * @attention LIMITED RIGHTS:
 * @attention The US Government is granted Limited Rights to this Data.
 *            Use, duplication, or disclosure is subject to the
 *            restrictions as stated in Agreement AFS12-1642.
 */
/* Copyright 2014 Sankalp Arora
 * cost_map.h
 *
 *  Created on: Sep 3, 2014
 *      Author: Sankalp Arora
 */

#ifndef REPRESENTATION_INTERFACE_INCLUDE_OBSTACLE_GRID_REPRESENTATION_INTERFACE_H
#define REPRESENTATION_INTERFACE_INCLUDE_OBSTACLE_GRID_REPRESENTATION_INTERFACE_H

#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <representation_interface/representation_interface.h>

namespace ca {
namespace representation_interface{

/*
 * type declaration for response to collision queries, the first member of
 *  the pair is vector of values encountered while collision checking, the
 *  second member is true if the query collides.
 */
typedef std::pair <std::vector<double>, bool> CollisionCheckReturn;


/*
 * type declaration for response to line collision queries, the first member of
 *  the struct is a vector of checked points, second stores the values encountered 
 *	while collision checking, the third member is true if a collision occurs
 */
struct LineCollisionCheckReturn {
	std::vector<Eigen::Vector3d> points_checked;
	std::vector<double> values;
	bool collision;
};

/**
 * \brief This is an interface class that captures how modules
 * generally interact with the a world grid representation that
 * encapsulates obstacles in one way or the other.
 * For example, occupancy gridmap, distance map
 */

class ObstacleGridRepresentationInterface: public RepresentationInterface<double,3>{
 public:
  ObstacleGridRepresentationInterface() {};
  virtual ~ObstacleGridRepresentationInterface() {};
  /**
   * \brief check a vector of locations for collisions, the function starts from
   *  top of the vector and breaks on the first collision as it goes through the
   *  vector
   * @params - a vector of locations
   *         - flag if set true, means collision is detected if a value
   *         greater than threshold is encountered, if false the check is
   *         reverted to lesser than
   *         - threshold below which or greater than which collision is detected
   *
   * @return CollisionCheckReturn, look at typedef definition for details
   */
  virtual CollisionCheckReturn CollisionCheck(const std::vector<Eigen::Vector3d> &query,
                                              bool greater, const double threshold)=0;
  /**
   * \brief check a vector of indices for collisions, the function starts from
   *  top of the vector and breaks on the first collision as it goes through the
   *  vector
   * @params - a vector of indices
   *         - flag if set true, means collision is detected if a value
   *         greater than threshold is encountered, if false the check is
   *         reverted to lesser than
   *         - threshold below which or greater than which collision is detected
   *
   * @return CollisionCheckReturn, look at typedef definition for details
   */
  virtual CollisionCheckReturn CollisionCheck(const std::vector<Eigen::Vector3i> &query,
                                              bool greater, const double threshold)=0;

  /**
   * \brief check a line for collisions
   * @params - start and end of a line
   *         - flag if set true, means collision is detected if a value
   *         greater than threshold is encountered, if false the check is
   *         reverted to lesser than
   *         - threshold below which or greater than which collision is detected
   *
   * @return LineCollisionCheckReturn, look at typedef definition for details
   */
  virtual LineCollisionCheckReturn CollisionCheckLine(const Eigen::Vector3d &start,const Eigen::Vector3d &end,
                                             const bool greater,const double threshold)=0;

  /**
    * \brief Returns the gradient at a vector of query locations (NORMALIZED)
    * @param query vector of vectorxd
    * @return gradients in form of vector of vectorxd
    */
   virtual std::vector<std::pair<Eigen::Vector3d,bool> > GetGradient(const std::vector<Eigen::Vector3d> &query)=0;
 /**
   * \brief Returns the gradient at a  query locations (NORMALIZED)
   * If the gradient is meaningless it will be 0 0 0
   * @param query
   * @return valid or not?
   */
  virtual bool GetGradient(const Eigen::Vector3d &query, Eigen::Vector3d &value)=0;

/** (Sankalp should this go up to representation?)
 * \brief Gets the max value (occupancy or max distance)
 * @param query
 * @param val
 * @return valid or not?
 */
  virtual bool GetMaxValue(const Eigen::Vector3d &query, double &val) = 0;
};

}  // namespace representation_interface
}  // namepsace ca


#endif  // REPRESENTATION_INTERFACE_INCLUDE_OBSTACLE_GRID_REPRESENTATION_INTERFACE_H
