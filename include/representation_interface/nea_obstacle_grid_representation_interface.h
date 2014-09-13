/* Copyright 2014 Sankalp Arora
 * nea_obstacle_grid_representation_interface.h
 *
 *  Created on: Sep 12, 2014
 *      Author: Sankalp Arora
 */

#ifndef REPRESENTATION_INTERFACE_INCLUDE_NEA_OBSTACLE_GRID_REPRESENTATION_INTERFACE_H
#define REPRESENTATION_INTERFACE_INCLUDE_NEA_OBSTACLE_GRID_REPRESENTATION_INTERFACE_H

#include <vector>
#include <utility>
#include <Eigen/Dense>
#include "perception_interface/mapping_client.h"

namespace ca {
namespace representation_interface{

/*
 * type declaration for response to collision queries, the first member of
 *  the pair is vector of values encountered while collision checking, the
 *  second member is true if the query collides.
 */
typedef std::pair <std::vector<double>, bool> CollisionCheckReturn;

/**
 * \brief This is an interface class that captures how modules
 * generally interact with the a world grid representation that
 * encapsulates obstacles in one way or the other.
 * For example, occupancy gridmap, distance map
 */


class NEAObstacleGridRepresentation: public RepresentationInterface<double>{
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
  virtual CollisionCheckReturn CollisionCheck(const std::vector<Eigen::VectorXd> &query,
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
  virtual CollisionCheckReturn CollisionCheck(const std::vector<Eigen::VectorXi> &query,
                                              bool greater, const double threshold)=0;

  /**
   * \brief check a line for collisions
   * @params - start and end of a line
   *         - flag if set true, means collision is detected if a value
   *         greater than threshold is encountered, if false the check is
   *         reverted to lesser than
   *         - threshold below which or greater than which collision is detected
   *
   * @return CollisionCheckReturn, look at typedef definition for details
   */
  virtual CollisionCheckReturn CollisionCheckLine(const Eigen::VectorXd &start,const Eigen::VectorXd &end,
                                             const bool greater,const double threshold)=0;

  /**
    * \brief Returns the gradient at a vector of query locations
    * @param query vector of vectorxd
    * @return gradients in form of vector of vectorxd
    */
   virtual std::vector<Eigen::VectorXd> GetGradient(const std::vector<Eigen::VectorXd> &query)=0;
 /**
   * \brief Returns the gradient at a  query locations
   * @param query
   * @return gradient
   */
  virtual Eigen::VectorXd GetGradient(const Eigen::VectorXd &query)=0;

  NEA::MappingClient *mapping_client_;

  void set_mapping_client_pointer(NEA::MappingClient *mapping_client)
  {
    mapping_client_ = mapping_client;
  }

  std::string get_frame()
  {
    return mapping_client_->getPerceptionFrame();
  };

  bool initialize()
  {
    return mapping_client_->initialize();
  }
};

}  // namespace representation_interface
}  // namepsace ca


#endif  // REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_INTERFACE_BASE_H
