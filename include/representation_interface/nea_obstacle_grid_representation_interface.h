/* Copyright 2014 Sankalp Arora
 * nea_obstacle_grid_representation_interface.h
 *
 *  Created on: Sep 12, 2014
 *      Author: Sankalp Arora
 */

#ifndef REPRESENTATION_INTERFACE_INCLUDE_NEA_OBSTACLE_GRID_REPRESENTATION_INTERFACE_H
#define REPRESENTATION_INTERFACE_INCLUDE_NEA_OBSTACLE_GRID_REPRESENTATION_INTERFACE_H

#include <vector>
#include <memory>
#include <utility>
#include <Eigen/Dense>
#include "perception_interface/mapping_client.h"
#include "representation_interface/obstacle_grid_representation_interface.h"

namespace ca {
namespace representation_interface{


/**
 * \brief
 * Collision check with the nea obstacle grid
 */


class NEAObstacleGridRepresentationInterface: public ObstacleGridRepresentationInterface{
 private:
  //std::shared_ptr<NEA::MappingClient> mapping_client_;
  NEA::MappingClient *mapping_client_;
 public:
  NEAObstacleGridRepresentationInterface(){};
  virtual ~NEAObstacleGridRepresentationInterface(){};

  //---------------------------------------------------------------------------------------------//
  virtual bool GetValue(const Eigen::Vector3d  &query, double &value);
   virtual bool GetValue(const Eigen::Vector3f  &query, double &value);
   virtual bool GetValue(const Eigen::Vector3i  &query, double &value);
   virtual std::vector<std::pair<double, bool> > GetValue(const std::vector<Eigen::Vector3d > &query);
   virtual std::vector<std::pair<double, bool> > GetValue(const std::vector<Eigen::Vector3i > &query);
   virtual bool IsValid(const Eigen::Vector3d  &query);
   virtual bool IsValid(const Eigen::Vector3i  &query);
   virtual Eigen::Vector3d GetMinQuery();
   virtual Eigen::Vector3d GetMaxQuery();
   virtual Eigen::Vector3d GetResolution(const Eigen::Vector3d &query);
   virtual Eigen::Vector3d GetResolution(const Eigen::Vector3i &query);

  //---------------------------------------------------------------------------------------------//


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
  CollisionCheckReturn CollisionCheck(const std::vector<Eigen::Vector3d> &query,
                                              bool greater, const double threshold);
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
                                              bool greater, const double threshold);

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
  virtual CollisionCheckReturn CollisionCheckLine(const Eigen::Vector3d &start,const Eigen::Vector3d &end,
                                             const bool greater,const double threshold);

  /**
    * \brief Returns the gradient at a vector of query locations
    * @param query vector of vectorxd
    * @return gradients in form of vector of vectorxd
    */
  virtual std::vector<std::pair<Eigen::Vector3d,bool> > GetGradient(const std::vector<Eigen::Vector3d> &query);
 /**
   * \brief Returns the gradient at a  query locations
   * @param query
   * @return gradient
   */
  virtual bool GetGradient(const Eigen::Vector3d &query, Eigen::Vector3d &value);

  /*
  void set_mapping_client_pointer(std::shared_ptr<NEA::MappingClient> mapping_client)
  {
    mapping_client_ = mapping_client;
  }*/

  void set_mapping_client_pointer(NEA::MappingClient* mapping_client)
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


#endif  // REPRESENTATION_INTERFACE_INCLUDE_NEA_OBSTACLE_GRID_REPRESENTATION_INTERFACE_H
