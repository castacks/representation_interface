/* Copyright 2014 Sankalp Arora
 * cost_map.h
 *
 *  Created on: Sep 3, 2014
 *      Author: Sankalp Arora
 */

#ifndef REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_INTERFACE_BASE_H
#define REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_INTERFACE_BASE_H

#include <Eigen/Dense>
#include <vector>
namespace ca {
namespace representation_interface{

/**
 * \brief This is an interface class that captures how modules
 * generally interact with the a world representation. The idea
 * is that all the future interactions with a world representation
 * should happen through this basic interface, hence allowing for quick
 * swapping of world representations.
 * For example, we can use this class as an interface for cmu occupancy grid,
 * if one wants to switch to using nea grid, we wrap it in this interface and do not change
 * any of code that uses this interface.
 */
template <class T>
class ObstacleGridRepresentationInterface: public RepresentationInterface<double>{
 public:
  ObstacleGridRepresentationInterface() {};
  virtual ~ObstacleGridRepresentationInterface() {};

  /**
   * \brief Returns the resolution at which the grid locally operates at that location
   * @param query
   * @return resolution
   */
  virtual std::vector<double> CollisionCheck(const std::vector<Eigen::VectorXd> &query, double threshold)=0;
  /**
   * \brief Returns the resolution at which the grid locally operates at that index
   * @param query
   * @return resolution
   */
  virtual std::vector<double> CollisionCheckLine(const Eigen::VectorXd &query,  double threshold)=0;
  /**
    * \brief Returns the resolution at which the grid locally operates at that index
    * @param query
    * @return resolution
    */
   virtual std::vector<Eigen::VectorXd> GetGradient(const std::vector<Eigen::VectorXd> &query,  double threshold)=0;
   /**
     * \brief Returns the resolution at which the grid locally operates at that index
     * @param query
     * @return resolution
     */
  virtual Eigen::VectorXd GetGradient(const Eigen::VectorXd &query,  double threshold)=0;
};

}  // namepsace ca
}  // namespace representation_interface


#endif  // REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_INTERFACE_BASE_H
