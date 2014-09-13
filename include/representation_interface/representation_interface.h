/* Copyright 2014 Sankalp Arora
 * cost_map.h
 *
 *  Created on: Sep 3, 2014
 *      Author: Sankalp Arora
 */

#ifndef REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_BASE_H
#define REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_BASE_H

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
class RepresentationInterface{
 public:
  RepresentationInterface() {};
  virtual ~RepresentationInterface() {};

  /** \brief Get value at query location.
   *
   * @param query Eigen column vector representing a general query
   * @return value at location
   */
  virtual T GetValue(const Eigen::VectorXd &query) = 0;

  /** \brief Get value at query index.
   *
   * @param query Eigen column vector representing a general query
   * @return value at location
   */
  virtual T GetValue(const Eigen::VectorXi &query) = 0;
  /** \brief Get value at query locations.
   *
   * @param query a vector of Eigen columns vector representing a general query index.
   * @return a vector of values at query locations.
   */
  virtual std::vector<T> GetValue(const std::vector<Eigen::VectorXd> &query) = 0;
  /** \brief Get value at query indices.
   *
   * @param query a vector of Eigen columns vector representing general query indices.
   * @return a vector of values at query locations.
   */
  virtual std::vector<T> GetValue(const std::vector<Eigen::VectorXi> &query) = 0;
  /**
   * \brief Is the query location valid.
   * @param query
   * @return boolean whether the query is valid or not
   */
  virtual bool IsValid(const Eigen::VectorXd &query) = 0;
  /**
    * \brief Are the query indices valid.
    * @param query
    * @return boolean whether the query is valid or not
    */
  virtual bool IsValid(const Eigen::VectorXi &query) = 0;
  /**
   * \brief Returns the minimum valid query that can be made to the representation
   * @param
   * @return minimum valid query location.
   */
  virtual Eigen::VectorXd GetMinQuery() = 0;
  /**
   * \brief Returns the maximum valid query
   * @param
   * @return maximum valid query location.
   */
  virtual Eigen::VectorXd GetMaxQuery()=0;
  /**
   * \brief Returns the minimum valid query indices
   * @param
   * @return minimum valid query location.
   */
  virtual Eigen::VectorXi GetMinQuery() = 0;
  /**
   * \brief Returns the maximum valid query indices
   * @param
   * @return maximum valid query location.
   */
  virtual Eigen::VectorXi GetMaxQuery()=0;
  /**
   * \brief Returns the resolution at which the grid locally operates at that location
   * @param query
   * @return resolution
   */
  virtual Eigen::VectorXd GetResolution(const Eigen::VectorXd &query)=0;
  /**
   * \brief Returns the resolution at which the grid locally operates at that index
   * @param query
   * @return resolution
   */
  virtual Eigen::VectorXd GetResolution(const Eigen::VectorXi &query)=0;
  /**
   * \brief returns the frame in which the representation exists,
   * @param
   * @return the frames in which the representation exists
   */
  virtual std::string get_frame()=0;

  /**
   * \brief initializes the interface class returns false if can't initialize
   * @param
   * @return true if initialization was successful, false otherwise
   */
  virtual bool initialize()=0;
};

}  // namepsace ca
}  // namespace representation_interface


#endif  // REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_INTERFACE_BASE_H
