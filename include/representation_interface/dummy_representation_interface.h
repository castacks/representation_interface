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
class DummyRepresentationInterface: public RepresentationInterface<double,3>{
    std::string frame;
 public:
  DummyRepresentationInterface(){}
  ~DummyRepresentationInterface(){}

  /**
   * \brief initializes the interface class returns false if can't initialize
   * @param
   * @return true if initialization was successful, false otherwise
   */
  virtual bool Initialize(){return true;}

  /** \brief Get value at query location.
   *
   * @param query Eigen column vector representing a general query
   * @return value at location
   */
  virtual bool GetValue(const Eigen::Vector3d  &query, double &value){ value = 0; return true;}
  /** \brief Get value at query location.
   *
   * @param query Eigen column vector representing a general query
   * @return value at location
   */
  virtual bool GetValue(const Eigen::Vector3f  &query, double &value){ value = 0; return true;}
  /** \brief Get value at query index.
   *
   * @param query Eigen column vector representing a general query
   * @return value at location
   */
  virtual bool GetValue(const Eigen::Vector3i  &query, double &value){ value = 0; return true;}
  /** \brief Get value at query locations.
   *
   * @param query a vector of Eigen columns vector representing a general query index.
   * @return a vector of values at query locations.
   */
  virtual std::vector<std::pair<double, bool> > GetValue(const std::vector<Eigen::Vector3d > &query){
      std::pair<double, bool> p(0,true);
      std::vector<std::pair<double, bool> > vector_p;
      for(auto i=0; i< query.size(); i++){
        vector_p.push_back(p);
      }
      return vector_p;
    }
  /** \brief Get value at query indices.
   *
   * @param query a vector of Eigen columns vector representing general query indices.
   * @return a vector of values at query locations.
   */
  virtual std::vector<std::pair<T, bool> > GetValue(const std::vector<Eigen::Matrix<int, N, 1> > &query) = 0;
  /**
   * \brief Is the query location valid.
   * @param query
   * @return boolean whether the query is valid or not
   */
  virtual bool IsValid(const Eigen::Vector3d  &query){return true; }
  /**
    * \brief Are the query indices valid.
    * @param query
    * @return boolean whether the query is valid or not
    */
  virtual bool IsValid(const Eigen::Vector3i  &query){ return true;}
  /**
   * \brief Returns the minimum valid query that can be made to the representation
   * @param
   * @return minimum valid query location.
   */
  virtual Eigen::Vector3d GetMinQuery(){ return Eigen::Vector3d(0,0,0);}
  /**
   * \brief Returns the maximum valid query
   * @param
   * @return maximum valid query location.
   */
  virtual Eigen::Vector3d GetMaxQuery(){ return Eigen::Vector3d(0,0,0);}
  /**
   * \brief Returns the resolution at which the grid locally operates at that location
   * @param query
   * @return resolution
   */
  virtual Eigen::Vector3d GetResolution(const Eigen::Vector3d &query){ return Eigen::Vector3d(0,0,0);}
  /**
   * \brief Returns the resolution at which the grid locally operates at that index
   * @param query
   * @return resolution
   */
  virtual Eigen::Vector3d GetResolution(const Eigen::Vector3i &query){return Eigen::Vector3d(0,0,0);}
  /**
   * \brief returns the frame in which the representation exists,
   * @param
   * @return the frames in which the representation exists
   */
  virtual std::string get_frame(){return frame;}

  void set_frame(std::string temp_frame){frame = temp_frame;}
};

}  // namepsace ca
}  // namespace representation_interface


#endif  // REPRESENTATION_INTERFACE_INCLUDE_REPRESENTATION_INTERFACE_INTERFACE_BASE_H
