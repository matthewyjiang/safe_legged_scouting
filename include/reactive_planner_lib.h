#ifndef REACTIVE_PLANNER_LIB_H
#define REACTIVE_PLANNER_LIB_H

// MIT License (modified)

// Copyright (c) 2020 The Trustees of the University of Pennsylvania
// Authors:
// Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this **file** (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Boost imports
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

// ROS imports
#include <rclcpp/rclcpp.hpp>

// Local imports
#include <polygeom_lib.h>

// Other imports
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <vector>

// Define namespaces
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Define various geometries
using point = bg::model::point < double, 2, bg::cs::cartesian >;
using polygon = bg::model::polygon < point, false, true >;
using line = bg::model::linestring < point >;
using multi_point = bg::model::multi_point < point >;
using multi_polygon = bg::model::multi_polygon < polygon >;

// RULES: 1) Use RobotPosition as a point
//        2) Use RobotPosition separately from RobotOrientation

std::vector < double > linspace(double a, double b, uint16_t n);

// *INDENT-OFF*
template < typename T > class SmartVector: public std::vector < T > {
public:
  // act like operator []
  T operator()(size_t _Pos) {
    return (*this)[_Pos];
  }

  // act like MATLAB operator ()
  SmartVector < T > operator()(std::vector < size_t > &positions) {
    SmartVector < T > sub;
    sub.resize(positions.size());
    size_t sub_i = 0;
    for (std::vector < size_t > ::iterator pit = positions.begin();
      pit != positions.end(); pit++, sub_i++)
    {
      sub[sub_i] = (*this)[*pit];
    }
    return sub;
  }
};
// *INDENT-ON*

struct OutputStructVector
{
  /**
   * Struct that includes the value, jacobian and derivatives of the jacobian
   * for a vector-valued function at a specific point
   *
   * Properties:
   *  1) Value: Value of the function
   *  2) Jacobian: Jacobian of the function
   *  3) JacobianD: Derivatives of the jacobian in the order 11_x, 11_y, 12_x,
   * 12_y, 21_x, 21_y, 22_x, 22_y
   */
  std::vector < double > Value;
  std::vector < std::vector < double >> Jacobian;
  std::vector < double > JacobianD;
};

struct OutputStructScalar
{
  /**
   * Struct that includes the value, jacobian and hessian for a scalar-valued
   * function at a specific point
   *
   * Properties:
   *  1) Value: Value of the function
   *  2) Gradient: Gradient of the function
   *  3) Hessian: Hessian of the function
   */
  double Value;
  std::vector < double > Gradient;
  std::vector < std::vector < double >> Hessian;
};

class DiffeoParamsClass {
  /**
   * Class that describes the diffeomorphism parameters
   *
   * Properties:
   *  1) p: R-function exponent
   *  2) epsilon: Distance for the switches
   *  3) varepsilon: Distance allowed to dilate the polygon
   *  4) mu_1: Switch exponent mu1
   *  5) mu_2: Switch exponent mu2
   *  6) workspace: Polygonal boundary of the workspace
   */

public:
  DiffeoParamsClass() {
  }

  DiffeoParamsClass(
    double p_in, double epsilon_in, double varepsilon_in,
    double mu_1_in, double mu_2_in,
    std::vector < std::vector < double >> workspace_in) {
    this->p = p_in;
    this->epsilon = epsilon_in;
    this->varepsilon = varepsilon_in;
    this->mu_1 = mu_1_in;
    this->mu_2 = mu_2_in;
    this->workspace = workspace_in;
  }

  double get_p() const {return this->p;}
  double get_epsilon() const {return this->epsilon;}
  double get_varepsilon() const {return this->varepsilon;}
  double get_mu_1() const {return this->mu_1;}
  double get_mu_2() const {return this->mu_2;}
  std::vector < std::vector < double >> get_workspace() const {
    return this->workspace;
  }

  void set_p(double p_in) {this->p = p_in;}
  void set_epsilon(double epsilon_in) {this->epsilon = epsilon_in;}
  void set_varepsilon(double varepsilon_in)
  {
    this->varepsilon = varepsilon_in;
  }
  void set_mu_1(double mu_1_in) {this->mu_1 = mu_1_in;}
  void set_mu_2(double mu_2_in) {this->mu_2 = mu_2_in;}
  void set_workspace(std::vector < std::vector < double >> workspace_in)
  {
    this->workspace = workspace_in;
  }

  void set_all_params(
    double p_in, double epsilon_in, double varepsilon_in,
    double mu_1_in, double mu_2_in,
    std::vector < std::vector < double >> workspace_in)
  {
    this->p = p_in;
    this->epsilon = epsilon_in;
    this->varepsilon = varepsilon_in;
    this->mu_1 = mu_1_in;
    this->mu_2 = mu_2_in;
    this->workspace = workspace_in;
  }

private:
  double p;
  double epsilon;
  double varepsilon;
  double mu_1;
  double mu_2;
  std::vector < std::vector < double >> workspace;
};

void diffeoTreeTriangulation(
  std::vector < std::vector < double >> PolygonVertices,
  DiffeoParamsClass DiffeoParams,
  std::vector < TriangleClass > * tree);

void diffeoTreeConvex(
  std::vector < std::vector < double >> PolygonVertices,
  DiffeoParamsClass DiffeoParams,
  std::vector < PolygonClass > * tree);

OutputStructVector
polygonDiffeoTriangulation(
  std::vector < double > Position,
  std::vector < TriangleClass > DiffeoTree,
  DiffeoParamsClass DiffeoParams);

OutputStructVector polygonDiffeoConvex(
  std::vector < double > Position,
  std::vector < PolygonClass > DiffeoTree,
  DiffeoParamsClass DiffeoParams);

OutputStructVector triangleDiffeo(
  std::vector < double > Position,
  TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams);

OutputStructVector polygonDiffeo(
  std::vector < double > Position,
  PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar triangleSwitch(
  std::vector < double > Position,
  TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar polygonSwitch(
  std::vector < double > Position,
  PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar triangleDeformingFactor(
  std::vector < double > Position,
  TriangleClass Triangle);

OutputStructScalar polygonDeformingFactor(
  std::vector < double > Position,
  PolygonClass PolygonUsed);

OutputStructScalar triangleBetaSwitch(
  std::vector < double > Position,
  TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar polygonBetaSwitch(
  std::vector < double > Position,
  PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar triangleGammaSwitch(
  std::vector < double > Position,
  TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar polygonGammaSwitch(
  std::vector < double > Position,
  PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar triangleOutsideImplicit(
  std::vector < double > Position,
  TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar polygonOutsideImplicit(
  std::vector < double > Position,
  PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar triangleInsideImplicit(
  std::vector < double > Position,
  TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams);

OutputStructScalar polygonInsideImplicit(
  std::vector < double > Position,
  PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams);

struct DiffeoTransformResult
{
  std::vector < double > transformed_position;
  std::vector < std::vector < double >> transformed_jacobian;
  std::vector < double > transformed_hessian;
  double alpha1, alpha2, beta1, beta2;
  double transformed_orientation;
};

DiffeoTransformResult computeDiffeoTransform(
  std::vector < double > & robot_position, double robot_orientation,
  std::vector < std::vector < PolygonClass >> & diffeo_tree_array,
  DiffeoParamsClass & diffeo_params, rclcpp::Logger logger);

#endif // REACTIVE_PLANNER_LIB_H
