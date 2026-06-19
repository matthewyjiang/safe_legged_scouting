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

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <reactive_planner_lib.h>


// Define properties for dilations
const int points_per_circle = 5;
bg::strategy::buffer::join_round join_strategy(points_per_circle);
bg::strategy::buffer::end_flat end_strategy;
bg::strategy::buffer::point_circle circle_strategy;
bg::strategy::buffer::side_straight side_strategy;


std::vector<double> linspace(double a, double b, uint16_t n)
{
  std::vector<double> array;
  double step = (b - a) / (n - 1);
  array.push_back(a);

  while (array.size() < n) {
    a += step;               // could recode to better handle rounding errors
    array.push_back(a);
  }
  array.pop_back();
  array.push_back(b);
  return array;
}


void diffeoTreeTriangulation(
  std::vector<std::vector<double>> PolygonVertices,
  DiffeoParamsClass DiffeoParams, std::vector<TriangleClass> * tree)
{
  /**
   * Function that calculates the triangulation tree of a polygon and augments it with properties used in semantic navigation (based on the ear clipping method)
   *
   * Input:
   *  1) PolygonVertices: Vertex Coordinates of input polygon (start and end vertices must be the same)
   *  2) DiffeoParams: Options for the diffeomorphism construction
   *  3) tree: Stack of triangles with all the desired properties
   *
   */

  // Create a dummy origin
  point origin(0.0, 0.0);

  // Unpack diffeomorphism parameters
  double varepsilon = DiffeoParams.get_varepsilon();
  std::vector<std::vector<double>> workspace = DiffeoParams.get_workspace();

  // Construct a polygon based on the input vertices - TODO: Check that the polygon is closed
  polygon PolygonIn = BoostPointToBoostPoly(StdToBoostPoint(PolygonVertices));

  // Construct a line and a polygon based on the workspace
  line workspaceLine = BoostPointToBoostLine(StdToBoostPoint(workspace));
  polygon workspacePolygon = BoostPointToBoostPoly(StdToBoostPoint(workspace));

  // Check if the polygon intersects the workspace boundary
  if (bg::intersects(PolygonIn, workspaceLine)) {
    // Compute the intersection with the workspace
    multi_polygon polygon_to_use;
    bg::intersection(PolygonIn, workspacePolygon, polygon_to_use);

    // Find the vertices of the polygon
    std::vector<std::vector<double>> PolygonVertexList =
      BoostPointToStd(BoostPolyToBoostPoint(polygon_to_use[0]));

    // Compute the triangulation tree of the polygon with its dual (adjacency) graph
    polytriangulation(PolygonVertexList, workspace, true, tree);

    // Find the center and the adjacency edge to the boundary
    std::vector<point> last_triangle_vertices = tree->back().get_vertices();
    std::vector<double> dist_vector(3, 0.0);
    for (size_t i = 0; i < 3; i++) {
      dist_vector[i] = bg::distance(last_triangle_vertices[i], workspaceLine);
    }
    size_t min_dist_element =
      std::distance(dist_vector.begin(), std::min_element(dist_vector.begin(), dist_vector.end()));
    point median_point;
    switch (min_dist_element) {
      case 0:
        if (dist_vector[1] <= dist_vector[2]) {
          tree->back().set_vertices(
            {last_triangle_vertices[0], last_triangle_vertices[1],
              last_triangle_vertices[2]});
          tree->back().set_adj_edge({last_triangle_vertices[0], last_triangle_vertices[1]});
          median_point.set<0>(
            0.5 *
            (last_triangle_vertices[0].get<0>() + last_triangle_vertices[1].get<0>()));
          median_point.set<1>(
            0.5 *
            (last_triangle_vertices[0].get<1>() + last_triangle_vertices[1].get<1>()));
        } else {
          tree->back().set_vertices(
            {last_triangle_vertices[2], last_triangle_vertices[0],
              last_triangle_vertices[1]});
          tree->back().set_adj_edge({last_triangle_vertices[2], last_triangle_vertices[0]});
          median_point.set<0>(
            0.5 *
            (last_triangle_vertices[2].get<0>() + last_triangle_vertices[0].get<0>()));
          median_point.set<1>(
            0.5 *
            (last_triangle_vertices[2].get<1>() + last_triangle_vertices[0].get<1>()));
        }
        break;
      case 1:
        if (dist_vector[0] <= dist_vector[2]) {
          tree->back().set_vertices(
            {last_triangle_vertices[0], last_triangle_vertices[1],
              last_triangle_vertices[2]});
          tree->back().set_adj_edge({last_triangle_vertices[0], last_triangle_vertices[1]});
          median_point.set<0>(
            0.5 *
            (last_triangle_vertices[0].get<0>() + last_triangle_vertices[1].get<0>()));
          median_point.set<1>(
            0.5 *
            (last_triangle_vertices[0].get<1>() + last_triangle_vertices[1].get<1>()));
        } else {
          tree->back().set_vertices(
            {last_triangle_vertices[1], last_triangle_vertices[2],
              last_triangle_vertices[0]});
          tree->back().set_adj_edge({last_triangle_vertices[1], last_triangle_vertices[2]});
          median_point.set<0>(
            0.5 *
            (last_triangle_vertices[1].get<0>() + last_triangle_vertices[2].get<0>()));
          median_point.set<1>(
            0.5 *
            (last_triangle_vertices[1].get<1>() + last_triangle_vertices[2].get<1>()));
        }
        break;
      case 2:
        if (dist_vector[0] <= dist_vector[1]) {
          tree->back().set_vertices(
            {last_triangle_vertices[2], last_triangle_vertices[0],
              last_triangle_vertices[1]});
          tree->back().set_adj_edge({last_triangle_vertices[2], last_triangle_vertices[0]});
          median_point.set<0>(
            0.5 *
            (last_triangle_vertices[2].get<0>() + last_triangle_vertices[0].get<0>()));
          median_point.set<1>(
            0.5 *
            (last_triangle_vertices[2].get<1>() + last_triangle_vertices[0].get<1>()));
        } else {
          tree->back().set_vertices(
            {last_triangle_vertices[1], last_triangle_vertices[2],
              last_triangle_vertices[0]});
          tree->back().set_adj_edge({last_triangle_vertices[1], last_triangle_vertices[2]});
          median_point.set<0>(
            0.5 *
            (last_triangle_vertices[1].get<0>() + last_triangle_vertices[2].get<0>()));
          median_point.set<1>(
            0.5 *
            (last_triangle_vertices[1].get<1>() + last_triangle_vertices[2].get<1>()));
        }
        break;
    }
    last_triangle_vertices = tree->back().get_vertices();
    point median_ray(median_point.get<0>() - last_triangle_vertices[2].get<0>(),
      median_point.get<1>() - last_triangle_vertices[2].get<1>());
    median_ray.set<0>(median_ray.get<0>() / bg::distance(median_ray, origin));
    median_ray.set<1>(median_ray.get<1>() / bg::distance(median_ray, origin));
    point last_triangle_center(median_point.get<0>() + 1.0 * median_ray.get<0>(),
      median_point.get<1>() + 1.0 * median_ray.get<1>());
    tree->back().set_center(last_triangle_center);

    // Compute the tangent and normal vectors of the root triangle
    std::vector<point> r_t_vector =
    {point(
        (last_triangle_vertices[1].get<0>() - last_triangle_vertices[0].get<0>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_vertices[1]),
        (last_triangle_vertices[1].get<1>() - last_triangle_vertices[0].get<1>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_vertices[1])),
      point(
        (last_triangle_vertices[2].get<0>() - last_triangle_vertices[1].get<0>()) /
        bg::distance(last_triangle_vertices[1], last_triangle_vertices[2]),
        (last_triangle_vertices[2].get<1>() - last_triangle_vertices[1].get<1>()) /
        bg::distance(last_triangle_vertices[1], last_triangle_vertices[2])),
      point(
        (last_triangle_vertices[0].get<0>() - last_triangle_vertices[2].get<0>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_vertices[2]),
        (last_triangle_vertices[0].get<1>() - last_triangle_vertices[2].get<1>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_vertices[2]))};
    std::vector<point> r_n_vector = {point(-r_t_vector[0].get<1>(), r_t_vector[0].get<0>()), point(
        -r_t_vector[1].get<1>(), r_t_vector[1].get<0>()),
      point(-r_t_vector[2].get<1>(), r_t_vector[2].get<0>())};
    tree->back().set_r_t(r_t_vector);
    tree->back().set_r_n(r_n_vector);

    // Find the remaining tangents and normals from vertices 0 and 1 to the center
    std::vector<point> r_center_t_vector =
    {point(
        (last_triangle_center.get<0>() - last_triangle_vertices[0].get<0>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_center),
        (last_triangle_center.get<1>() - last_triangle_vertices[0].get<1>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_center)),
      point(
        (last_triangle_vertices[1].get<0>() - last_triangle_center.get<0>()) /
        bg::distance(last_triangle_vertices[1], last_triangle_center),
        (last_triangle_vertices[1].get<1>() - last_triangle_center.get<1>()) /
        bg::distance(last_triangle_vertices[1], last_triangle_center))};
    std::vector<point> r_center_n_vector =
    {point(-r_center_t_vector[0].get<1>(), r_center_t_vector[0].get<0>()), point(
        -r_center_t_vector[1].get<1>(), r_center_t_vector[1].get<0>())};
    tree->back().set_r_center_t(r_center_t_vector);
    tree->back().set_r_center_n(r_center_n_vector);

    // Compute the dilated polygon and truncate it by the rays emanating from the center
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(varepsilon);
    polygon last_triangle_polygon = BoostPointToBoostPoly(
      {last_triangle_vertices[0],
        last_triangle_vertices[1],
        last_triangle_vertices[2],
        last_triangle_vertices[0]});
    multi_polygon last_triangle_multipolygon_dilated;
    bg::buffer(
      last_triangle_polygon, last_triangle_multipolygon_dilated, distance_strategy,
      side_strategy, join_strategy, end_strategy, circle_strategy);
    polygon last_triangle_polygon_dilated = last_triangle_multipolygon_dilated.front();
    polygon intersect_1 = cvxpolyxhplane(
      last_triangle_polygon_dilated, last_triangle_center,
      tree->back().get_r_center_n().front());
    polygon intersect_2 = cvxpolyxhplane(
      intersect_1, last_triangle_center,
      tree->back().get_r_center_n().back());

    // Compute the intersection with the workspace
    multi_polygon output_1;
    multi_polygon output_2;
    polygon final_polygon;
    bg::intersection(intersect_2, workspacePolygon, output_1);
    bg::union_(
      output_1[0],
      BoostPointToBoostPoly(
        {last_triangle_center, last_triangle_vertices[1],
          last_triangle_vertices[2], last_triangle_vertices[0], last_triangle_center}), output_2);
    bg::simplify(output_2[0], final_polygon, 0.02);
    std::vector<point> last_triangle_vertices_tilde = BoostPolyToBoostPoint(final_polygon);
    last_triangle_vertices_tilde.pop_back();
    tree->back().set_vertices_tilde(last_triangle_vertices_tilde);

    // Find the tangent and normal vectors for the generated polygonal collar
    std::vector<point> r_tilde_t_vector, r_tilde_n_vector;
    for (size_t i = 0; i < last_triangle_vertices_tilde.size(); i++) {
      size_t j = (i + 1) % (last_triangle_vertices_tilde.size());
      double dist_ij =
        bg::distance(last_triangle_vertices_tilde[i], last_triangle_vertices_tilde[j]);
      r_tilde_t_vector.push_back(
        point(
          (last_triangle_vertices_tilde[j].get<0>() -
          last_triangle_vertices_tilde[i].get<0>()) / dist_ij,
          (last_triangle_vertices_tilde[j].get<1>() - last_triangle_vertices_tilde[i].get<1>()) /
          dist_ij));
      r_tilde_n_vector.push_back(
        point(
          -(last_triangle_vertices_tilde[j].get<1>() -
          last_triangle_vertices_tilde[i].get<1>()) / dist_ij,
          (last_triangle_vertices_tilde[j].get<0>() - last_triangle_vertices_tilde[i].get<0>()) /
          dist_ij));
    }
    tree->back().set_r_tilde_t(r_tilde_t_vector);
    tree->back().set_r_tilde_n(r_tilde_n_vector);

    // Add a dummy radius
    tree->back().set_radius(0.0);
  } else {
    // Compute the triangulation tree of the polygon with its dual (adjacency) graph
    polytriangulation(PolygonVertices, workspace, false, tree);

    // Find the center and radius of the root
    std::vector<point> last_triangle_vertices = tree->back().get_vertices();
    tree->back().set_center(
      point(
        (last_triangle_vertices[0].get<0>() +
        last_triangle_vertices[1].get<0>() + last_triangle_vertices[2].get<0>()) / 3.0,
        (last_triangle_vertices[0].get<1>() + last_triangle_vertices[1].get<1>() +
        last_triangle_vertices[2].get<1>()) / 3.0));
    tree->back().set_radius(
      0.8 *
      bg::distance(
        tree->back().get_center(),
        BoostPointToBoostLine(
          {last_triangle_vertices[0], last_triangle_vertices[1],
            last_triangle_vertices[2], last_triangle_vertices[0]})));

    // Compute the tangent and normal vectors of the root triangle
    std::vector<point> r_t_vector =
    {point(
        (last_triangle_vertices[1].get<0>() - last_triangle_vertices[0].get<0>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_vertices[1]),
        (last_triangle_vertices[1].get<1>() - last_triangle_vertices[0].get<1>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_vertices[1])),
      point(
        (last_triangle_vertices[2].get<0>() - last_triangle_vertices[1].get<0>()) /
        bg::distance(last_triangle_vertices[1], last_triangle_vertices[2]),
        (last_triangle_vertices[2].get<1>() - last_triangle_vertices[1].get<1>()) /
        bg::distance(last_triangle_vertices[1], last_triangle_vertices[2])),
      point(
        (last_triangle_vertices[0].get<0>() - last_triangle_vertices[2].get<0>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_vertices[2]),
        (last_triangle_vertices[0].get<1>() - last_triangle_vertices[2].get<1>()) /
        bg::distance(last_triangle_vertices[0], last_triangle_vertices[2]))};
    std::vector<point> r_n_vector = {point(-r_t_vector[0].get<1>(), r_t_vector[0].get<0>()), point(
        -r_t_vector[1].get<1>(), r_t_vector[1].get<0>()),
      point(-r_t_vector[2].get<1>(), r_t_vector[2].get<0>())};
    tree->back().set_r_t(r_t_vector);
    tree->back().set_r_n(r_n_vector);

    // Find the polygonal color for the root by dilating the triangle by varepsilon
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(varepsilon);
    polygon last_triangle_polygon = BoostPointToBoostPoly(
      {last_triangle_vertices[0],
        last_triangle_vertices[1],
        last_triangle_vertices[2],
        last_triangle_vertices[0]});
    multi_polygon last_triangle_multipolygon_dilated;
    bg::buffer(
      last_triangle_polygon, last_triangle_multipolygon_dilated, distance_strategy,
      side_strategy, join_strategy, end_strategy, circle_strategy);
    polygon last_triangle_polygon_dilated = last_triangle_multipolygon_dilated.front();

    // Compute the intersection with the workspace
    multi_polygon output_1;
    polygon final_polygon;
    bg::intersection(last_triangle_polygon_dilated, workspacePolygon, output_1);
    bg::simplify(output_1[0], final_polygon, 0.02);
    std::vector<point> last_triangle_vertices_tilde = BoostPolyToBoostPoint(final_polygon);
    last_triangle_vertices_tilde.pop_back();
    tree->back().set_vertices_tilde(last_triangle_vertices_tilde);

    // Find the tangent and normal vectors for the generated polygonal collar
    std::vector<point> r_tilde_t_vector, r_tilde_n_vector;
    for (size_t i = 0; i < last_triangle_vertices_tilde.size(); i++) {
      size_t j = (i + 1) % (last_triangle_vertices_tilde.size());
      double dist_ij =
        bg::distance(last_triangle_vertices_tilde[i], last_triangle_vertices_tilde[j]);
      r_tilde_t_vector.push_back(
        point(
          (last_triangle_vertices_tilde[j].get<0>() -
          last_triangle_vertices_tilde[i].get<0>()) / dist_ij,
          (last_triangle_vertices_tilde[j].get<1>() - last_triangle_vertices_tilde[i].get<1>()) /
          dist_ij));
      r_tilde_n_vector.push_back(
        point(
          -(last_triangle_vertices_tilde[j].get<1>() -
          last_triangle_vertices_tilde[i].get<1>()) / dist_ij,
          (last_triangle_vertices_tilde[j].get<0>() - last_triangle_vertices_tilde[i].get<0>()) /
          dist_ij));
    }
    tree->back().set_r_tilde_t(r_tilde_t_vector);
    tree->back().set_r_tilde_n(r_tilde_n_vector);
  }

  // Identify all the children properties
  for (size_t i = 0; i < tree->size() - 1; i++) {
    // Compute the tangent and normal vectors of the child hyperplanes
    // r0 is always the shared edge between the parent and the child, r1 and r2 the rest in CCW order
    std::vector<point> triangle_vertices = (*tree)[i].get_vertices();
    std::vector<point> r_t_vector =
    {point(
        (triangle_vertices[1].get<0>() - triangle_vertices[0].get<0>()) /
        bg::distance(triangle_vertices[0], triangle_vertices[1]),
        (triangle_vertices[1].get<1>() - triangle_vertices[0].get<1>()) /
        bg::distance(triangle_vertices[0], triangle_vertices[1])),
      point(
        (triangle_vertices[2].get<0>() - triangle_vertices[1].get<0>()) /
        bg::distance(triangle_vertices[1], triangle_vertices[2]),
        (triangle_vertices[2].get<1>() - triangle_vertices[1].get<1>()) /
        bg::distance(triangle_vertices[1], triangle_vertices[2])),
      point(
        (triangle_vertices[0].get<0>() - triangle_vertices[2].get<0>()) /
        bg::distance(triangle_vertices[0], triangle_vertices[2]),
        (triangle_vertices[0].get<1>() - triangle_vertices[2].get<1>()) /
        bg::distance(triangle_vertices[0], triangle_vertices[2]))};
    std::vector<point> r_n_vector = {point(-r_t_vector[0].get<1>(), r_t_vector[0].get<0>()), point(
        -r_t_vector[1].get<1>(), r_t_vector[1].get<0>()),
      point(-r_t_vector[2].get<1>(), r_t_vector[2].get<0>())};
    (*tree)[i].set_r_t(r_t_vector);
    (*tree)[i].set_r_n(r_n_vector);

    // Find the median from the 3rd point to the shared edge and from that compute the center for the purging transformation
    std::vector<point> triangle_adj_edge = (*tree)[i].get_adj_edge();
    point median_point;
    median_point.set<0>(0.5 * (triangle_adj_edge[0].get<0>() + triangle_adj_edge[1].get<0>()));
    median_point.set<1>(0.5 * (triangle_adj_edge[0].get<1>() + triangle_adj_edge[1].get<1>()));
    point median_ray(median_point.get<0>() - triangle_vertices[2].get<0>(),
      median_point.get<1>() - triangle_vertices[2].get<1>());
    median_ray.set<0>(median_ray.get<0>() / bg::distance(median_ray, origin));
    median_ray.set<1>(median_ray.get<1>() / bg::distance(median_ray, origin));
    std::vector<point> predecessor_triangle_vertices =
      (*tree)[(*tree)[i].get_predecessor()].get_vertices();
    point intersection_point =
      polyxray(
      BoostPointToBoostPoly(
        {predecessor_triangle_vertices[0],
          predecessor_triangle_vertices[1], predecessor_triangle_vertices[2],
          predecessor_triangle_vertices[0]}), median_point, median_ray);
    point triangle_center(0.2 * median_point.get<0>() + 0.8 * intersection_point.get<0>(),
      0.2 * median_point.get<1>() + 0.8 * intersection_point.get<1>());
    (*tree)[i].set_center(triangle_center);

    // Find the remaining tangents and normals from vertices 0 and 1 to the center
    std::vector<point> r_center_t_vector =
    {point(
        (triangle_center.get<0>() - triangle_vertices[0].get<0>()) /
        bg::distance(triangle_vertices[0], triangle_center),
        (triangle_center.get<1>() - triangle_vertices[0].get<1>()) /
        bg::distance(triangle_vertices[0], triangle_center)),
      point(
        (triangle_vertices[1].get<0>() - triangle_center.get<0>()) /
        bg::distance(triangle_vertices[1], triangle_center),
        (triangle_vertices[1].get<1>() - triangle_center.get<1>()) /
        bg::distance(triangle_vertices[1], triangle_center))};
    std::vector<point> r_center_n_vector =
    {point(-r_center_t_vector[0].get<1>(), r_center_t_vector[0].get<0>()), point(
        -r_center_t_vector[1].get<1>(), r_center_t_vector[1].get<0>())};
    (*tree)[i].set_r_center_t(r_center_t_vector);
    (*tree)[i].set_r_center_n(r_center_n_vector);

    // Compute the dilated polygon and truncate it by the rays emanating from the center
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(varepsilon);
    polygon triangle_polygon = BoostPointToBoostPoly(
      {triangle_vertices[0], triangle_vertices[1],
        triangle_vertices[2], triangle_vertices[0]});
    multi_polygon triangle_multipolygon_dilated;
    bg::buffer(
      triangle_polygon, triangle_multipolygon_dilated, distance_strategy, side_strategy,
      join_strategy, end_strategy, circle_strategy);
    polygon triangle_polygon_dilated = triangle_multipolygon_dilated.front();
    polygon intersect_1 = cvxpolyxhplane(
      triangle_polygon_dilated, triangle_center,
      (*tree)[i].get_r_center_n().front());
    polygon intersect_2 = cvxpolyxhplane(
      intersect_1, triangle_center,
      (*tree)[i].get_r_center_n().back());
    polygon candidate_polygon = intersect_2;

    // Check for collisions with all the triangles that will succeed i in the diffeomorphism construction except for its parent
    for (size_t j = i + 1; j < tree->size(); j++) {
      if (j == (*tree)[i].get_predecessor()) {
        continue;
      } else {
        std::vector<point> triangle_to_test_vertices = (*tree)[j].get_vertices();
        polygon triangle_to_test = BoostPointToBoostPoly(
          {triangle_to_test_vertices[0],
            triangle_to_test_vertices[1],
            triangle_to_test_vertices[2],
            triangle_to_test_vertices[0]});
        multi_polygon difference_output;
        bg::difference(candidate_polygon, triangle_to_test, difference_output);

        // If the difference operation created a multipolygon, keep only the polygon that contains the barycenter of the extended triangle
        point point_to_consider((triangle_vertices[0].get<0>() + triangle_vertices[1].get<0>() +
          triangle_center.get<0>()) / 3.0,
          (triangle_vertices[0].get<1>() + triangle_vertices[1].get<1>() +
          triangle_center.get<1>()) / 3.0);
        BOOST_FOREACH(polygon const & difference_component, difference_output) {
          if (bg::within(point_to_consider, difference_component)) {
            candidate_polygon = difference_component;
            break;
          }
        }
      }
    }

    // Extract final vertices
    polygon candidate_polygon_simplified = candidate_polygon;
    bg::simplify(candidate_polygon, candidate_polygon_simplified, 0.02);
    std::vector<std::vector<double>> candidate_polygon_vertices = BoostPointToStd(
      BoostPolyToBoostPoint(
        candidate_polygon_simplified));
    candidate_polygon_vertices.pop_back();

    // Decompose the polygon into its convex pieces and find the piece that includes the barycenter of the extended triangle
    CGAL_Polygon_2 cgal_polygon;
    CGAL_Polygon_list partition_polys;
    CGAL_Traits partition_traits;
    CGAL::set_pretty_mode(std::cout);
    std::vector<point> final_polygon_vertices;
    for (size_t k = 0; k < candidate_polygon_vertices.size(); k++) {
      cgal_polygon.push_back(
        CGAL_Point_2(
          candidate_polygon_vertices[k][0],
          candidate_polygon_vertices[k][1]));
    }
    if (cgal_polygon.is_simple()) {
      // std::cout << cgal_polygon << std::endl;
      CGAL::greene_approx_convex_partition_2(
        cgal_polygon.vertices_begin(),
        cgal_polygon.vertices_end(),
        std::back_inserter(partition_polys),
        partition_traits);
      assert(
        CGAL::convex_partition_is_valid_2(
          cgal_polygon.vertices_begin(),
          cgal_polygon.vertices_end(),
          partition_polys.begin(),
          partition_polys.end(),
          partition_traits));
      CGAL_Point_2 cgal_point_to_consider((triangle_vertices[0].get<0>() +
        triangle_vertices[1].get<0>() + triangle_center.get<0>()) / 3.0,
        (triangle_vertices[0].get<1>() + triangle_vertices[1].get<1>() + triangle_center.get<1>()) /
        3.0);
      for (size_t k = 0; k < partition_polys.size(); k++) {
        auto it = std::next(partition_polys.begin(), k);
        CGAL_Polygon_2 polygon_to_consider = *it;
        final_polygon_vertices = {};
        for (size_t l = 0; l < polygon_to_consider.size(); l++) {
          final_polygon_vertices.push_back(
            point(
              polygon_to_consider.vertex(l).x(),
              polygon_to_consider.vertex(l).y()));
        }
        final_polygon_vertices.push_back(final_polygon_vertices[0]);
        if (bg::within(
            point(
              (triangle_vertices[0].get<0>() + triangle_vertices[1].get<0>() +
              triangle_center.get<0>()) / 3.0,
              (triangle_vertices[0].get<1>() + triangle_vertices[1].get<1>() +
              triangle_center.get<1>()) / 3.0), BoostPointToBoostPoly(final_polygon_vertices)))
        {
          break;
        }
      }
    } else {
      final_polygon_vertices = BoostPolyToBoostPoint(intersect_2);
    }

    // Generate the outer polygonal collar
    polygon final_polygon = BoostPointToBoostPoly(final_polygon_vertices);
    multi_polygon output;
    bg::intersection(final_polygon, workspacePolygon, output);
    std::vector<point> output_vertices = BoostPolyToBoostPoint(output[0]);
    output_vertices.pop_back();
    (*tree)[i].set_vertices_tilde(output_vertices);

    // Find the tangent and normal vectors for the generated polygonal collar
    std::vector<point> r_tilde_t_vector, r_tilde_n_vector;
    for (size_t k = 0; k < output_vertices.size(); k++) {
      size_t l = (k + 1) % (output_vertices.size());
      double dist_kl = bg::distance(output_vertices[k], output_vertices[l]);
      r_tilde_t_vector.push_back(
        point(
          (output_vertices[l].get<0>() - output_vertices[k].get<0>()) /
          dist_kl, (output_vertices[l].get<1>() - output_vertices[k].get<1>()) / dist_kl));
      r_tilde_n_vector.push_back(
        point(
          -(output_vertices[l].get<1>() -
          output_vertices[k].get<1>()) / dist_kl,
          (output_vertices[l].get<0>() - output_vertices[k].get<0>()) / dist_kl));
    }
    (*tree)[i].set_r_tilde_t(r_tilde_t_vector);
    (*tree)[i].set_r_tilde_n(r_tilde_n_vector);
  }

  return;
}


void diffeoTreeConvex(
  std::vector<std::vector<double>> PolygonVertices,
  DiffeoParamsClass DiffeoParams, std::vector<PolygonClass> * tree)
{
  /**
   * Function that calculates the convex decomposition tree of a polygon and augments it with properties used in semantic navigation
   *
   * Input:
   *  1) PolygonVertices: Vertex Coordinates of input polygon (start and end vertices must be the same)
   *  2) DiffeoParams: Options for the diffeomorphism construction
   *  3) tree: Stack of triangles with all the desired properties
   *
   */

  // Create a dummy origin
  point origin(0.0, 0.0);

  // Unpack diffeomorphism parameters
  double varepsilon = DiffeoParams.get_varepsilon();
  std::vector<std::vector<double>> workspace = DiffeoParams.get_workspace();

  // Construct a polygon based on the input vertices
  polygon PolygonIn = BoostPointToBoostPoly(StdToBoostPoint(PolygonVertices));

  // Construct a line and a polygon based on the workspace
  line workspaceLine = BoostPointToBoostLine(StdToBoostPoint(workspace));
  polygon workspacePolygon = BoostPointToBoostPoly(StdToBoostPoint(workspace));

  // Check if the polygon intersects the workspace boundary
  if (bg::intersects(PolygonIn, workspaceLine)) {
    // Compute the intersection with the workspace
    multi_polygon polygon_to_use;
    bg::intersection(PolygonIn, workspacePolygon, polygon_to_use);

    // Find the vertices of the polygon
    std::vector<std::vector<double>> PolygonVertexList =
      BoostPointToStd(BoostPolyToBoostPoint(polygon_to_use[0]));

    // Compute the convex decomposition tree of the polygon with its dual (adjacency) graph
    polyconvexdecomposition(PolygonVertexList, workspace, true, tree);

    // Find the adjacency edge to the boundary
    std::vector<point> last_polygon_vertices = tree->back().get_vertices();
    std::vector<double> dist_vector(last_polygon_vertices.size(), 0.0);
    for (size_t i = 0; i < last_polygon_vertices.size(); i++) {
      dist_vector[i] = bg::distance(last_polygon_vertices[i], workspaceLine);
    }
    int min_dist_element =
      std::distance(dist_vector.begin(), std::min_element(dist_vector.begin(), dist_vector.end()));
    std::vector<point> new_root_vertices;
    int last_polygon_vertices_size = static_cast<int>(last_polygon_vertices.size());
    if (dist_vector[(min_dist_element + 1) % last_polygon_vertices_size] >=
      dist_vector[(last_polygon_vertices_size +
      ((min_dist_element - 1) % last_polygon_vertices_size)) % last_polygon_vertices_size])
    {
      for (int j = 0; j < last_polygon_vertices_size; j++) {
        new_root_vertices.push_back(
          point(
            last_polygon_vertices[(last_polygon_vertices_size +
            ((min_dist_element - 1 + j) % last_polygon_vertices_size)) %
            last_polygon_vertices_size].get<0>(),
            last_polygon_vertices[(last_polygon_vertices_size +
            ((min_dist_element - 1 + j) % last_polygon_vertices_size)) %
            last_polygon_vertices_size].get<1>()));
      }
    } else {
      for (int j = 0; j < last_polygon_vertices.size(); j++) {
        new_root_vertices.push_back(
          point(
            last_polygon_vertices[(min_dist_element + j) %
            last_polygon_vertices.size()].get<0>(),
            last_polygon_vertices[(min_dist_element + j) % last_polygon_vertices.size()].get<1>()));
      }
    }
    tree->back().set_vertices(new_root_vertices);
    last_polygon_vertices = tree->back().get_vertices();
    std::vector<point> last_polygon_adj_edge = {last_polygon_vertices[0], last_polygon_vertices[1]};
    tree->back().set_adj_edge(last_polygon_adj_edge);

    // Find the center of transformation
    point median_point(0.5 *
      (last_polygon_vertices[1].get<0>() + last_polygon_vertices[0].get<0>()),
      0.5 * (last_polygon_vertices[1].get<1>() + last_polygon_vertices[0].get<1>()));
    point median_ray(-(last_polygon_adj_edge[0].get<1>() - last_polygon_adj_edge[1].get<1>()),
      last_polygon_adj_edge[0].get<0>() - last_polygon_adj_edge[1].get<0>());
    median_ray.set<0>(median_ray.get<0>() / bg::distance(median_ray, origin));
    median_ray.set<1>(median_ray.get<1>() / bg::distance(median_ray, origin));
    point last_polygon_center(median_point.get<0>() + 0.1 * median_ray.get<0>(),
      median_point.get<1>() + 0.1 * median_ray.get<1>());
    tree->back().set_center(last_polygon_center);

    // Find the tangent and normal vectors for the generated polygon
    std::vector<point> r_t_vector, r_n_vector;
    for (size_t i = 0; i < last_polygon_vertices.size(); i++) {
      size_t j = (i + 1) % (last_polygon_vertices.size());
      double dist_ij = bg::distance(last_polygon_vertices[i], last_polygon_vertices[j]);
      r_t_vector.push_back(
        point(
          (last_polygon_vertices[j].get<0>() -
          last_polygon_vertices[i].get<0>()) / dist_ij,
          (last_polygon_vertices[j].get<1>() - last_polygon_vertices[i].get<1>()) / dist_ij));
      r_n_vector.push_back(
        point(
          -(last_polygon_vertices[j].get<1>() -
          last_polygon_vertices[i].get<1>()) / dist_ij,
          (last_polygon_vertices[j].get<0>() - last_polygon_vertices[i].get<0>()) / dist_ij));
    }
    tree->back().set_r_t(r_t_vector);
    tree->back().set_r_n(r_n_vector);

    // Find the remaining tangents and normals from vertices 0 and 1 to the center
    std::vector<point> r_center_t_vector =
    {point(
        (last_polygon_center.get<0>() - last_polygon_vertices[0].get<0>()) /
        bg::distance(last_polygon_vertices[0], last_polygon_center),
        (last_polygon_center.get<1>() - last_polygon_vertices[0].get<1>()) /
        bg::distance(last_polygon_vertices[0], last_polygon_center)),
      point(
        (last_polygon_vertices[1].get<0>() - last_polygon_center.get<0>()) /
        bg::distance(last_polygon_vertices[1], last_polygon_center),
        (last_polygon_vertices[1].get<1>() - last_polygon_center.get<1>()) /
        bg::distance(last_polygon_vertices[1], last_polygon_center))};
    std::vector<point> r_center_n_vector =
    {point(-r_center_t_vector[0].get<1>(), r_center_t_vector[0].get<0>()), point(
        -r_center_t_vector[1].get<1>(), r_center_t_vector[1].get<0>())};
    tree->back().set_r_center_t(r_center_t_vector);
    tree->back().set_r_center_n(r_center_n_vector);

    // Compute the dilated polygon and truncate it by the rays emanating from the center
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(varepsilon);
    std::vector<point> polygon_to_consider = last_polygon_vertices;
    polygon_to_consider.push_back(last_polygon_vertices[0]);
    polygon last_polygon = BoostPointToBoostPoly(polygon_to_consider);
    multi_polygon last_polygon_multipolygon_dilated;
    bg::buffer(
      last_polygon, last_polygon_multipolygon_dilated, distance_strategy, side_strategy,
      join_strategy, end_strategy, circle_strategy);
    polygon last_triangle_polygon_dilated = last_polygon_multipolygon_dilated.front();
    polygon intersect_1 = cvxpolyxhplane(
      last_triangle_polygon_dilated, last_polygon_center,
      tree->back().get_r_center_n().front());
    polygon intersect_2 = cvxpolyxhplane(
      intersect_1, last_polygon_center,
      tree->back().get_r_center_n().back());

    // Compute the intersection with the workspace
    multi_polygon output_1, output_2;
    polygon final_polygon;
    bg::intersection(intersect_2, workspacePolygon, output_1);
    bg::union_(
      output_1[0],
      BoostPointToBoostPoly(
        {last_polygon_center, last_polygon_vertices[1],
          last_polygon_vertices[2], last_polygon_vertices[0], last_polygon_center}), output_2);
    bg::simplify(output_2[0], final_polygon, 0.02);
    std::vector<point> last_polygon_vertices_tilde = BoostPolyToBoostPoint(final_polygon);
    last_polygon_vertices_tilde.pop_back();
    tree->back().set_vertices_tilde(last_polygon_vertices_tilde);

    // Find the tangent and normal vectors for the generated polygonal collar
    std::vector<point> r_tilde_t_vector, r_tilde_n_vector;
    for (size_t i = 0; i < last_polygon_vertices_tilde.size(); i++) {
      size_t j = (i + 1) % (last_polygon_vertices_tilde.size());
      double dist_ij = bg::distance(last_polygon_vertices_tilde[i], last_polygon_vertices_tilde[j]);
      r_tilde_t_vector.push_back(
        point(
          (last_polygon_vertices_tilde[j].get<0>() -
          last_polygon_vertices_tilde[i].get<0>()) / dist_ij,
          (last_polygon_vertices_tilde[j].get<1>() - last_polygon_vertices_tilde[i].get<1>()) /
          dist_ij));
      r_tilde_n_vector.push_back(
        point(
          -(last_polygon_vertices_tilde[j].get<1>() -
          last_polygon_vertices_tilde[i].get<1>()) / dist_ij,
          (last_polygon_vertices_tilde[j].get<0>() - last_polygon_vertices_tilde[i].get<0>()) /
          dist_ij));
    }
    tree->back().set_r_tilde_t(r_tilde_t_vector);
    tree->back().set_r_tilde_n(r_tilde_n_vector);

    // Finally, compute the augmented inner polygon that includes the center of deformation and update
    last_polygon_vertices.insert(last_polygon_vertices.begin() + 1, last_polygon_center);
    r_t_vector.erase(r_t_vector.begin());
    r_n_vector.erase(r_n_vector.begin());
    r_t_vector.insert(r_t_vector.begin(), r_center_t_vector[1]);
    r_t_vector.insert(r_t_vector.begin(), r_center_t_vector[0]);
    r_n_vector.insert(r_n_vector.begin(), r_center_n_vector[1]);
    r_n_vector.insert(r_n_vector.begin(), r_center_n_vector[0]);
    tree->back().set_augmented_vertices(last_polygon_vertices);
    tree->back().set_r_t(r_t_vector);
    tree->back().set_r_n(r_n_vector);

    // Add a dummy radius
    tree->back().set_radius(0.0);
  } else {
    // Compute the convex decomposition tree of the polygon with its dual (adjacency) graph
    polyconvexdecomposition(PolygonVertices, workspace, false, tree);

    // Find the center of the root
    std::vector<point> last_polygon_vertices = tree->back().get_vertices();
    tree->back().set_augmented_vertices(last_polygon_vertices);
    double sum_x = 0.0, sum_y = 0.0;
    for (size_t i = 0; i < last_polygon_vertices.size(); i++) {
      sum_x = sum_x + last_polygon_vertices[i].get<0>();
      sum_y = sum_y + last_polygon_vertices[i].get<1>();
    }
    double polygon_vertex_size = (double)last_polygon_vertices.size();
    tree->back().set_center(point(sum_x / polygon_vertex_size, sum_y / polygon_vertex_size));

    // Find the radius of the root
    std::vector<point> polygon_to_consider = last_polygon_vertices;
    polygon_to_consider.push_back(last_polygon_vertices[0]);
    tree->back().set_radius(
      0.8 *
      bg::distance(tree->back().get_center(), BoostPointToBoostLine(polygon_to_consider)));

    // Compute the tangent and normal vectors of the root polygon
    std::vector<point> r_t_vector, r_n_vector;
    for (size_t i = 0; i < last_polygon_vertices.size(); i++) {
      size_t j = (i + 1) % (last_polygon_vertices.size());
      double dist_ij = bg::distance(last_polygon_vertices[i], last_polygon_vertices[j]);
      r_t_vector.push_back(
        point(
          (last_polygon_vertices[j].get<0>() -
          last_polygon_vertices[i].get<0>()) / dist_ij,
          (last_polygon_vertices[j].get<1>() - last_polygon_vertices[i].get<1>()) / dist_ij));
      r_n_vector.push_back(
        point(
          -(last_polygon_vertices[j].get<1>() -
          last_polygon_vertices[i].get<1>()) / dist_ij,
          (last_polygon_vertices[j].get<0>() - last_polygon_vertices[i].get<0>()) / dist_ij));
    }
    tree->back().set_r_t(r_t_vector);
    tree->back().set_r_n(r_n_vector);

    // Find the polygonal color for the root by dilating the triangle by varepsilon
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(varepsilon);
    polygon last_polygon = BoostPointToBoostPoly(polygon_to_consider);
    multi_polygon last_polygon_multipolygon_dilated;
    bg::buffer(
      last_polygon, last_polygon_multipolygon_dilated, distance_strategy, side_strategy,
      join_strategy, end_strategy, circle_strategy);
    polygon last_polygon_dilated = last_polygon_multipolygon_dilated.front();

    // Compute the intersection with the workspace
    multi_polygon output_1;
    polygon final_polygon;
    bg::intersection(last_polygon_dilated, workspacePolygon, output_1);
    bg::simplify(output_1[0], final_polygon, 0.02);
    std::vector<point> last_polygon_vertices_tilde = BoostPolyToBoostPoint(final_polygon);
    last_polygon_vertices_tilde.pop_back();
    tree->back().set_vertices_tilde(last_polygon_vertices_tilde);

    // Find the tangent and normal vectors for the generated polygonal collar
    std::vector<point> r_tilde_t_vector, r_tilde_n_vector;
    for (size_t i = 0; i < last_polygon_vertices_tilde.size(); i++) {
      size_t j = (i + 1) % (last_polygon_vertices_tilde.size());
      double dist_ij = bg::distance(last_polygon_vertices_tilde[i], last_polygon_vertices_tilde[j]);
      r_tilde_t_vector.push_back(
        point(
          (last_polygon_vertices_tilde[j].get<0>() -
          last_polygon_vertices_tilde[i].get<0>()) / dist_ij,
          (last_polygon_vertices_tilde[j].get<1>() - last_polygon_vertices_tilde[i].get<1>()) /
          dist_ij));
      r_tilde_n_vector.push_back(
        point(
          -(last_polygon_vertices_tilde[j].get<1>() -
          last_polygon_vertices_tilde[i].get<1>()) / dist_ij,
          (last_polygon_vertices_tilde[j].get<0>() - last_polygon_vertices_tilde[i].get<0>()) /
          dist_ij));
    }
    tree->back().set_r_tilde_t(r_tilde_t_vector);
    tree->back().set_r_tilde_n(r_tilde_n_vector);
  }

  // Identify all the children properties
  for (size_t i = 0; i < tree->size() - 1; i++) {
    // Compute the tangent and normal vectors of the child hyperplanes
    // r0 is always the shared edge between the parent and the child, r1 and r2 the rest in CCW order
    std::vector<point> polygon_vertices = (*tree)[i].get_vertices();
    std::vector<point> r_t_vector, r_n_vector;
    for (size_t k = 0; k < polygon_vertices.size(); k++) {
      size_t j = (k + 1) % (polygon_vertices.size());
      double dist_jk = bg::distance(polygon_vertices[k], polygon_vertices[j]);
      r_t_vector.push_back(
        point(
          (polygon_vertices[j].get<0>() - polygon_vertices[k].get<0>()) /
          dist_jk, (polygon_vertices[j].get<1>() - polygon_vertices[k].get<1>()) / dist_jk));
      r_n_vector.push_back(
        point(
          -(polygon_vertices[j].get<1>() - polygon_vertices[k].get<1>()) /
          dist_jk, (polygon_vertices[j].get<0>() - polygon_vertices[k].get<0>()) / dist_jk));
    }
    (*tree)[i].set_r_t(r_t_vector);
    (*tree)[i].set_r_n(r_n_vector);

    // Find the median from the point furthest away from the adjacency edge and from that compute the center for the purging transformation
    // To compute the center, first compute the intersection of the parent polygon with the hyperplanes of the child polygon next to the adjacency edge - this defines the admissible region within which you are allowed to search for a center
    std::vector<double> dot_product_list;
    for (size_t k = 0; k < polygon_vertices.size(); k++) {
      dot_product_list.push_back(
        (polygon_vertices[k].get<0>() - polygon_vertices[0].get<0>()) * r_n_vector[0].get<0>() +
        (polygon_vertices[k].get<1>() - polygon_vertices[0].get<1>()) * r_n_vector[0].get<1>());
    }
    size_t max_dot_product_element =
      std::distance(
      dot_product_list.begin(),
      std::max_element(dot_product_list.begin(), dot_product_list.end()));
    std::vector<point> polygon_adj_edge = (*tree)[i].get_adj_edge();
    point median_point(0.5 * (polygon_adj_edge[0].get<0>() + polygon_adj_edge[1].get<0>()),
      0.5 * (polygon_adj_edge[0].get<1>() + polygon_adj_edge[1].get<1>()));
    point median_ray(median_point.get<0>() - polygon_vertices[max_dot_product_element].get<0>(),
      median_point.get<1>() - polygon_vertices[max_dot_product_element].get<1>());
    median_ray.set<0>(median_ray.get<0>() / bg::distance(median_ray, origin));
    median_ray.set<1>(median_ray.get<1>() / bg::distance(median_ray, origin));
    std::vector<point> predecessor_polygon_vertices =
      (*tree)[(*tree)[i].get_predecessor()].get_vertices();
    predecessor_polygon_vertices.push_back(predecessor_polygon_vertices[0]);
    polygon intersect_1_cvx = cvxpolyxhplane(
      BoostPointToBoostPoly(
        predecessor_polygon_vertices), polygon_adj_edge[0], r_n_vector.back());
    polygon intersect_2_cvx = cvxpolyxhplane(intersect_1_cvx, polygon_adj_edge[1], r_n_vector[1]);
    point intersection_point =
      polyxray(
      intersect_2_cvx,
      point(
        median_point.get<0>() + 0.05 * median_ray.get<0>(),
        median_point.get<1>() + 0.05 * median_ray.get<1>()), median_ray);
    point polygon_center(0.5 * median_point.get<0>() + 0.5 * intersection_point.get<0>(),
      0.5 * median_point.get<1>() + 0.5 * intersection_point.get<1>());
    (*tree)[i].set_center(polygon_center);

    // Find the remaining tangents and normals from vertices 0 and 1 to the center
    std::vector<point> r_center_t_vector =
    {point(
        (polygon_center.get<0>() - polygon_vertices[0].get<0>()) /
        bg::distance(polygon_vertices[0], polygon_center),
        (polygon_center.get<1>() - polygon_vertices[0].get<1>()) /
        bg::distance(polygon_vertices[0], polygon_center)),
      point(
        (polygon_vertices[1].get<0>() - polygon_center.get<0>()) /
        bg::distance(polygon_vertices[1], polygon_center),
        (polygon_vertices[1].get<1>() - polygon_center.get<1>()) /
        bg::distance(polygon_vertices[1], polygon_center))};
    std::vector<point> r_center_n_vector =
    {point(-r_center_t_vector[0].get<1>(), r_center_t_vector[0].get<0>()), point(
        -r_center_t_vector[1].get<1>(), r_center_t_vector[1].get<0>())};
    (*tree)[i].set_r_center_t(r_center_t_vector);
    (*tree)[i].set_r_center_n(r_center_n_vector);

    // Compute the augmented polygon, dilate it and truncate it by the rays emanating from the center
    // Make sure that the intersection of the dilation with the convex pieces succeeding the current convex piece in the transformation does not generate a multipolygon - otherwise reduce the radius of dilation until we have a single piece
    std::vector<point> polygon_vertices_used = polygon_vertices;
    polygon_vertices_used.insert(polygon_vertices_used.begin() + 1, polygon_center);
    polygon_vertices_used.push_back(polygon_vertices_used[0]);
    polygon polygon_used = BoostPointToBoostPoly(polygon_vertices_used);
    multi_polygon polygon_used_multipolygon_dilated;
    bool multiple_elements_flag = true;
    double varepsilon_used = varepsilon;
    while (multiple_elements_flag) {
      bg::strategy::buffer::distance_symmetric<double> distance_strategy(varepsilon_used);
      bg::buffer(
        polygon_used, polygon_used_multipolygon_dilated, distance_strategy, side_strategy,
        join_strategy, end_strategy, circle_strategy);
      multi_polygon temp_result, output_union;
      std::vector<point> next_polygon = (*tree)[i + 1].get_vertices();
      next_polygon.push_back(next_polygon[0]);
      output_union.push_back(BoostPointToBoostPoly(next_polygon));
      for (size_t j = i + 2; j < tree->size(); j++) {
        std::vector<point> polygon_to_test_vertices = (*tree)[j].get_vertices();
        polygon_to_test_vertices.push_back(polygon_to_test_vertices[0]);
        bg::union_(output_union, BoostPointToBoostPoly(polygon_to_test_vertices), temp_result);
        output_union = temp_result;
      }
      multi_polygon output_intersection;
      bg::intersection(
        polygon_used_multipolygon_dilated.front(), output_union,
        output_intersection);
      if (output_intersection.size() > 1) {
        varepsilon_used = 0.5 * varepsilon_used;
      } else {
        multiple_elements_flag = false;
      }
    }
    polygon polygon_used_dilated = polygon_used_multipolygon_dilated.front();
    polygon intersect_1 = cvxpolyxhplane(
      polygon_used_dilated, polygon_center,
      (*tree)[i].get_r_center_n().front());
    polygon intersect_2 = cvxpolyxhplane(
      intersect_1, polygon_center,
      (*tree)[i].get_r_center_n().back());
    polygon candidate_polygon = intersect_2;

    // Check for collisions with all the polygons that will succeed i in the diffeomorphism construction except for its parent
    for (size_t j = i + 1; j < tree->size(); j++) {
      if (j == (*tree)[i].get_predecessor()) {
        continue;
      } else {
        std::vector<point> polygon_to_test_vertices = (*tree)[j].get_vertices();
        polygon_to_test_vertices.push_back(polygon_to_test_vertices[0]);
        multi_polygon difference_output;
        bg::difference(
          candidate_polygon, BoostPointToBoostPoly(
            polygon_to_test_vertices), difference_output);

        // If the difference operation created a multipolygon, keep only the polygon that contains the barycenter of the extended triangle
        point point_to_consider((polygon_vertices[0].get<0>() + polygon_vertices[1].get<0>() +
          polygon_center.get<0>()) / 3.0,
          (polygon_vertices[0].get<1>() + polygon_vertices[1].get<1>() + polygon_center.get<1>()) /
          3.0);
        BOOST_FOREACH(polygon const & difference_component, difference_output) {
          if (bg::within(point_to_consider, difference_component)) {
            candidate_polygon = difference_component;
            break;
          }
        }
      }
    }

    // Extract final vertices
    polygon candidate_polygon_simplified = candidate_polygon;
    bg::simplify(candidate_polygon, candidate_polygon_simplified, 0.02);
    std::vector<std::vector<double>> candidate_polygon_vertices = BoostPointToStd(
      BoostPolyToBoostPoint(
        candidate_polygon_simplified));
    candidate_polygon_vertices.pop_back();

    // Decompose the polygon into its convex pieces and find the piece that includes the barycenter of the extended triangle
    CGAL_Polygon_2 cgal_polygon;
    CGAL_Polygon_list partition_polys;
    CGAL_Traits partition_traits;
    CGAL::set_pretty_mode(std::cout);
    std::vector<point> final_polygon_vertices;
    for (size_t k = 0; k < candidate_polygon_vertices.size(); k++) {
      cgal_polygon.push_back(
        CGAL_Point_2(
          candidate_polygon_vertices[k][0],
          candidate_polygon_vertices[k][1]));
    }
    if (cgal_polygon.is_simple()) {
      // std::cout << cgal_polygon << std::endl;
      CGAL::optimal_convex_partition_2(
        cgal_polygon.vertices_begin(),
        cgal_polygon.vertices_end(),
        std::back_inserter(partition_polys),
        partition_traits);
      assert(
        CGAL::convex_partition_is_valid_2(
          cgal_polygon.vertices_begin(),
          cgal_polygon.vertices_end(),
          partition_polys.begin(),
          partition_polys.end(),
          partition_traits));
      CGAL_Point_2 cgal_point_to_consider((polygon_vertices[0].get<0>() +
        polygon_vertices[1].get<0>() + polygon_center.get<0>()) / 3.0,
        (polygon_vertices[0].get<1>() + polygon_vertices[1].get<1>() + polygon_center.get<1>()) /
        3.0);
      for (size_t k = 0; k < partition_polys.size(); k++) {
        auto it = std::next(partition_polys.begin(), k);
        CGAL_Polygon_2 polygon_to_consider = *it;
        final_polygon_vertices = {};
        for (size_t l = 0; l < polygon_to_consider.size(); l++) {
          final_polygon_vertices.push_back(
            point(
              polygon_to_consider.vertex(l).x(),
              polygon_to_consider.vertex(l).y()));
        }
        final_polygon_vertices.push_back(final_polygon_vertices[0]);
        if (bg::within(
            point(
              (polygon_vertices[0].get<0>() + polygon_vertices[1].get<0>() +
              polygon_center.get<0>()) / 3.0,
              (polygon_vertices[0].get<1>() + polygon_vertices[1].get<1>() +
              polygon_center.get<1>()) /
              3.0), BoostPointToBoostPoly(final_polygon_vertices)))
        {
          break;
        }
      }
    } else {
      final_polygon_vertices = BoostPolyToBoostPoint(intersect_2);
    }

    // Generate the outer polygonal collar
    polygon final_polygon = BoostPointToBoostPoly(final_polygon_vertices);
    multi_polygon output;
    bg::intersection(final_polygon, workspacePolygon, output);
    std::vector<point> output_vertices = BoostPolyToBoostPoint(output[0]);
    output_vertices.pop_back();
    (*tree)[i].set_vertices_tilde(output_vertices);

    // Find the tangent and normal vectors for the generated polygonal collar
    std::vector<point> r_tilde_t_vector, r_tilde_n_vector;
    for (size_t k = 0; k < output_vertices.size(); k++) {
      size_t l = (k + 1) % (output_vertices.size());
      double dist_kl = bg::distance(output_vertices[k], output_vertices[l]);
      r_tilde_t_vector.push_back(
        point(
          (output_vertices[l].get<0>() - output_vertices[k].get<0>()) /
          dist_kl, (output_vertices[l].get<1>() - output_vertices[k].get<1>()) / dist_kl));
      r_tilde_n_vector.push_back(
        point(
          -(output_vertices[l].get<1>() -
          output_vertices[k].get<1>()) / dist_kl,
          (output_vertices[l].get<0>() - output_vertices[k].get<0>()) / dist_kl));
    }
    (*tree)[i].set_r_tilde_t(r_tilde_t_vector);
    (*tree)[i].set_r_tilde_n(r_tilde_n_vector);

    // Finally, compute the augmented inner polygon that includes the center of deformation and update
    polygon_vertices.insert(polygon_vertices.begin() + 1, polygon_center);
    r_t_vector.erase(r_t_vector.begin());
    r_n_vector.erase(r_n_vector.begin());
    r_t_vector.insert(r_t_vector.begin(), r_center_t_vector[1]);
    r_t_vector.insert(r_t_vector.begin(), r_center_t_vector[0]);
    r_n_vector.insert(r_n_vector.begin(), r_center_n_vector[1]);
    r_n_vector.insert(r_n_vector.begin(), r_center_n_vector[0]);
    (*tree)[i].set_augmented_vertices(polygon_vertices);
    (*tree)[i].set_r_t(r_t_vector);
    (*tree)[i].set_r_n(r_n_vector);
  }

  return;
}


OutputStructVector polygonDiffeoTriangulation(
  std::vector<double> Position,
  std::vector<TriangleClass> DiffeoTree,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes h(x) (i.e., the position of point x in the model layer), Dh(x) and the derivatives of Dh, when purging a specific polygon whose dual graph and diffeomorphism properties are known (based on the ear clipping method)
   *
   * Input:
   *  1) Position: Point to consider
   *  2) DiffeoTree: Tree that contains the diffeomorphism properties for a particular polygon
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, jacobian and derivatives of the jacobian
   */

  // Begin purging process with default values
  OutputStructVector Output;
  std::vector<double> map_h = Position;
  std::vector<std::vector<double>> map_hd = {{1.0, 0.0}, {0.0, 1.0}};
  std::vector<double> map_hdd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Iterate through the polygon triangles
  for (size_t i = 0; i < DiffeoTree.size(); i++) {
    OutputStructVector OutputNew = triangleDiffeo(map_h, DiffeoTree[i], DiffeoParams);

    std::vector<double> map_h_new = OutputNew.Value;
    std::vector<std::vector<double>> map_hd_new = OutputNew.Jacobian;
    std::vector<double> map_hdd_new = OutputNew.JacobianD;

    double res1 = map_hd_new[0][0] * map_hdd[0] + map_hd_new[0][1] * map_hdd[4] + map_hd[0][0] *
      (map_hdd_new[0] * map_hd[0][0] + map_hdd_new[1] * map_hd[1][0]) + map_hd[1][0] *
      (map_hdd_new[2] * map_hd[0][0] + map_hdd_new[3] * map_hd[1][0]);
    double res2 = map_hd_new[0][0] * map_hdd[1] + map_hd_new[0][1] * map_hdd[5] + map_hd[0][0] *
      (map_hdd_new[0] * map_hd[0][1] + map_hdd_new[1] * map_hd[1][1]) + map_hd[1][0] *
      (map_hdd_new[2] * map_hd[0][1] + map_hdd_new[3] * map_hd[1][1]);
    double res3 = map_hd_new[0][0] * map_hdd[2] + map_hd_new[0][1] * map_hdd[6] + map_hd[0][1] *
      (map_hdd_new[0] * map_hd[0][0] + map_hdd_new[1] * map_hd[1][0]) + map_hd[1][1] *
      (map_hdd_new[2] * map_hd[0][0] + map_hdd_new[3] * map_hd[1][0]);
    double res4 = map_hd_new[0][0] * map_hdd[3] + map_hd_new[0][1] * map_hdd[7] + map_hd[0][1] *
      (map_hdd_new[0] * map_hd[0][1] + map_hdd_new[1] * map_hd[1][1]) + map_hd[1][1] *
      (map_hdd_new[2] * map_hd[0][1] + map_hdd_new[3] * map_hd[1][1]);
    double res5 = map_hd_new[1][0] * map_hdd[0] + map_hd_new[1][1] * map_hdd[4] + map_hd[0][0] *
      (map_hdd_new[4] * map_hd[0][0] + map_hdd_new[5] * map_hd[1][0]) + map_hd[1][0] *
      (map_hdd_new[6] * map_hd[0][0] + map_hdd_new[7] * map_hd[1][0]);
    double res6 = map_hd_new[1][0] * map_hdd[1] + map_hd_new[1][1] * map_hdd[5] + map_hd[0][0] *
      (map_hdd_new[4] * map_hd[0][1] + map_hdd_new[5] * map_hd[1][1]) + map_hd[1][0] *
      (map_hdd_new[6] * map_hd[0][1] + map_hdd_new[7] * map_hd[1][1]);
    double res7 = map_hd_new[1][0] * map_hdd[2] + map_hd_new[1][1] * map_hdd[6] + map_hd[0][1] *
      (map_hdd_new[4] * map_hd[0][0] + map_hdd_new[5] * map_hd[1][0]) + map_hd[1][1] *
      (map_hdd_new[6] * map_hd[0][0] + map_hdd_new[7] * map_hd[1][0]);
    double res8 = map_hd_new[1][0] * map_hdd[3] + map_hd_new[1][1] * map_hdd[7] + map_hd[0][1] *
      (map_hdd_new[4] * map_hd[0][1] + map_hdd_new[5] * map_hd[1][1]) + map_hd[1][1] *
      (map_hdd_new[6] * map_hd[0][1] + map_hdd_new[7] * map_hd[1][1]);
    map_hdd[0] = res1;
    map_hdd[1] = res2;
    map_hdd[2] = res3;
    map_hdd[3] = res4;
    map_hdd[4] = res5;
    map_hdd[5] = res6;
    map_hdd[6] = res7;
    map_hdd[7] = res8;

    map_hd = MatrixMatrixMultiplication(map_hd_new, map_hd);

    map_h = map_h_new;
  }

  // Populate output
  Output.Value = map_h;
  Output.Jacobian = map_hd;
  Output.JacobianD = map_hdd;

  return Output;
}


OutputStructVector polygonDiffeoConvex(
  std::vector<double> Position,
  std::vector<PolygonClass> DiffeoTree,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes h(x) (i.e., the position of point x in the model layer), Dh(x) and the derivatives of Dh, when purging a specific polygon whose dual graph and diffeomorphism properties are known (based on the convex decomposition method)
   *
   * Input:
   *  1) Position: Point to consider
   *  2) DiffeoTree: Tree that contains the diffeomorphism properties for a particular polygon
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, jacobian and derivatives of the jacobian
   */

  // Begin purging process with default values
  OutputStructVector Output;
  std::vector<double> map_h = Position;
  std::vector<std::vector<double>> map_hd = {{1.0, 0.0}, {0.0, 1.0}};
  std::vector<double> map_hdd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Iterate through the polygon triangles
  for (size_t i = 0; i < DiffeoTree.size(); i++) {
    OutputStructVector OutputNew = polygonDiffeo(map_h, DiffeoTree[i], DiffeoParams);

    std::vector<double> map_h_new = OutputNew.Value;
    std::vector<std::vector<double>> map_hd_new = OutputNew.Jacobian;
    std::vector<double> map_hdd_new = OutputNew.JacobianD;

    double res1 = map_hd_new[0][0] * map_hdd[0] + map_hd_new[0][1] * map_hdd[4] + map_hd[0][0] *
      (map_hdd_new[0] * map_hd[0][0] + map_hdd_new[1] * map_hd[1][0]) + map_hd[1][0] *
      (map_hdd_new[2] * map_hd[0][0] + map_hdd_new[3] * map_hd[1][0]);
    double res2 = map_hd_new[0][0] * map_hdd[1] + map_hd_new[0][1] * map_hdd[5] + map_hd[0][0] *
      (map_hdd_new[0] * map_hd[0][1] + map_hdd_new[1] * map_hd[1][1]) + map_hd[1][0] *
      (map_hdd_new[2] * map_hd[0][1] + map_hdd_new[3] * map_hd[1][1]);
    double res3 = map_hd_new[0][0] * map_hdd[2] + map_hd_new[0][1] * map_hdd[6] + map_hd[0][1] *
      (map_hdd_new[0] * map_hd[0][0] + map_hdd_new[1] * map_hd[1][0]) + map_hd[1][1] *
      (map_hdd_new[2] * map_hd[0][0] + map_hdd_new[3] * map_hd[1][0]);
    double res4 = map_hd_new[0][0] * map_hdd[3] + map_hd_new[0][1] * map_hdd[7] + map_hd[0][1] *
      (map_hdd_new[0] * map_hd[0][1] + map_hdd_new[1] * map_hd[1][1]) + map_hd[1][1] *
      (map_hdd_new[2] * map_hd[0][1] + map_hdd_new[3] * map_hd[1][1]);
    double res5 = map_hd_new[1][0] * map_hdd[0] + map_hd_new[1][1] * map_hdd[4] + map_hd[0][0] *
      (map_hdd_new[4] * map_hd[0][0] + map_hdd_new[5] * map_hd[1][0]) + map_hd[1][0] *
      (map_hdd_new[6] * map_hd[0][0] + map_hdd_new[7] * map_hd[1][0]);
    double res6 = map_hd_new[1][0] * map_hdd[1] + map_hd_new[1][1] * map_hdd[5] + map_hd[0][0] *
      (map_hdd_new[4] * map_hd[0][1] + map_hdd_new[5] * map_hd[1][1]) + map_hd[1][0] *
      (map_hdd_new[6] * map_hd[0][1] + map_hdd_new[7] * map_hd[1][1]);
    double res7 = map_hd_new[1][0] * map_hdd[2] + map_hd_new[1][1] * map_hdd[6] + map_hd[0][1] *
      (map_hdd_new[4] * map_hd[0][0] + map_hdd_new[5] * map_hd[1][0]) + map_hd[1][1] *
      (map_hdd_new[6] * map_hd[0][0] + map_hdd_new[7] * map_hd[1][0]);
    double res8 = map_hd_new[1][0] * map_hdd[3] + map_hd_new[1][1] * map_hdd[7] + map_hd[0][1] *
      (map_hdd_new[4] * map_hd[0][1] + map_hdd_new[5] * map_hd[1][1]) + map_hd[1][1] *
      (map_hdd_new[6] * map_hd[0][1] + map_hdd_new[7] * map_hd[1][1]);
    map_hdd[0] = res1;
    map_hdd[1] = res2;
    map_hdd[2] = res3;
    map_hdd[3] = res4;
    map_hdd[4] = res5;
    map_hdd[5] = res6;
    map_hdd[6] = res7;
    map_hdd[7] = res8;

    map_hd = MatrixMatrixMultiplication(map_hd_new, map_hd);

    map_h = map_h_new;
  }

  // Populate output
  Output.Value = map_h;
  Output.Jacobian = map_hd;
  Output.JacobianD = map_hdd;

  return Output;
}


OutputStructVector triangleDiffeo(
  std::vector<double> Position, TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes h(x) (i.e., the position of point x in the model layer), Dh(x) and the derivatives of Dh, when purging a specific triangle in space
   *
   * Input:
   *  1) Position: Point to consider
   *  2) Triangle: Description of the triangle
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, jacobian and derivatives of the jacobian
   */

  // Compute the triangle switch and its gradient
  OutputStructScalar switchOutput = triangleSwitch(Position, Triangle, DiffeoParams);
  double sigma = switchOutput.Value;
  std::vector<double> sigmad = switchOutput.Gradient;
  std::vector<std::vector<double>> sigmadd = switchOutput.Hessian;

  // Compute the triangle deforming factor
  OutputStructScalar deformingFactorOutput = triangleDeformingFactor(Position, Triangle);
  double nu = deformingFactorOutput.Value;
  std::vector<double> nud = deformingFactorOutput.Gradient;
  std::vector<std::vector<double>> nudd = deformingFactorOutput.Hessian;

  // Extract the center
  std::vector<double> center = {Triangle.get_center().get<0>(), Triangle.get_center().get<1>()};

  // Find the map and its jacobian
  std::vector<double> map_h =
  {sigma * (center[0] + nu * (Position[0] - center[0])) + (1 - sigma) * Position[0],
    sigma * (center[1] + nu * (Position[1] - center[1])) + (1 - sigma) * Position[1]};
  std::vector<double> PositionMinusCenter = {Position[0] - center[0], Position[1] - center[1]};
  std::vector<std::vector<double>> SigmadOuter = VectorOuterProduct(PositionMinusCenter, sigmad);
  std::vector<std::vector<double>> NudOuter = VectorOuterProduct(PositionMinusCenter, nud);
  std::vector<std::vector<double>> map_hd =
  {{(nu - 1.0) * SigmadOuter[0][0] + sigma * NudOuter[0][0] + (1.0 + sigma * (nu - 1.0)),
    (nu - 1.0) * SigmadOuter[0][1] + sigma * NudOuter[0][1]},
    {(nu - 1.0) * SigmadOuter[1][0] + sigma * NudOuter[1][0],
      (nu - 1.0) * SigmadOuter[1][1] + sigma * NudOuter[1][1] + (1.0 + sigma * (nu - 1.0))}};

  // Find the derivatives of the jacobian
  double map_hdd_m0_r0_s0 = 2.0 * sigma * nud[0] + 2.0 * (nu - 1.0) * sigmad[0] + 2.0 *
    (Position[0] - center[0]) * sigmad[0] * nud[0] + (Position[0] - center[0]) * sigma *
    nudd[0][0] + (Position[0] - center[0]) * (nu - 1.0) * sigmadd[0][0];
  double map_hdd_m0_r0_s1 = sigma * nud[1] + (nu - 1.0) * sigmad[1] + (Position[0] - center[0]) *
    sigmad[1] * nud[0] + (Position[0] - center[0]) * sigma * nudd[0][1] +
    (Position[0] - center[0]) * sigmad[0] * nud[1] + (Position[0] - center[0]) * (nu - 1.0) *
    sigmadd[0][1];
  double map_hdd_m0_r1_s0 = sigma * nud[1] + (Position[0] - center[0]) * sigmad[0] * nud[1] +
    (Position[0] - center[0]) * sigma * nudd[0][1] + (nu - 1.0) * sigmad[1] +
    (Position[0] - center[0]) * sigmad[1] * nud[0] + (Position[0] - center[0]) * (nu - 1.0) *
    sigmadd[0][1];
  double map_hdd_m0_r1_s1 = 2.0 * (Position[0] - center[0]) * sigmad[1] * nud[1] +
    (Position[0] - center[0]) * sigma * nudd[1][1] + (Position[0] - center[0]) * (nu - 1.0) *
    sigmadd[1][1];
  double map_hdd_m1_r0_s0 = 2.0 * (Position[1] - center[1]) * sigmad[0] * nud[0] +
    (Position[1] - center[1]) * sigma * nudd[0][0] + (Position[1] - center[1]) * (nu - 1.0) *
    sigmadd[0][0];
  double map_hdd_m1_r0_s1 = sigma * nud[0] + (Position[1] - center[1]) * sigmad[1] * nud[0] +
    (Position[1] - center[1]) * sigma * nudd[0][1] + (nu - 1.0) * sigmad[0] +
    (Position[1] - center[1]) * sigmad[0] * nud[1] + (Position[1] - center[1]) * (nu - 1.0) *
    sigmadd[0][1];
  double map_hdd_m1_r1_s0 = sigma * nud[0] + (nu - 1.0) * sigmad[0] + (Position[1] - center[1]) *
    sigmad[0] * nud[1] + (Position[1] - center[1]) * sigma * nudd[0][1] +
    (Position[1] - center[1]) * sigmad[1] * nud[0] + (Position[1] - center[1]) * (nu - 1.0) *
    sigmadd[0][1];
  double map_hdd_m1_r1_s1 = 2.0 * sigma * nud[1] + 2.0 * (nu - 1.0) * sigmad[1] + 2.0 *
    (Position[1] - center[1]) * sigmad[1] * nud[1] + (Position[1] - center[1]) * sigma *
    nudd[1][1] + (Position[1] - center[1]) * (nu - 1.0) * sigmadd[1][1];

  std::vector<double> map_hdd =
  {map_hdd_m0_r0_s0, map_hdd_m0_r0_s1, map_hdd_m0_r1_s0, map_hdd_m0_r1_s1, map_hdd_m1_r0_s0,
    map_hdd_m1_r0_s1, map_hdd_m1_r1_s0, map_hdd_m1_r1_s1};

  // Construct the output
  OutputStructVector Output;
  Output.Value = map_h;
  Output.Jacobian = map_hd;
  Output.JacobianD = map_hdd;

  return Output;
}


OutputStructVector polygonDiffeo(
  std::vector<double> Position, PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes h(x) (i.e., the position of point x in the model layer), Dh(x) and the derivatives of Dh, when purging a specific polygon in space
   *
   * Input:
   *  1) Position: Point to consider
   *  2) PolygonUsed: Description of the polygon
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, jacobian and derivatives of the jacobian
   */

  // Compute the polygon switch and its gradient
  OutputStructScalar switchOutput = polygonSwitch(Position, PolygonUsed, DiffeoParams);
  double sigma = switchOutput.Value;
  std::vector<double> sigmad = switchOutput.Gradient;
  std::vector<std::vector<double>> sigmadd = switchOutput.Hessian;

  // Compute the polygon deforming factor
  OutputStructScalar deformingFactorOutput = polygonDeformingFactor(Position, PolygonUsed);
  double nu = deformingFactorOutput.Value;
  std::vector<double> nud = deformingFactorOutput.Gradient;
  std::vector<std::vector<double>> nudd = deformingFactorOutput.Hessian;

  // Extract the center
  std::vector<double> center =
  {PolygonUsed.get_center().get<0>(), PolygonUsed.get_center().get<1>()};

  // Find the map and its jacobian
  std::vector<double> map_h =
  {sigma * (center[0] + nu * (Position[0] - center[0])) + (1.0 - sigma) * Position[0],
    sigma * (center[1] + nu * (Position[1] - center[1])) + (1.0 - sigma) * Position[1]};
  std::vector<double> PositionMinusCenter = {Position[0] - center[0], Position[1] - center[1]};
  std::vector<std::vector<double>> SigmadOuter = VectorOuterProduct(PositionMinusCenter, sigmad);
  std::vector<std::vector<double>> NudOuter = VectorOuterProduct(PositionMinusCenter, nud);
  std::vector<std::vector<double>> map_hd =
  {{(nu - 1.0) * SigmadOuter[0][0] + sigma * NudOuter[0][0] + (1.0 + sigma * (nu - 1.0)),
    (nu - 1.0) * SigmadOuter[0][1] + sigma * NudOuter[0][1]},
    {(nu - 1.0) * SigmadOuter[1][0] + sigma * NudOuter[1][0],
      (nu - 1.0) * SigmadOuter[1][1] + sigma * NudOuter[1][1] + (1.0 + sigma * (nu - 1.0))}};

  // Find the derivatives of the jacobian
  double map_hdd_m0_r0_s0 = 2.0 * sigma * nud[0] + 2.0 * (nu - 1.0) * sigmad[0] + 2.0 *
    (Position[0] - center[0]) * sigmad[0] * nud[0] + (Position[0] - center[0]) * sigma *
    nudd[0][0] + (Position[0] - center[0]) * (nu - 1.0) * sigmadd[0][0];
  double map_hdd_m0_r0_s1 = sigma * nud[1] + (nu - 1.0) * sigmad[1] + (Position[0] - center[0]) *
    sigmad[1] * nud[0] + (Position[0] - center[0]) * sigma * nudd[0][1] +
    (Position[0] - center[0]) * sigmad[0] * nud[1] + (Position[0] - center[0]) * (nu - 1.0) *
    sigmadd[0][1];
  double map_hdd_m0_r1_s0 = sigma * nud[1] + (Position[0] - center[0]) * sigmad[0] * nud[1] +
    (Position[0] - center[0]) * sigma * nudd[0][1] + (nu - 1.0) * sigmad[1] +
    (Position[0] - center[0]) * sigmad[1] * nud[0] + (Position[0] - center[0]) * (nu - 1.0) *
    sigmadd[0][1];
  double map_hdd_m0_r1_s1 = 2.0 * (Position[0] - center[0]) * sigmad[1] * nud[1] +
    (Position[0] - center[0]) * sigma * nudd[1][1] + (Position[0] - center[0]) * (nu - 1.0) *
    sigmadd[1][1];
  double map_hdd_m1_r0_s0 = 2.0 * (Position[1] - center[1]) * sigmad[0] * nud[0] +
    (Position[1] - center[1]) * sigma * nudd[0][0] + (Position[1] - center[1]) * (nu - 1.0) *
    sigmadd[0][0];
  double map_hdd_m1_r0_s1 = sigma * nud[0] + (Position[1] - center[1]) * sigmad[1] * nud[0] +
    (Position[1] - center[1]) * sigma * nudd[0][1] + (nu - 1.0) * sigmad[0] +
    (Position[1] - center[1]) * sigmad[0] * nud[1] + (Position[1] - center[1]) * (nu - 1.0) *
    sigmadd[0][1];
  double map_hdd_m1_r1_s0 = sigma * nud[0] + (nu - 1.0) * sigmad[0] + (Position[1] - center[1]) *
    sigmad[0] * nud[1] + (Position[1] - center[1]) * sigma * nudd[0][1] +
    (Position[1] - center[1]) * sigmad[1] * nud[0] + (Position[1] - center[1]) * (nu - 1.0) *
    sigmadd[0][1];
  double map_hdd_m1_r1_s1 = 2.0 * sigma * nud[1] + 2.0 * (nu - 1.0) * sigmad[1] + 2.0 *
    (Position[1] - center[1]) * sigmad[1] * nud[1] + (Position[1] - center[1]) * sigma *
    nudd[1][1] + (Position[1] - center[1]) * (nu - 1.0) * sigmadd[1][1];

  std::vector<double> map_hdd =
  {map_hdd_m0_r0_s0, map_hdd_m0_r0_s1, map_hdd_m0_r1_s0, map_hdd_m0_r1_s1, map_hdd_m1_r0_s0,
    map_hdd_m1_r0_s1, map_hdd_m1_r1_s0, map_hdd_m1_r1_s1};

  // Construct the output
  OutputStructVector Output;
  Output.Value = map_h;
  Output.Jacobian = map_hd;
  Output.JacobianD = map_hdd;

  return Output;
}


OutputStructScalar triangleSwitch(
  std::vector<double> Position, TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes the overall switch value, its gradient and hessian for a point x outside a triangle
   *
   * Input:
   *  1) Position: Point to consider
   *  2) Triangle: Description of the triangle
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Find the separate switch values, gradients and hessians
  OutputStructScalar BetaOutput = triangleBetaSwitch(Position, Triangle, DiffeoParams);
  OutputStructScalar GammaOutput = triangleGammaSwitch(Position, Triangle, DiffeoParams);

  // Unwrap values
  double sigma_beta = BetaOutput.Value;
  std::vector<double> sigma_betad = BetaOutput.Gradient;
  std::vector<std::vector<double>> sigma_betadd = BetaOutput.Hessian;
  double sigma_gamma = GammaOutput.Value;
  std::vector<double> sigma_gammad = GammaOutput.Gradient;
  std::vector<std::vector<double>> sigma_gammadd = GammaOutput.Hessian;

  // Find the overall switch value gradient and hessian
  double sigma;
  std::vector<double> sigmad;
  std::vector<std::vector<double>> sigmadd;
  if ((sigma_beta == 1.0) && (sigma_gamma == 0.0)) {
    sigma = 1.0;
    sigmad = {0.0, 0.0};
    sigmadd = {{0.0, 0.0}, {0.0, 0.0}};
  } else {
    double nom = sigma_beta * sigma_gamma;
    double denom = sigma_beta * sigma_gamma + (1 - sigma_beta);
    sigma = nom / denom;

    std::vector<double> nomd =
    {sigma_gamma * sigma_betad[0] + sigma_beta * sigma_gammad[0],
      sigma_gamma * sigma_betad[1] + sigma_beta * sigma_gammad[1]};
    std::vector<double> denomd =
    {sigma_gamma * sigma_betad[0] + sigma_beta * sigma_gammad[0] - sigma_betad[0],
      sigma_gamma * sigma_betad[1] + sigma_beta * sigma_gammad[1] - sigma_betad[1]};
    sigmad =
    {(1.0 / denom) * nomd[0] - (nom / pow(denom, 2.0)) * denomd[0],
      (1.0 / denom) * nomd[1] - (nom / pow(denom, 2.0)) * denomd[1]};

    std::vector<std::vector<double>> BetaGammaOuter = VectorOuterProduct(sigma_betad, sigma_gammad);
    std::vector<std::vector<double>> GammaBetaOuter = VectorOuterProduct(sigma_gammad, sigma_betad);
    std::vector<std::vector<double>> nomdd =
    {{sigma_gamma * sigma_betadd[0][0] + BetaGammaOuter[0][0] + GammaBetaOuter[0][0] +
      sigma_beta * sigma_gammadd[0][0],
      sigma_gamma * sigma_betadd[0][1] + BetaGammaOuter[0][1] + GammaBetaOuter[0][1] +
      sigma_beta * sigma_gammadd[0][1]},
      {sigma_gamma * sigma_betadd[1][0] + BetaGammaOuter[1][0] + GammaBetaOuter[1][0] +
        sigma_beta * sigma_gammadd[1][0],
        sigma_gamma * sigma_betadd[1][1] + BetaGammaOuter[1][1] + GammaBetaOuter[1][1] +
        sigma_beta * sigma_gammadd[1][1]}};
    std::vector<std::vector<double>> denomdd =
    {{sigma_gamma * sigma_betadd[0][0] + BetaGammaOuter[0][0] + GammaBetaOuter[0][0] +
      sigma_beta * sigma_gammadd[0][0] - sigma_betadd[0][0],
      sigma_gamma * sigma_betadd[0][1] + BetaGammaOuter[0][1] + GammaBetaOuter[0][1] +
      sigma_beta * sigma_gammadd[0][1] - sigma_betadd[0][1]},
      {sigma_gamma * sigma_betadd[1][0] + BetaGammaOuter[1][0] + GammaBetaOuter[1][0] +
        sigma_beta * sigma_gammadd[1][0] - sigma_betadd[1][0],
        sigma_gamma * sigma_betadd[1][1] + BetaGammaOuter[1][1] + GammaBetaOuter[1][1] +
        sigma_beta * sigma_gammadd[1][1] - sigma_betadd[1][1]}};
    std::vector<std::vector<double>> NomDenomOuter = VectorOuterProduct(nomd, denomd);
    std::vector<std::vector<double>> DenomNomOuter = VectorOuterProduct(denomd, nomd);
    std::vector<std::vector<double>> DenomDenomOuter = VectorOuterProduct(denomd, denomd);
    sigmadd =
    {{(1.0 / denom) * nomdd[0][0] -
      (1.0 /
      pow(
        denom,
        2.0)) * (NomDenomOuter[0][0] + DenomNomOuter[0][0]) + 2.0 *
      (nom / pow(denom, 3.0)) * DenomDenomOuter[0][0] - (nom / pow(denom, 2.0)) * denomdd[0][0],
      (1.0 / denom) * nomdd[0][1] -
      (1.0 /
      pow(
        denom,
        2.0)) * (NomDenomOuter[0][1] + DenomNomOuter[0][1]) + 2.0 *
      (nom /
      pow(denom, 3.0)) * DenomDenomOuter[0][1] - (nom / pow(denom, 2.0)) * denomdd[0][1]},
      {(1.0 / denom) * nomdd[1][0] -
        (1.0 /
        pow(
          denom,
          2.0)) * (NomDenomOuter[1][0] + DenomNomOuter[1][0]) + 2.0 *
        (nom / pow(denom, 3.0)) * DenomDenomOuter[1][0] - (nom / pow(denom, 2.0)) * denomdd[1][0],
        (1.0 / denom) * nomdd[1][1] -
        (1.0 /
        pow(
          denom,
          2.0)) * (NomDenomOuter[1][1] + DenomNomOuter[1][1]) + 2.0 *
        (nom /
        pow(denom, 3.0)) * DenomDenomOuter[1][1] - (nom / pow(denom, 2.0)) * denomdd[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = sigma;
  Output.Gradient = sigmad;
  Output.Hessian = sigmadd;

  return Output;
}


OutputStructScalar polygonSwitch(
  std::vector<double> Position, PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes the overall switch value, its gradient and hessian for a point x outside a polygon
   *
   * Input:
   *  1) Position: Point to consider
   *  2) PolygonUsed: Description of the polygon
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Find the separate switch values, gradients and hessians
  OutputStructScalar BetaOutput = polygonBetaSwitch(Position, PolygonUsed, DiffeoParams);
  OutputStructScalar GammaOutput = polygonGammaSwitch(Position, PolygonUsed, DiffeoParams);

  // Unwrap values
  double sigma_beta = BetaOutput.Value;
  std::vector<double> sigma_betad = BetaOutput.Gradient;
  std::vector<std::vector<double>> sigma_betadd = BetaOutput.Hessian;
  double sigma_gamma = GammaOutput.Value;
  std::vector<double> sigma_gammad = GammaOutput.Gradient;
  std::vector<std::vector<double>> sigma_gammadd = GammaOutput.Hessian;

  // Find the overall switch value gradient and hessian
  double sigma;
  std::vector<double> sigmad;
  std::vector<std::vector<double>> sigmadd;
  if ((sigma_beta == 1.0) && (sigma_gamma == 0.0)) {
    sigma = 1.0;
    sigmad = {0.0, 0.0};
    sigmadd = {{0.0, 0.0}, {0.0, 0.0}};
  } else {
    double nom = sigma_beta * sigma_gamma;
    double denom = sigma_beta * sigma_gamma + (1 - sigma_beta);
    sigma = nom / denom;

    std::vector<double> nomd =
    {sigma_gamma * sigma_betad[0] + sigma_beta * sigma_gammad[0],
      sigma_gamma * sigma_betad[1] + sigma_beta * sigma_gammad[1]};
    std::vector<double> denomd =
    {sigma_gamma * sigma_betad[0] + sigma_beta * sigma_gammad[0] - sigma_betad[0],
      sigma_gamma * sigma_betad[1] + sigma_beta * sigma_gammad[1] - sigma_betad[1]};
    sigmad =
    {(1.0 / denom) * nomd[0] - (nom / pow(denom, 2.0)) * denomd[0],
      (1.0 / denom) * nomd[1] - (nom / pow(denom, 2.0)) * denomd[1]};

    std::vector<std::vector<double>> BetaGammaOuter = VectorOuterProduct(sigma_betad, sigma_gammad);
    std::vector<std::vector<double>> GammaBetaOuter = VectorOuterProduct(sigma_gammad, sigma_betad);
    std::vector<std::vector<double>> nomdd =
    {{sigma_gamma * sigma_betadd[0][0] + BetaGammaOuter[0][0] + GammaBetaOuter[0][0] +
      sigma_beta * sigma_gammadd[0][0],
      sigma_gamma * sigma_betadd[0][1] + BetaGammaOuter[0][1] + GammaBetaOuter[0][1] +
      sigma_beta * sigma_gammadd[0][1]},
      {sigma_gamma * sigma_betadd[1][0] + BetaGammaOuter[1][0] + GammaBetaOuter[1][0] +
        sigma_beta * sigma_gammadd[1][0],
        sigma_gamma * sigma_betadd[1][1] + BetaGammaOuter[1][1] + GammaBetaOuter[1][1] +
        sigma_beta * sigma_gammadd[1][1]}};
    std::vector<std::vector<double>> denomdd =
    {{sigma_gamma * sigma_betadd[0][0] + BetaGammaOuter[0][0] + GammaBetaOuter[0][0] +
      sigma_beta * sigma_gammadd[0][0] - sigma_betadd[0][0],
      sigma_gamma * sigma_betadd[0][1] + BetaGammaOuter[0][1] + GammaBetaOuter[0][1] +
      sigma_beta * sigma_gammadd[0][1] - sigma_betadd[0][1]},
      {sigma_gamma * sigma_betadd[1][0] + BetaGammaOuter[1][0] + GammaBetaOuter[1][0] +
        sigma_beta * sigma_gammadd[1][0] - sigma_betadd[1][0],
        sigma_gamma * sigma_betadd[1][1] + BetaGammaOuter[1][1] + GammaBetaOuter[1][1] +
        sigma_beta * sigma_gammadd[1][1] - sigma_betadd[1][1]}};
    std::vector<std::vector<double>> NomDenomOuter = VectorOuterProduct(nomd, denomd);
    std::vector<std::vector<double>> DenomNomOuter = VectorOuterProduct(denomd, nomd);
    std::vector<std::vector<double>> DenomDenomOuter = VectorOuterProduct(denomd, denomd);
    sigmadd =
    {{(1.0 / denom) * nomdd[0][0] -
      (1.0 /
      pow(
        denom,
        2.0)) * (NomDenomOuter[0][0] + DenomNomOuter[0][0]) + 2.0 *
      (nom / pow(denom, 3.0)) * DenomDenomOuter[0][0] - (nom / pow(denom, 2.0)) * denomdd[0][0],
      (1.0 / denom) * nomdd[0][1] -
      (1.0 /
      pow(
        denom,
        2.0)) * (NomDenomOuter[0][1] + DenomNomOuter[0][1]) + 2.0 *
      (nom /
      pow(denom, 3.0)) * DenomDenomOuter[0][1] - (nom / pow(denom, 2.0)) * denomdd[0][1]},
      {(1.0 / denom) * nomdd[1][0] -
        (1.0 /
        pow(
          denom,
          2.0)) * (NomDenomOuter[1][0] + DenomNomOuter[1][0]) + 2.0 *
        (nom / pow(denom, 3.0)) * DenomDenomOuter[1][0] - (nom / pow(denom, 2.0)) * denomdd[1][0],
        (1.0 / denom) * nomdd[1][1] -
        (1.0 /
        pow(
          denom,
          2.0)) * (NomDenomOuter[1][1] + DenomNomOuter[1][1]) + 2.0 *
        (nom /
        pow(denom, 3.0)) * DenomDenomOuter[1][1] - (nom / pow(denom, 2.0)) * denomdd[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = sigma;
  Output.Gradient = sigmad;
  Output.Hessian = sigmadd;

  return Output;
}


OutputStructScalar triangleDeformingFactor(std::vector<double> Position, TriangleClass Triangle)
{
  /**
   * Function that computes the value, gradient and hessian of the deforming factor for a point x outside a triangle
   *
   * Input:
   *  1) Position: Point to consider
   *  2) Triangle: Description of the triangle
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Make position a point
  point PositionPoint(Position[0], Position[1]);

  // Unwrap triangle properties
  uint16_t depth = Triangle.get_depth();
  std::vector<point> adj_edge = Triangle.get_adj_edge();
  double radius = Triangle.get_radius();
  point CenterPoint = Triangle.get_center();
  std::vector<double> Center = {CenterPoint.get<0>(), CenterPoint.get<1>()};

  // Get triangle vertices and normal vectors and convert them
  std::vector<point> TriangleVertexListPoint = Triangle.get_vertices();
  std::vector<point> NormalVectorListPoint = Triangle.get_r_n();
  std::vector<std::vector<double>> TriangleVertexList;
  std::vector<std::vector<double>> NormalVectorList;
  for (size_t i = 0; i < Triangle.get_vertices().size(); i++) {
    TriangleVertexList.push_back(
      {TriangleVertexListPoint[i].get<0>(), TriangleVertexListPoint[i].get<1>()});
    NormalVectorList.push_back(
      {NormalVectorListPoint[i].get<0>(),
        NormalVectorListPoint[i].get<1>()});
  }

  // Distinguish whether the triangle to consider is the root or some child
  double nu;
  std::vector<double> nud;
  std::vector<std::vector<double>> nudd;
  if ((depth == 0) && adj_edge.empty()) {
    nu = radius / bg::distance(PositionPoint, CenterPoint);
    nud =
    {-(radius / pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * (Position[0] - Center[0]),
      -(radius / pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * (Position[1] - Center[1])};
    std::vector<double> PositionMinusCenter = {Position[0] - Center[0], Position[1] - Center[1]};
    std::vector<std::vector<double>> PositionPositionOuter = VectorOuterProduct(
      PositionMinusCenter,
      PositionMinusCenter);
    nudd =
    {{((3.0 * radius) /
      pow(
        bg::distance(PositionPoint, CenterPoint),
        5.0)) * PositionPositionOuter[0][0] -
      (radius / pow(bg::distance(PositionPoint, CenterPoint), 3.0)),
      ((3.0 * radius) /
      pow(bg::distance(PositionPoint, CenterPoint), 5.0)) * PositionPositionOuter[0][1]},
      {((3.0 * radius) /
        pow(bg::distance(PositionPoint, CenterPoint), 5.0)) * PositionPositionOuter[1][0],
        ((3.0 * radius) /
        pow(
          bg::distance(PositionPoint, CenterPoint),
          5.0)) * PositionPositionOuter[1][1] -
        (radius / pow(bg::distance(PositionPoint, CenterPoint), 3.0))}};
  } else {
    nu =
      ((TriangleVertexList[0][0] - Center[0]) * NormalVectorList[0][0] +
      (TriangleVertexList[0][1] - Center[1]) * NormalVectorList[0][1]) /
      ((Position[0] - Center[0]) * NormalVectorList[0][0] + (Position[1] - Center[1]) *
      NormalVectorList[0][1]);
    nud =
    {-(((TriangleVertexList[0][0] - Center[0]) * NormalVectorList[0][0] +
      (TriangleVertexList[0][1] - Center[1]) * NormalVectorList[0][1]) /
      pow(
        (Position[0] - Center[0]) * NormalVectorList[0][0] + (Position[1] - Center[1]) *
        NormalVectorList[0][1], 2.0)) * NormalVectorList[0][0],
      -(((TriangleVertexList[0][0] - Center[0]) * NormalVectorList[0][0] +
      (TriangleVertexList[0][1] - Center[1]) * NormalVectorList[0][1]) /
      pow(
        (Position[0] - Center[0]) * NormalVectorList[0][0] + (Position[1] - Center[1]) *
        NormalVectorList[0][1], 2.0)) * NormalVectorList[0][1]};
    std::vector<double> NormalVector = {NormalVectorList[0][0], NormalVectorList[0][1]};
    std::vector<std::vector<double>> NormalVectorOuter = VectorOuterProduct(
      NormalVector,
      NormalVector);
    nudd =
    {{2.0 *
      (((TriangleVertexList[0][0] - Center[0]) * NormalVectorList[0][0] +
      (TriangleVertexList[0][1] - Center[1]) * NormalVectorList[0][1]) /
      pow(
        (Position[0] - Center[0]) * NormalVectorList[0][0] + (Position[1] - Center[1]) *
        NormalVectorList[0][1], 3.0)) * NormalVectorOuter[0][0],
      2.0 *
      (((TriangleVertexList[0][0] - Center[0]) * NormalVectorList[0][0] +
      (TriangleVertexList[0][1] - Center[1]) * NormalVectorList[0][1]) /
      pow(
        (Position[0] - Center[0]) * NormalVectorList[0][0] + (Position[1] - Center[1]) *
        NormalVectorList[0][1], 3.0)) * NormalVectorOuter[0][1]},
      {2.0 *
        (((TriangleVertexList[0][0] - Center[0]) * NormalVectorList[0][0] +
        (TriangleVertexList[0][1] - Center[1]) * NormalVectorList[0][1]) /
        pow(
          (Position[0] - Center[0]) * NormalVectorList[0][0] + (Position[1] - Center[1]) *
          NormalVectorList[0][1], 3.0)) * NormalVectorOuter[1][0],
        2.0 *
        (((TriangleVertexList[0][0] - Center[0]) * NormalVectorList[0][0] +
        (TriangleVertexList[0][1] - Center[1]) * NormalVectorList[0][1]) /
        pow(
          (Position[0] - Center[0]) * NormalVectorList[0][0] + (Position[1] - Center[1]) *
          NormalVectorList[0][1], 3.0)) * NormalVectorOuter[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = nu;
  Output.Gradient = nud;
  Output.Hessian = nudd;

  return Output;
}


OutputStructScalar polygonDeformingFactor(std::vector<double> Position, PolygonClass PolygonUsed)
{
  /**
   * Function that computes the value, gradient and hessian of the deforming factor for a point x outside a polygon
   *
   * Input:
   *  1) Position: Point to consider
   *  2) PolygonUsed: Description of the polygon
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Make position a point
  point PositionPoint(Position[0], Position[1]);

  // Unwrap polygon properties
  uint16_t depth = PolygonUsed.get_depth();
  std::vector<point> adj_edge = PolygonUsed.get_adj_edge();
  double radius = PolygonUsed.get_radius();
  point CenterPoint = PolygonUsed.get_center();
  std::vector<double> Center = {CenterPoint.get<0>(), CenterPoint.get<1>()};

  // Get polygon vertices and normal vectors and convert them
  std::vector<point> PolygonVertexListPoint = PolygonUsed.get_augmented_vertices();
  std::vector<point> NormalVectorListPoint = PolygonUsed.get_r_n();
  std::vector<std::vector<double>> PolygonVertexList;
  for (size_t i = 0; i < PolygonUsed.get_augmented_vertices().size(); i++) {
    PolygonVertexList.push_back(
      {PolygonVertexListPoint[i].get<0>(), PolygonVertexListPoint[i].get<1>()});
  }

  // Distinguish whether the polygon to consider is the root or some child
  double nu;
  std::vector<double> nud;
  std::vector<std::vector<double>> nudd;
  if ((depth == 0) && adj_edge.empty()) {
    nu = radius / bg::distance(PositionPoint, CenterPoint);
    nud =
    {-(radius / pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * (Position[0] - Center[0]),
      -(radius / pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * (Position[1] - Center[1])};
    std::vector<double> PositionMinusCenter = {Position[0] - Center[0], Position[1] - Center[1]};
    std::vector<std::vector<double>> PositionPositionOuter = VectorOuterProduct(
      PositionMinusCenter,
      PositionMinusCenter);
    nudd =
    {{((3.0 * radius) /
      pow(
        bg::distance(PositionPoint, CenterPoint),
        5.0)) * PositionPositionOuter[0][0] -
      (radius / pow(bg::distance(PositionPoint, CenterPoint), 3.0)),
      ((3.0 * radius) /
      pow(bg::distance(PositionPoint, CenterPoint), 5.0)) * PositionPositionOuter[0][1]},
      {((3.0 * radius) /
        pow(bg::distance(PositionPoint, CenterPoint), 5.0)) * PositionPositionOuter[1][0],
        ((3.0 * radius) /
        pow(
          bg::distance(PositionPoint, CenterPoint),
          5.0)) * PositionPositionOuter[1][1] -
        (radius / pow(bg::distance(PositionPoint, CenterPoint), 3.0))}};
  } else {
    std::vector<double> adj_edge_tangent =
    {(adj_edge[1].get<0>() - adj_edge[0].get<0>()) / bg::distance(adj_edge[0], adj_edge[1]),
      (adj_edge[1].get<1>() - adj_edge[0].get<1>()) / bg::distance(adj_edge[0], adj_edge[1])};
    std::vector<double> adj_edge_normal = {-adj_edge_tangent[1], adj_edge_tangent[0]};
    nu =
      ((PolygonVertexList[0][0] - Center[0]) * adj_edge_normal[0] +
      (PolygonVertexList[0][1] - Center[1]) * adj_edge_normal[1]) /
      ((Position[0] - Center[0]) * adj_edge_normal[0] + (Position[1] - Center[1]) *
      adj_edge_normal[1]);
    nud =
    {-(((PolygonVertexList[0][0] - Center[0]) * adj_edge_normal[0] +
      (PolygonVertexList[0][1] - Center[1]) * adj_edge_normal[1]) /
      pow(
        (Position[0] - Center[0]) * adj_edge_normal[0] + (Position[1] - Center[1]) *
        adj_edge_normal[1], 2.0)) * adj_edge_normal[0],
      -(((PolygonVertexList[0][0] - Center[0]) * adj_edge_normal[0] +
      (PolygonVertexList[0][1] - Center[1]) * adj_edge_normal[1]) /
      pow(
        (Position[0] - Center[0]) * adj_edge_normal[0] + (Position[1] - Center[1]) *
        adj_edge_normal[1], 2.0)) * adj_edge_normal[1]};
    std::vector<std::vector<double>> NormalVectorOuter = VectorOuterProduct(
      adj_edge_normal,
      adj_edge_normal);
    nudd =
    {{2.0 *
      (((PolygonVertexList[0][0] - Center[0]) * adj_edge_normal[0] +
      (PolygonVertexList[0][1] - Center[1]) * adj_edge_normal[1]) /
      pow(
        (Position[0] - Center[0]) * adj_edge_normal[0] + (Position[1] - Center[1]) *
        adj_edge_normal[1], 3.0)) * NormalVectorOuter[0][0],
      2.0 *
      (((PolygonVertexList[0][0] - Center[0]) * adj_edge_normal[0] +
      (PolygonVertexList[0][1] - Center[1]) * adj_edge_normal[1]) /
      pow(
        (Position[0] - Center[0]) * adj_edge_normal[0] + (Position[1] - Center[1]) *
        adj_edge_normal[1], 3.0)) * NormalVectorOuter[0][1]},
      {2.0 *
        (((PolygonVertexList[0][0] - Center[0]) * adj_edge_normal[0] +
        (PolygonVertexList[0][1] - Center[1]) * adj_edge_normal[1]) /
        pow(
          (Position[0] - Center[0]) * adj_edge_normal[0] + (Position[1] - Center[1]) *
          adj_edge_normal[1], 3.0)) * NormalVectorOuter[1][0],
        2.0 *
        (((PolygonVertexList[0][0] - Center[0]) * adj_edge_normal[0] +
        (PolygonVertexList[0][1] - Center[1]) * adj_edge_normal[1]) /
        pow(
          (Position[0] - Center[0]) * adj_edge_normal[0] + (Position[1] - Center[1]) *
          adj_edge_normal[1], 3.0)) * NormalVectorOuter[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = nu;
  Output.Gradient = nud;
  Output.Hessian = nudd;

  return Output;
}


OutputStructScalar triangleBetaSwitch(
  std::vector<double> Position, TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes the value, gradient and hessian of the beta-switch for a point x outside a triangle
   *
   * Input:
   *  1) Position: Point to consider
   *  2) Triangle: Description of the triangle
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Unwrap parameters
  double mu_1 = DiffeoParams.get_mu_1();
  double epsilon = DiffeoParams.get_epsilon();

  // Compute the value of beta and its gradient and hessian
  OutputStructScalar BetaOutput = triangleOutsideImplicit(Position, Triangle, DiffeoParams);
  double beta = BetaOutput.Value;
  std::vector<double> betad = BetaOutput.Gradient;
  std::vector<std::vector<double>> betadd = BetaOutput.Hessian;

  // Compute the value of the switch
  double sigma;
  std::vector<double> sigmad;
  std::vector<std::vector<double>> sigmadd;
  if (beta >= epsilon - 1e-3) {
    sigma = 0.0;
    sigmad = {0.0, 0.0};
    sigmadd = {{0.0, 0.0}, {0.0, 0.0}};
  } else {
    sigma = exp(-mu_1 / (epsilon - beta)) / exp(-mu_1 / epsilon);
    sigmad =
    {-mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betad[0],
      -mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betad[1]};
    std::vector<std::vector<double>> BetadOuter = VectorOuterProduct(betad, betad);
    sigmadd =
    {{((pow(
        mu_1,
        2.0) *
      (sigma /
      pow(
        epsilon - beta,
        4.0))) - 2.0 * mu_1 *
      (sigma /
      pow(
        epsilon - beta,
        3.0))) * BetadOuter[0][0] - mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betadd[0][0],
      ((pow(
        mu_1,
        2.0) *
      (sigma /
      pow(
        epsilon - beta,
        4.0))) - 2.0 * mu_1 *
      (sigma /
      pow(
        epsilon - beta,
        3.0))) * BetadOuter[0][1] - mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betadd[0][1]},
      {((pow(
          mu_1,
          2.0) *
        (sigma /
        pow(
          epsilon - beta,
          4.0))) - 2.0 * mu_1 *
        (sigma /
        pow(
          epsilon - beta,
          3.0))) * BetadOuter[1][0] - mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betadd[1][0],
        ((pow(
          mu_1,
          2.0) *
        (sigma /
        pow(
          epsilon - beta,
          4.0))) - 2.0 * mu_1 *
        (sigma /
        pow(
          epsilon - beta,
          3.0))) * BetadOuter[1][1] - mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betadd[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = sigma;
  Output.Gradient = sigmad;
  Output.Hessian = sigmadd;

  return Output;
}


OutputStructScalar polygonBetaSwitch(
  std::vector<double> Position, PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes the value, gradient and hessian of the beta-switch for a point x outside a polygon
   *
   * Input:
   *  1) Position: Point to consider
   *  2) PolygonUsed: Description of the polygon
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Unwrap parameters
  double mu_1 = DiffeoParams.get_mu_1();
  double epsilon = DiffeoParams.get_epsilon();

  // Compute the value of beta and its gradient and hessian
  OutputStructScalar BetaOutput = polygonOutsideImplicit(Position, PolygonUsed, DiffeoParams);
  double beta = BetaOutput.Value;
  std::vector<double> betad = BetaOutput.Gradient;
  std::vector<std::vector<double>> betadd = BetaOutput.Hessian;

  // Compute the value of the switch
  double sigma;
  std::vector<double> sigmad;
  std::vector<std::vector<double>> sigmadd;
  if (beta >= epsilon) {
    sigma = 0.0;
    sigmad = {0.0, 0.0};
    sigmadd = {{0.0, 0.0}, {0.0, 0.0}};
  } else {
    sigma = exp(-mu_1 / (epsilon - beta)) / exp(-mu_1 / epsilon);
    sigmad =
    {-mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betad[0],
      -mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betad[1]};
    std::vector<std::vector<double>> BetadOuter = VectorOuterProduct(betad, betad);
    sigmadd =
    {{((pow(
        mu_1,
        2.0) *
      (sigma /
      pow(
        epsilon - beta,
        4.0))) - 2.0 * mu_1 *
      (sigma /
      pow(
        epsilon - beta,
        3.0))) * BetadOuter[0][0] - mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betadd[0][0],
      ((pow(
        mu_1,
        2.0) *
      (sigma /
      pow(
        epsilon - beta,
        4.0))) - 2.0 * mu_1 *
      (sigma /
      pow(
        epsilon - beta,
        3.0))) * BetadOuter[0][1] - mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betadd[0][1]},
      {((pow(
          mu_1,
          2.0) *
        (sigma /
        pow(
          epsilon - beta,
          4.0))) - 2.0 * mu_1 *
        (sigma /
        pow(
          epsilon - beta,
          3.0))) * BetadOuter[1][0] - mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betadd[1][0],
        ((pow(
          mu_1,
          2.0) *
        (sigma /
        pow(
          epsilon - beta,
          4.0))) - 2.0 * mu_1 *
        (sigma /
        pow(
          epsilon - beta,
          3.0))) * BetadOuter[1][1] - mu_1 * (sigma / pow(epsilon - beta, 2.0)) * betadd[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = sigma;
  Output.Gradient = sigmad;
  Output.Hessian = sigmadd;

  return Output;
}


OutputStructScalar triangleGammaSwitch(
  std::vector<double> Position, TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes the value, gradient and hessian of the gamma-switch for a point x outside a triangle
   *
   * Input:
   *  1) Position: Point to consider
   *  2) Triangle: Description of the triangle
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Unwrap parameters
  double mu_2 = DiffeoParams.get_mu_2();

  // Make position a point
  point PositionPoint(Position[0], Position[1]);

  // Unwrap center properties
  point CenterPoint = Triangle.get_center();
  std::vector<double> Center = {CenterPoint.get<0>(), CenterPoint.get<1>()};

  // Compute the value of gamma and its gradient and hessian
  OutputStructScalar GammaOutput = triangleInsideImplicit(Position, Triangle, DiffeoParams);
  double gamma = GammaOutput.Value;
  std::vector<double> gammad = GammaOutput.Gradient;
  std::vector<std::vector<double>> gammadd = GammaOutput.Hessian;

  // Compute the value of the switch
  double sigma;
  std::vector<double> sigmad;
  std::vector<std::vector<double>> sigmadd;
  if (gamma <= 0.0) {
    sigma = 0.0;
    sigmad = {0.0, 0.0};
    sigmadd = {{0.0, 0.0}, {0.0, 0.0}};
  } else {
    // Compute the value of alpha and its gradient and hessian
    double nom = gamma;
    double denom = bg::distance(PositionPoint, CenterPoint);
    double alpha = nom / denom;

    std::vector<double> nomd = gammad;
    std::vector<double> denomd =
    {(1.0 / bg::distance(PositionPoint, CenterPoint)) * (Position[0] - Center[0]),
      (1.0 / bg::distance(PositionPoint, CenterPoint)) * (Position[1] - Center[1])};
    std::vector<double> alphad =
    {(1.0 / denom) * nomd[0] - (nom / pow(denom, 2.0)) * denomd[0],
      (1.0 / denom) * nomd[1] - (nom / pow(denom, 2.0)) * denomd[1]};

    std::vector<std::vector<double>> nomdd = gammadd;
    std::vector<double> PositionMinusCenter = {Position[0] - Center[0], Position[1] - Center[1]};
    std::vector<std::vector<double>> PositionPositionOuter = VectorOuterProduct(
      PositionMinusCenter,
      PositionMinusCenter);
    std::vector<std::vector<double>> denomdd =
    {{(1.0 /
      bg::distance(
        PositionPoint,
        CenterPoint)) -
      (1.0 / pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * PositionPositionOuter[0][0],
      -(1.0 /
      pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * PositionPositionOuter[0][1]},
      {-(1.0 / pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * PositionPositionOuter[1][0],
        (1.0 /
        bg::distance(
          PositionPoint,
          CenterPoint)) -
        (1.0 /
        pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * PositionPositionOuter[1][1]}};
    std::vector<std::vector<double>> NomDenomOuter = VectorOuterProduct(nomd, denomd);
    std::vector<std::vector<double>> DenomNomOuter = VectorOuterProduct(denomd, nomd);
    std::vector<std::vector<double>> DenomDenomOuter = VectorOuterProduct(denomd, denomd);
    std::vector<std::vector<double>> alphadd = {{(1.0 / denom) * nomdd[0][0] - (1.0 / pow(
        denom,
        2.0)) *
      (NomDenomOuter[0][0] + DenomNomOuter[0][0]) + 2.0 *
      (nom / pow(denom, 3.0)) * DenomDenomOuter[0][0] - (nom / pow(denom, 2.0)) * denomdd[0][0],
      (1.0 / denom) * nomdd[0][1] -
      (1.0 /
      pow(
        denom,
        2.0)) * (NomDenomOuter[0][1] + DenomNomOuter[0][1]) + 2.0 *
      (nom /
      pow(denom, 3.0)) * DenomDenomOuter[0][1] - (nom / pow(denom, 2.0)) * denomdd[0][1]},
      {(1.0 / denom) * nomdd[1][0] -
        (1.0 /
        pow(
          denom,
          2.0)) * (NomDenomOuter[1][0] + DenomNomOuter[1][0]) + 2.0 *
        (nom / pow(denom, 3.0)) * DenomDenomOuter[1][0] - (nom / pow(denom, 2.0)) * denomdd[1][0],
        (1.0 / denom) * nomdd[1][1] -
        (1.0 /
        pow(
          denom,
          2.0)) * (NomDenomOuter[1][1] + DenomNomOuter[1][1]) + 2.0 *
        (nom /
        pow(denom, 3.0)) * DenomDenomOuter[1][1] - (nom / pow(denom, 2.0)) * denomdd[1][1]}};

    sigma = exp(-mu_2 / alpha);
    sigmad =
    {mu_2 * (sigma / pow(alpha, 2.0)) * alphad[0], mu_2 * (sigma / pow(alpha, 2.0)) * alphad[1]};
    std::vector<std::vector<double>> AlphadOuter = VectorOuterProduct(alphad, alphad);
    sigmadd =
    {{(pow(
        mu_2,
        2.0) *
      (sigma /
      pow(
        alpha,
        4.0)) - 2.0 * mu_2 *
      (sigma /
      pow(alpha, 3.0))) * AlphadOuter[0][0] + mu_2 * (sigma / pow(alpha, 2.0)) * alphadd[0][0],
      (pow(
        mu_2,
        2.0) *
      (sigma /
      pow(
        alpha,
        4.0)) - 2.0 * mu_2 *
      (sigma /
      pow(alpha, 3.0))) * AlphadOuter[0][1] + mu_2 * (sigma / pow(alpha, 2.0)) * alphadd[0][1]},
      {(pow(
          mu_2,
          2.0) *
        (sigma /
        pow(
          alpha,
          4.0)) - 2.0 * mu_2 *
        (sigma /
        pow(alpha, 3.0))) * AlphadOuter[1][0] + mu_2 * (sigma / pow(alpha, 2.0)) * alphadd[1][0],
        (pow(
          mu_2,
          2.0) *
        (sigma /
        pow(
          alpha,
          4.0)) - 2.0 * mu_2 *
        (sigma /
        pow(
          alpha,
          3.0))) * AlphadOuter[1][1] + mu_2 * (sigma / pow(alpha, 2.0)) * alphadd[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = sigma;
  Output.Gradient = sigmad;
  Output.Hessian = sigmadd;

  return Output;
}


OutputStructScalar polygonGammaSwitch(
  std::vector<double> Position, PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes the value, gradient and hessian of the gamma-switch for a point x outside a polygon
   *
   * Input:
   *  1) Position: Point to consider
   *  2) PolygonUsed: Description of the polygon
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Unwrap parameters
  double mu_2 = DiffeoParams.get_mu_2();

  // Make position a point
  point PositionPoint(Position[0], Position[1]);

  // Unwrap center properties
  point CenterPoint = PolygonUsed.get_center();
  std::vector<double> Center = {CenterPoint.get<0>(), CenterPoint.get<1>()};

  // Compute the value of gamma and its gradient and hessian
  OutputStructScalar GammaOutput = polygonInsideImplicit(Position, PolygonUsed, DiffeoParams);
  double gamma = GammaOutput.Value;
  std::vector<double> gammad = GammaOutput.Gradient;
  std::vector<std::vector<double>> gammadd = GammaOutput.Hessian;

  // Compute the value of the switch
  double sigma;
  std::vector<double> sigmad;
  std::vector<std::vector<double>> sigmadd;
  if (gamma <= 0.01) {
    sigma = 0.0;
    sigmad = {0.0, 0.0};
    sigmadd = {{0.0, 0.0}, {0.0, 0.0}};
  } else {
    // Compute the value of alpha and its gradient and hessian
    double nom = gamma;
    double denom = bg::distance(PositionPoint, CenterPoint);
    double alpha = nom / denom;

    std::vector<double> nomd = gammad;
    std::vector<double> denomd =
    {(1.0 / bg::distance(PositionPoint, CenterPoint)) * (Position[0] - Center[0]),
      (1.0 / bg::distance(PositionPoint, CenterPoint)) * (Position[1] - Center[1])};
    std::vector<double> alphad =
    {(1.0 / denom) * nomd[0] - (nom / pow(denom, 2.0)) * denomd[0],
      (1.0 / denom) * nomd[1] - (nom / pow(denom, 2.0)) * denomd[1]};

    std::vector<std::vector<double>> nomdd = gammadd;
    std::vector<double> PositionMinusCenter = {Position[0] - Center[0], Position[1] - Center[1]};
    std::vector<std::vector<double>> PositionPositionOuter = VectorOuterProduct(
      PositionMinusCenter,
      PositionMinusCenter);
    std::vector<std::vector<double>> denomdd =
    {{(1.0 /
      bg::distance(
        PositionPoint,
        CenterPoint)) -
      (1.0 / pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * PositionPositionOuter[0][0],
      -(1.0 /
      pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * PositionPositionOuter[0][1]},
      {-(1.0 / pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * PositionPositionOuter[1][0],
        (1.0 /
        bg::distance(
          PositionPoint,
          CenterPoint)) -
        (1.0 /
        pow(bg::distance(PositionPoint, CenterPoint), 3.0)) * PositionPositionOuter[1][1]}};
    std::vector<std::vector<double>> NomDenomOuter = VectorOuterProduct(nomd, denomd);
    std::vector<std::vector<double>> DenomNomOuter = VectorOuterProduct(denomd, nomd);
    std::vector<std::vector<double>> DenomDenomOuter = VectorOuterProduct(denomd, denomd);
    std::vector<std::vector<double>> alphadd = {{(1.0 / denom) * nomdd[0][0] - (1.0 / pow(
        denom,
        2.0)) *
      (NomDenomOuter[0][0] + DenomNomOuter[0][0]) + 2.0 *
      (nom / pow(denom, 3.0)) * DenomDenomOuter[0][0] - (nom / pow(denom, 2.0)) * denomdd[0][0],
      (1.0 / denom) * nomdd[0][1] -
      (1.0 /
      pow(
        denom,
        2.0)) * (NomDenomOuter[0][1] + DenomNomOuter[0][1]) + 2.0 *
      (nom /
      pow(denom, 3.0)) * DenomDenomOuter[0][1] - (nom / pow(denom, 2.0)) * denomdd[0][1]},
      {(1.0 / denom) * nomdd[1][0] -
        (1.0 /
        pow(
          denom,
          2.0)) * (NomDenomOuter[1][0] + DenomNomOuter[1][0]) + 2.0 *
        (nom / pow(denom, 3.0)) * DenomDenomOuter[1][0] - (nom / pow(denom, 2.0)) * denomdd[1][0],
        (1.0 / denom) * nomdd[1][1] -
        (1.0 /
        pow(
          denom,
          2.0)) * (NomDenomOuter[1][1] + DenomNomOuter[1][1]) + 2.0 *
        (nom /
        pow(denom, 3.0)) * DenomDenomOuter[1][1] - (nom / pow(denom, 2.0)) * denomdd[1][1]}};

    sigma = exp(-mu_2 / alpha);
    sigmad =
    {mu_2 * (sigma / pow(alpha, 2.0)) * alphad[0], mu_2 * (sigma / pow(alpha, 2.0)) * alphad[1]};
    std::vector<std::vector<double>> AlphadOuter = VectorOuterProduct(alphad, alphad);
    sigmadd =
    {{(pow(
        mu_2,
        2.0) *
      (sigma /
      pow(
        alpha,
        4.0)) - 2.0 * mu_2 *
      (sigma /
      pow(alpha, 3.0))) * AlphadOuter[0][0] + mu_2 * (sigma / pow(alpha, 2.0)) * alphadd[0][0],
      (pow(
        mu_2,
        2.0) *
      (sigma /
      pow(
        alpha,
        4.0)) - 2.0 * mu_2 *
      (sigma /
      pow(alpha, 3.0))) * AlphadOuter[0][1] + mu_2 * (sigma / pow(alpha, 2.0)) * alphadd[0][1]},
      {(pow(
          mu_2,
          2.0) *
        (sigma /
        pow(
          alpha,
          4.0)) - 2.0 * mu_2 *
        (sigma /
        pow(alpha, 3.0))) * AlphadOuter[1][0] + mu_2 * (sigma / pow(alpha, 2.0)) * alphadd[1][0],
        (pow(
          mu_2,
          2.0) *
        (sigma /
        pow(
          alpha,
          4.0)) - 2.0 * mu_2 *
        (sigma /
        pow(
          alpha,
          3.0))) * AlphadOuter[1][1] + mu_2 * (sigma / pow(alpha, 2.0)) * alphadd[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = sigma;
  Output.Gradient = sigmad;
  Output.Hessian = sigmadd;

  return Output;
}


OutputStructScalar triangleOutsideImplicit(
  std::vector<double> Position, TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes beta(x) (i.e., the R-function) for a point x outside a triangle, and its gradient and hessian
   *
   * Input:
   *  1) Position: Point to consider
   *  2) Triangle: Description of the triangle
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Unwrap parameters
  double p = DiffeoParams.get_p();

  // Make position a point
  point PositionPoint(Position[0], Position[1]);

  // Unwrap triangle properties
  uint16_t depth = Triangle.get_depth();
  std::vector<point> adj_edge = Triangle.get_adj_edge();
  point CenterPoint = Triangle.get_center();
  std::vector<double> Center = {CenterPoint.get<0>(), CenterPoint.get<1>()};

  // Get triangle vertices and normal vectors and convert them
  std::vector<point> TriangleVertexListPoint = Triangle.get_vertices();
  std::vector<point> NormalVectorListPoint = Triangle.get_r_n();
  std::vector<std::vector<double>> TriangleVertexList;
  std::vector<std::vector<double>> NormalVectorList;
  for (size_t i = 0; i < Triangle.get_vertices().size(); i++) {
    TriangleVertexList.push_back(
      {TriangleVertexListPoint[i].get<0>(), TriangleVertexListPoint[i].get<1>()});
    NormalVectorList.push_back(
      {NormalVectorListPoint[i].get<0>(),
        NormalVectorListPoint[i].get<1>()});
  }

  // Distinguish between the root triangle and all other triangles
  double beta;
  std::vector<double> betad;
  std::vector<std::vector<double>> betadd;
  if ((depth == 0) && adj_edge.empty()) {
    // Find the hyperplane values
    double hyperplane_1 = (Position[0] - TriangleVertexList[2][0]) * NormalVectorList[1][0] +
      (Position[1] - TriangleVertexList[2][1]) * NormalVectorList[1][1];
    double hyperplane_2 = (Position[0] - TriangleVertexList[2][0]) * NormalVectorList[2][0] +
      (Position[1] - TriangleVertexList[2][1]) * NormalVectorList[2][1];
    double hyperplane_3 = (Position[0] - TriangleVertexList[0][0]) * NormalVectorList[0][0] +
      (Position[1] - TriangleVertexList[0][1]) * NormalVectorList[0][1];

    // Compute the R-function
    double hyperplane_12 = hyperplane_1 + hyperplane_2 -
      pow(pow(hyperplane_1, p) + pow(hyperplane_2, p), (1.0 / p));
    double hyperplane_123 = hyperplane_12 + hyperplane_3 -
      pow(pow(hyperplane_12, p) + pow(hyperplane_3, p), (1.0 / p));
    beta = -hyperplane_123;

    // Compute the gradients
    std::vector<double> hyperplane_1d = {NormalVectorList[1][0], NormalVectorList[1][1]};
    std::vector<double> hyperplane_2d = {NormalVectorList[2][0], NormalVectorList[2][1]};
    std::vector<double> hyperplane_3d = {NormalVectorList[0][0], NormalVectorList[0][1]};
    std::vector<double> hyperplane_12d =
    {(1.0 -
      ((pow(
        hyperplane_1,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_1d[0] +
      (1.0 -
      ((pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(pow(hyperplane_1, p) + pow(hyperplane_2, p), (p - 1.0) / p)))) * hyperplane_2d[0],
      (1.0 -
      ((pow(
        hyperplane_1,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_1d[1] +
      (1.0 -
      ((pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(pow(hyperplane_1, p) + pow(hyperplane_2, p), (p - 1.0) / p)))) * hyperplane_2d[1]};
    std::vector<double> hyperplane_123d =
    {(1.0 -
      ((pow(
        hyperplane_12,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p)))) * hyperplane_12d[0] +
      (1.0 -
      ((pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(pow(hyperplane_12, p) + pow(hyperplane_3, p), (p - 1.0) / p)))) * hyperplane_3d[0],
      (1.0 -
      ((pow(
        hyperplane_12,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p)))) * hyperplane_12d[1] +
      (1.0 -
      ((pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(pow(hyperplane_12, p) + pow(hyperplane_3, p), (p - 1.0) / p)))) * hyperplane_3d[1]};
    betad = {-hyperplane_123d[0], -hyperplane_123d[1]};

    // Compute the hessian
    std::vector<std::vector<double>> hyperplane_1dd = {{0.0, 0.0}, {0.0, 0.0}};
    std::vector<std::vector<double>> hyperplane_2dd = {{0.0, 0.0}, {0.0, 0.0}};
    std::vector<std::vector<double>> hyperplane_3dd = {{0.0, 0.0}, {0.0, 0.0}};
    std::vector<std::vector<double>> Hyperplane1Hyperplane1Outer = VectorOuterProduct(
      hyperplane_1d,
      hyperplane_1d);
    std::vector<std::vector<double>> Hyperplane1Hyperplane2Outer = VectorOuterProduct(
      hyperplane_1d,
      hyperplane_2d);
    std::vector<std::vector<double>> Hyperplane2Hyperplane1Outer = VectorOuterProduct(
      hyperplane_2d,
      hyperplane_1d);
    std::vector<std::vector<double>> Hyperplane2Hyperplane2Outer = VectorOuterProduct(
      hyperplane_2d,
      hyperplane_2d);
    std::vector<std::vector<double>> hyperplane_12dd =
    {{(-(p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_1,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane1Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 1.0) *
      pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane2Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_1,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_1dd[0][0] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_2,
        p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_2,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane2Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 1.0) *
      pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane1Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_2dd[0][0],
      (-(p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_1,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane1Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 1.0) *
      pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane2Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_1,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_1dd[0][1] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_2,
        p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_2,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane2Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 1.0) *
      pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane1Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_2dd[0][1]},
      {(-(p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_1,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane1Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 1.0) *
        pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane2Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_1,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p)))) * hyperplane_1dd[1][0] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_2,
          p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_2,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane2Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 1.0) *
        pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane1Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p)))) * hyperplane_2dd[1][0],
        (-(p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_1,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane1Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 1.0) *
        pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane2Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_1,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p)))) * hyperplane_1dd[1][1] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_2,
          p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_2,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane2Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 1.0) *
        pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane1Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p)))) * hyperplane_2dd[1][1]}};
    std::vector<std::vector<double>> Hyperplane12Hyperplane12Outer = VectorOuterProduct(
      hyperplane_12d, hyperplane_12d);
    std::vector<std::vector<double>> Hyperplane12Hyperplane3Outer = VectorOuterProduct(
      hyperplane_12d, hyperplane_3d);
    std::vector<std::vector<double>> Hyperplane3Hyperplane12Outer = VectorOuterProduct(
      hyperplane_3d, hyperplane_12d);
    std::vector<std::vector<double>> Hyperplane3Hyperplane3Outer = VectorOuterProduct(
      hyperplane_3d,
      hyperplane_3d);
    std::vector<std::vector<double>> hyperplane_123dd =
    {{(-(p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_12,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane12Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 1.0) *
      pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane3Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_12,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p)))) * hyperplane_12dd[0][0] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_3,
        p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_3,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane3Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 1.0) *
      pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane12Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p)))) * hyperplane_3dd[0][0],
      (-(p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_12,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane12Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 1.0) *
      pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane3Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_12,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p)))) * hyperplane_12dd[0][1] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_3,
        p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_3,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane3Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 1.0) *
      pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane12Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_3, p),
        (p - 1.0) / p)))) * hyperplane_3dd[0][1]},
      {(-(p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_12,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane12Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 1.0) *
        pow(
          hyperplane_3,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane3Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_12,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          (p - 1.0) / p)))) * hyperplane_12dd[1][0] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_3,
          p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_3,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane3Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 1.0) *
        pow(
          hyperplane_3,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane12Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_3,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          (p - 1.0) / p)))) * hyperplane_3dd[1][0],
        (-(p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_12,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane12Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 1.0) *
        pow(
          hyperplane_3,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane3Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_12,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          (p - 1.0) / p)))) * hyperplane_12dd[1][1] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_3,
          p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_3,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane3Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 1.0) *
        pow(
          hyperplane_3,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane12Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_3,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_3, p),
          (p - 1.0) / p)))) * hyperplane_3dd[1][1]}};
    betadd =
    {{-hyperplane_123dd[0][0], -hyperplane_123dd[0][1]},
      {-hyperplane_123dd[1][0], -hyperplane_123dd[1][1]}};
  } else {
    // Find the center hyperplane normals
    std::vector<point> RCenterNormalVectorListPoint = Triangle.get_r_center_n();
    std::vector<std::vector<double>> RCenterNormalVectorList;
    for (size_t i = 0; i < RCenterNormalVectorListPoint.size(); i++) {
      RCenterNormalVectorList.push_back(
        {RCenterNormalVectorListPoint[i].get<0>(), RCenterNormalVectorListPoint[i].get<1>()});
    }

    // Find the hyperplane values
    double hyperplane_1 = (Position[0] - TriangleVertexList[2][0]) * NormalVectorList[1][0] +
      (Position[1] - TriangleVertexList[2][1]) * NormalVectorList[1][1];
    double hyperplane_2 = (Position[0] - TriangleVertexList[2][0]) * NormalVectorList[2][0] +
      (Position[1] - TriangleVertexList[2][1]) * NormalVectorList[2][1];
    double hyperplane_3 = (Position[0] - Center[0]) * RCenterNormalVectorList[0][0] +
      (Position[1] - Center[1]) * RCenterNormalVectorList[0][1];
    double hyperplane_4 = (Position[0] - Center[0]) * RCenterNormalVectorList[1][0] +
      (Position[1] - Center[1]) * RCenterNormalVectorList[1][1];

    // Compute the R-function
    double hyperplane_12 = hyperplane_1 + hyperplane_2 -
      pow(pow(hyperplane_1, p) + pow(hyperplane_2, p), (1.0 / p));
    double hyperplane_34 = hyperplane_3 + hyperplane_4 -
      pow(pow(hyperplane_3, p) + pow(hyperplane_4, p), (1.0 / p));
    double hyperplane_1234 = hyperplane_12 + hyperplane_34 -
      pow(pow(hyperplane_12, p) + pow(hyperplane_34, p), (1.0 / p));
    beta = -hyperplane_1234;

    // Compute the gradients
    std::vector<double> hyperplane_1d = NormalVectorList[1];
    std::vector<double> hyperplane_2d = NormalVectorList[2];
    std::vector<double> hyperplane_3d = RCenterNormalVectorList[0];
    std::vector<double> hyperplane_4d = RCenterNormalVectorList[1];
    std::vector<double> hyperplane_12d =
    {(1.0 -
      ((pow(
        hyperplane_1,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_1d[0] +
      (1.0 -
      ((pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(pow(hyperplane_1, p) + pow(hyperplane_2, p), (p - 1.0) / p)))) * hyperplane_2d[0],
      (1.0 -
      ((pow(
        hyperplane_1,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_1d[1] +
      (1.0 -
      ((pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(pow(hyperplane_1, p) + pow(hyperplane_2, p), (p - 1.0) / p)))) * hyperplane_2d[1]};
    std::vector<double> hyperplane_34d =
    {(1.0 -
      ((pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p)))) * hyperplane_3d[0] +
      (1.0 -
      ((pow(
        hyperplane_4,
        p - 1.0)) /
      (pow(pow(hyperplane_3, p) + pow(hyperplane_4, p), (p - 1.0) / p)))) * hyperplane_4d[0],
      (1.0 -
      ((pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p)))) * hyperplane_3d[1] +
      (1.0 -
      ((pow(
        hyperplane_4,
        p - 1.0)) /
      (pow(pow(hyperplane_3, p) + pow(hyperplane_4, p), (p - 1.0) / p)))) * hyperplane_4d[1]};
    std::vector<double> hyperplane_1234d =
    {(1.0 -
      ((pow(
        hyperplane_12,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p)))) * hyperplane_12d[0] +
      (1.0 -
      ((pow(
        hyperplane_34,
        p - 1.0)) /
      (pow(pow(hyperplane_12, p) + pow(hyperplane_34, p), (p - 1.0) / p)))) * hyperplane_34d[0],
      (1.0 -
      ((pow(
        hyperplane_12,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p)))) * hyperplane_12d[1] +
      (1.0 -
      ((pow(
        hyperplane_34,
        p - 1.0)) /
      (pow(pow(hyperplane_12, p) + pow(hyperplane_34, p), (p - 1.0) / p)))) * hyperplane_34d[1]};
    betad = {-hyperplane_1234d[0], -hyperplane_1234d[1]};

    // Compute the hessian
    std::vector<std::vector<double>> hyperplane_1dd = {{0.0, 0.0}, {0.0, 0.0}};
    std::vector<std::vector<double>> hyperplane_2dd = {{0.0, 0.0}, {0.0, 0.0}};
    std::vector<std::vector<double>> hyperplane_3dd = {{0.0, 0.0}, {0.0, 0.0}};
    std::vector<std::vector<double>> hyperplane_4dd = {{0.0, 0.0}, {0.0, 0.0}};
    std::vector<std::vector<double>> Hyperplane1Hyperplane1Outer = VectorOuterProduct(
      hyperplane_1d,
      hyperplane_1d);
    std::vector<std::vector<double>> Hyperplane1Hyperplane2Outer = VectorOuterProduct(
      hyperplane_1d,
      hyperplane_2d);
    std::vector<std::vector<double>> Hyperplane2Hyperplane1Outer = VectorOuterProduct(
      hyperplane_2d,
      hyperplane_1d);
    std::vector<std::vector<double>> Hyperplane2Hyperplane2Outer = VectorOuterProduct(
      hyperplane_2d,
      hyperplane_2d);
    std::vector<std::vector<double>> hyperplane_12dd =
    {{(-(p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_1,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane1Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 1.0) *
      pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane2Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_1,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_1dd[0][0] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_2,
        p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_2,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane2Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 1.0) *
      pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane1Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_2dd[0][0],
      (-(p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_1,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane1Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 1.0) *
      pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane2Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_1,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_1dd[0][1] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_2,
        p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_2,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane2Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_1,
        p - 1.0) *
      pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane1Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_2,
        p - 1.0)) /
      (pow(
        pow(hyperplane_1, p) + pow(hyperplane_2, p),
        (p - 1.0) / p)))) * hyperplane_2dd[0][1]},
      {(-(p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_1,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane1Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 1.0) *
        pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane2Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_1,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p)))) * hyperplane_1dd[1][0] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_2,
          p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_2,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane2Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 1.0) *
        pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane1Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p)))) * hyperplane_2dd[1][0],
        (-(p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_1,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane1Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 1.0) *
        pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane1Hyperplane2Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_1,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p)))) * hyperplane_1dd[1][1] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_2,
          p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_2,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane2Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_1,
          p - 1.0) *
        pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane2Hyperplane1Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_2,
          p - 1.0)) /
        (pow(
          pow(hyperplane_1, p) + pow(hyperplane_2, p),
          (p - 1.0) / p)))) * hyperplane_2dd[1][1]}};

    std::vector<std::vector<double>> Hyperplane3Hyperplane3Outer = VectorOuterProduct(
      hyperplane_3d,
      hyperplane_3d);
    std::vector<std::vector<double>> Hyperplane3Hyperplane4Outer = VectorOuterProduct(
      hyperplane_3d,
      hyperplane_4d);
    std::vector<std::vector<double>> Hyperplane4Hyperplane3Outer = VectorOuterProduct(
      hyperplane_4d,
      hyperplane_3d);
    std::vector<std::vector<double>> Hyperplane4Hyperplane4Outer = VectorOuterProduct(
      hyperplane_4d,
      hyperplane_4d);
    std::vector<std::vector<double>> hyperplane_34dd =
    {{(-(p - 1.0) *
      ((pow(
        hyperplane_3,
        p - 2.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_3,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane3Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_3,
        p - 1.0) *
      pow(
        hyperplane_4,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane4Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p)))) * hyperplane_3dd[0][0] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_4,
        p - 2.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_4,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane4Hyperplane4Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_3,
        p - 1.0) *
      pow(
        hyperplane_4,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane4Hyperplane3Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_4,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p)))) * hyperplane_4dd[0][0],
      (-(p - 1.0) *
      ((pow(
        hyperplane_3,
        p - 2.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_3,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane3Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_3,
        p - 1.0) *
      pow(
        hyperplane_4,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane4Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_3,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p)))) * hyperplane_3dd[0][1] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_4,
        p - 2.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_4,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane4Hyperplane4Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_3,
        p - 1.0) *
      pow(
        hyperplane_4,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane4Hyperplane3Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_4,
        p - 1.0)) /
      (pow(
        pow(hyperplane_3, p) + pow(hyperplane_4, p),
        (p - 1.0) / p)))) * hyperplane_4dd[0][1]},
      {(-(p - 1.0) *
        ((pow(
          hyperplane_3,
          p - 2.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_3,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane3Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_3,
          p - 1.0) *
        pow(
          hyperplane_4,
          p - 1.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane4Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_3,
          p - 1.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          (p - 1.0) / p)))) * hyperplane_3dd[1][0] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_4,
          p - 2.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_4,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane4Hyperplane4Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_3,
          p - 1.0) *
        pow(
          hyperplane_4,
          p - 1.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane4Hyperplane3Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_4,
          p - 1.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          (p - 1.0) / p)))) * hyperplane_4dd[1][0],
        (-(p - 1.0) *
        ((pow(
          hyperplane_3,
          p - 2.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_3,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane3Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_3,
          p - 1.0) *
        pow(
          hyperplane_4,
          p - 1.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane3Hyperplane4Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_3,
          p - 1.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          (p - 1.0) / p)))) * hyperplane_3dd[1][1] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_4,
          p - 2.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_4,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane4Hyperplane4Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_3,
          p - 1.0) *
        pow(
          hyperplane_4,
          p - 1.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane4Hyperplane3Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_4,
          p - 1.0)) /
        (pow(
          pow(hyperplane_3, p) + pow(hyperplane_4, p),
          (p - 1.0) / p)))) * hyperplane_4dd[1][1]}};

    std::vector<std::vector<double>> Hyperplane12Hyperplane12Outer = VectorOuterProduct(
      hyperplane_12d, hyperplane_12d);
    std::vector<std::vector<double>> Hyperplane12Hyperplane34Outer = VectorOuterProduct(
      hyperplane_12d, hyperplane_34d);
    std::vector<std::vector<double>> Hyperplane34Hyperplane12Outer = VectorOuterProduct(
      hyperplane_34d, hyperplane_12d);
    std::vector<std::vector<double>> Hyperplane34Hyperplane34Outer = VectorOuterProduct(
      hyperplane_34d, hyperplane_34d);
    std::vector<std::vector<double>> hyperplane_1234dd =
    {{(-(p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_12,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane12Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 1.0) *
      pow(
        hyperplane_34,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane34Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_12,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p)))) * hyperplane_12dd[0][0] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_34,
        p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_34,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane34Hyperplane34Outer[0][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 1.0) *
      pow(
        hyperplane_34,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane34Hyperplane12Outer[0][0] +
      (1.0 -
      ((pow(
        hyperplane_34,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p)))) * hyperplane_34dd[0][0],
      (-(p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_12,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane12Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 1.0) *
      pow(
        hyperplane_34,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane34Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_12,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p)))) * hyperplane_12dd[0][1] +
      (-(p - 1.0) *
      ((pow(
        hyperplane_34,
        p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane_34,
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane34Hyperplane34Outer[0][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane_12,
        p - 1.0) *
      pow(
        hyperplane_34,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        1.0 + (p - 1.0) / p)))) * Hyperplane34Hyperplane12Outer[0][1] +
      (1.0 -
      ((pow(
        hyperplane_34,
        p - 1.0)) /
      (pow(
        pow(hyperplane_12, p) + pow(hyperplane_34, p),
        (p - 1.0) / p)))) * hyperplane_34dd[0][1]},
      {(-(p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_12,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane12Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 1.0) *
        pow(
          hyperplane_34,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane34Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_12,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          (p - 1.0) / p)))) * hyperplane_12dd[1][0] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_34,
          p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_34,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane34Hyperplane34Outer[1][0] +
        ((p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 1.0) *
        pow(
          hyperplane_34,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane34Hyperplane12Outer[1][0] +
        (1.0 -
        ((pow(
          hyperplane_34,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          (p - 1.0) / p)))) * hyperplane_34dd[1][0],
        (-(p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_12,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane12Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 1.0) *
        pow(
          hyperplane_34,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane12Hyperplane34Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_12,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          (p - 1.0) / p)))) * hyperplane_12dd[1][1] +
        (-(p - 1.0) *
        ((pow(
          hyperplane_34,
          p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane_34,
          2.0 * p - 2.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane34Hyperplane34Outer[1][1] +
        ((p - 1.0) *
        ((pow(
          hyperplane_12,
          p - 1.0) *
        pow(
          hyperplane_34,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          1.0 + (p - 1.0) / p)))) * Hyperplane34Hyperplane12Outer[1][1] +
        (1.0 -
        ((pow(
          hyperplane_34,
          p - 1.0)) /
        (pow(
          pow(hyperplane_12, p) + pow(hyperplane_34, p),
          (p - 1.0) / p)))) * hyperplane_34dd[1][1]}};
    betadd =
    {{-hyperplane_1234dd[0][0], -hyperplane_1234dd[0][1]},
      {-hyperplane_1234dd[1][0], -hyperplane_1234dd[1][1]}};
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = beta;
  Output.Gradient = betad;
  Output.Hessian = betadd;

  return Output;
}


OutputStructScalar polygonOutsideImplicit(
  std::vector<double> Position, PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes beta(x) (i.e., the R-function) for a point x outside a polygon, and its gradient and hessian
   *
   * Input:
   *  1) Position: Point to consider
   *  2) PolygonUsed: Description of the polygon
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Unwrap parameters
  double p = DiffeoParams.get_p();

  // Make position a point
  point PositionPoint(Position[0], Position[1]);

  // Get polygon vertices and normal vectors and convert them
  std::vector<point> PolygonVertexListPoint = PolygonUsed.get_augmented_vertices();
  std::vector<point> NormalVectorListPoint = PolygonUsed.get_r_n();
  std::vector<std::vector<double>> PolygonVertexList;
  std::vector<std::vector<double>> NormalVectorList;
  for (size_t i = 0; i < PolygonVertexListPoint.size(); i++) {
    PolygonVertexList.push_back(
      {PolygonVertexListPoint[i].get<0>(), PolygonVertexListPoint[i].get<1>()});
    NormalVectorList.push_back(
      {NormalVectorListPoint[i].get<0>(),
        NormalVectorListPoint[i].get<1>()});
  }

  // Compute hyperplane functions
  std::vector<double> hyperplane(PolygonVertexList.size(), 0.0);
  for (size_t i = 0; i < PolygonVertexList.size(); i++) {
    hyperplane[i] = (Position[0] - PolygonVertexList[i][0]) * NormalVectorList[i][0] +
      (Position[1] - PolygonVertexList[i][1]) * NormalVectorList[i][1];
  }

  // Compute the R-function and its gradient and hessian
  std::vector<std::vector<double>> N0N0Outer = VectorOuterProduct(
    NormalVectorList[0],
    NormalVectorList[0]);
  std::vector<std::vector<double>> N0N1Outer = VectorOuterProduct(
    NormalVectorList[0],
    NormalVectorList[1]);
  std::vector<std::vector<double>> N1N0Outer = VectorOuterProduct(
    NormalVectorList[1],
    NormalVectorList[0]);
  std::vector<std::vector<double>> N1N1Outer = VectorOuterProduct(
    NormalVectorList[1],
    NormalVectorList[1]);
  std::vector<std::vector<double>> betadd =
  {{(-(p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[0],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N0Outer[0][0] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N1Outer[0][0] +
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * 0.0 +
    (-(p - 1.0) *
    ((pow(
      hyperplane[1],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[1],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N1Outer[0][0] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N0Outer[0][0] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0,
    (-(p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[0],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N0Outer[0][1] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N1Outer[0][1] +
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * 0.0 +
    (-(p - 1.0) *
    ((pow(
      hyperplane[1],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[1],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N1Outer[0][1] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N0Outer[0][1] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0},
    {(-(p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[0],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N0Outer[1][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N1Outer[1][0] +
      (1.0 -
      ((pow(
        hyperplane[0],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p)))) * 0.0 +
      (-(p - 1.0) *
      ((pow(
        hyperplane[1],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[1],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N1Outer[1][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N0Outer[1][0] +
      (1.0 -
      ((pow(
        hyperplane[1],
        p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0,
      (-(p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[0],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N0Outer[1][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N1Outer[1][1] +
      (1.0 -
      ((pow(
        hyperplane[0],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p)))) * 0.0 +
      (-(p - 1.0) *
      ((pow(
        hyperplane[1],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[1],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N1Outer[1][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N0Outer[1][1] +
      (1.0 -
      ((pow(
        hyperplane[1],
        p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0}};

  std::vector<double> betad =
  {(1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorList[0][0] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorList[1][0],
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorList[0][1] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorList[1][1]};

  double beta = hyperplane[0] + hyperplane[1] - pow(
    pow(hyperplane[0], p) + pow(
      hyperplane[1],
      p), 1.0 / p);

  for (size_t i = 2; i < hyperplane.size(); i++) {
    std::vector<std::vector<double>> BetadBetadOuter = VectorOuterProduct(betad, betad);
    std::vector<std::vector<double>> BetaNiOuter = VectorOuterProduct(betad, NormalVectorList[i]);
    std::vector<std::vector<double>> NiBetadOuter = VectorOuterProduct(NormalVectorList[i], betad);
    std::vector<std::vector<double>> NiNiOuter = VectorOuterProduct(
      NormalVectorList[i],
      NormalVectorList[i]);
    betadd =
    {{(-(p - 1.0) *
      ((pow(
        beta,
        p - 2.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        beta,
        2.0 * p - 2.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * BetadBetadOuter[0][0] +
      ((p - 1.0) *
      ((pow(
        beta,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * BetaNiOuter[0][0] +
      (1.0 -
      ((pow(
        beta,
        p - 1.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * betadd[0][0] +
      (-(p - 1.0) *
      ((pow(
        hyperplane[i],
        p - 2.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[i],
        2.0 * p - 2.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiNiOuter[0][0] +
      ((p - 1.0) *
      ((pow(
        beta,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiBetadOuter[0][0] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) / (pow(pow(beta, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0,
      (-(p - 1.0) *
      ((pow(
        beta,
        p - 2.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        beta,
        2.0 * p - 2.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * BetadBetadOuter[0][1] +
      ((p - 1.0) *
      ((pow(
        beta,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * BetaNiOuter[0][1] +
      (1.0 -
      ((pow(
        beta,
        p - 1.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * betadd[0][1] +
      (-(p - 1.0) *
      ((pow(
        hyperplane[i],
        p - 2.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[i],
        2.0 * p - 2.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiNiOuter[0][1] +
      ((p - 1.0) *
      ((pow(
        beta,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiBetadOuter[0][1] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) / (pow(pow(beta, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0},
      {(-(p - 1.0) *
        ((pow(
          beta,
          p - 2.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          beta,
          2.0 * p - 2.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * BetadBetadOuter[1][0] +
        ((p - 1.0) *
        ((pow(
          beta,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * BetaNiOuter[1][0] +
        (1.0 -
        ((pow(
          beta,
          p - 1.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          (p - 1.0) / p)))) * betadd[1][0] +
        (-(p - 1.0) *
        ((pow(
          hyperplane[i],
          p - 2.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane[i],
          2.0 * p - 2.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiNiOuter[1][0] +
        ((p - 1.0) *
        ((pow(
          beta,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiBetadOuter[1][0] +
        (1.0 -
        ((pow(
          hyperplane[i],
          p - 1.0)) / (pow(pow(beta, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0,
        (-(p - 1.0) *
        ((pow(
          beta,
          p - 2.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          beta,
          2.0 * p - 2.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * BetadBetadOuter[1][1] +
        ((p - 1.0) *
        ((pow(
          beta,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * BetaNiOuter[1][1] +
        (1.0 -
        ((pow(
          beta,
          p - 1.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          (p - 1.0) / p)))) * betadd[1][1] +
        (-(p - 1.0) *
        ((pow(
          hyperplane[i],
          p - 2.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane[i],
          2.0 * p - 2.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiNiOuter[1][1] +
        ((p - 1.0) *
        ((pow(
          beta,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(beta, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiBetadOuter[1][1] +
        (1.0 -
        ((pow(
          hyperplane[i],
          p - 1.0)) / (pow(pow(beta, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0}};

    betad =
    {(1.0 -
      ((pow(
        beta,
        p - 1.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * betad[0] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(pow(beta, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * NormalVectorList[i][0],
      (1.0 -
      ((pow(
        beta,
        p - 1.0)) /
      (pow(
        pow(beta, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * betad[1] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(pow(beta, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * NormalVectorList[i][1]};

    beta = beta + hyperplane[i] - pow(pow(beta, p) + pow(hyperplane[i], p), 1.0 / p);
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = -beta;
  Output.Gradient = {-betad[0], -betad[1]};
  Output.Hessian = {{-betadd[0][0], -betadd[0][1]}, {-betadd[1][0], -betadd[1][1]}};

  return Output;
}


OutputStructScalar triangleInsideImplicit(
  std::vector<double> Position, TriangleClass Triangle,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes gamma(x) (i.e., the R-function) for a point x inside an enclosing polygon, and its gradient and hessian
   *
   * Input:
   *  1) Position: Point to consider
   *  2) Triangle: Description of the triangle
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Unwrap parameters
  double p = DiffeoParams.get_p();

  // Make position a point
  point PositionPoint(Position[0], Position[1]);

  // Get collar vertices and normal vectors and convert them
  std::vector<point> TriangleVertexTildeListPoint = Triangle.get_vertices_tilde();
  std::vector<point> NormalVectorTildeListPoint = Triangle.get_r_tilde_n();
  std::vector<std::vector<double>> TriangleVertexTildeList;
  std::vector<std::vector<double>> NormalVectorTildeList;
  for (size_t i = 0; i < TriangleVertexTildeListPoint.size(); i++) {
    TriangleVertexTildeList.push_back(
      {TriangleVertexTildeListPoint[i].get<0>(), TriangleVertexTildeListPoint[i].get<1>()});
    NormalVectorTildeList.push_back(
      {NormalVectorTildeListPoint[i].get<0>(), NormalVectorTildeListPoint[i].get<1>()});
  }

  // Compute hyperplane functions
  std::vector<double> hyperplane(TriangleVertexTildeList.size(), 0.0);
  for (size_t i = 0; i < TriangleVertexTildeList.size(); i++) {
    hyperplane[i] = (Position[0] - TriangleVertexTildeList[i][0]) * NormalVectorTildeList[i][0] +
      (Position[1] - TriangleVertexTildeList[i][1]) * NormalVectorTildeList[i][1];
  }

  // Compute the R-function and its gradient and hessian
  std::vector<std::vector<double>> N0N0Outer = VectorOuterProduct(
    NormalVectorTildeList[0],
    NormalVectorTildeList[0]);
  std::vector<std::vector<double>> N0N1Outer = VectorOuterProduct(
    NormalVectorTildeList[0],
    NormalVectorTildeList[1]);
  std::vector<std::vector<double>> N1N0Outer = VectorOuterProduct(
    NormalVectorTildeList[1],
    NormalVectorTildeList[0]);
  std::vector<std::vector<double>> N1N1Outer = VectorOuterProduct(
    NormalVectorTildeList[1],
    NormalVectorTildeList[1]);
  std::vector<std::vector<double>> gammadd =
  {{(-(p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[0],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N0Outer[0][0] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N1Outer[0][0] +
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * 0.0 +
    (-(p - 1.0) *
    ((pow(
      hyperplane[1],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[1],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N1Outer[0][0] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N0Outer[0][0] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0,
    (-(p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[0],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N0Outer[0][1] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N1Outer[0][1] +
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * 0.0 +
    (-(p - 1.0) *
    ((pow(
      hyperplane[1],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[1],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N1Outer[0][1] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N0Outer[0][1] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0},
    {(-(p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[0],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N0Outer[1][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N1Outer[1][0] +
      (1.0 -
      ((pow(
        hyperplane[0],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p)))) * 0.0 +
      (-(p - 1.0) *
      ((pow(
        hyperplane[1],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[1],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N1Outer[1][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N0Outer[1][0] +
      (1.0 -
      ((pow(
        hyperplane[1],
        p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0,
      (-(p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[0],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N0Outer[1][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N1Outer[1][1] +
      (1.0 -
      ((pow(
        hyperplane[0],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p)))) * 0.0 +
      (-(p - 1.0) *
      ((pow(
        hyperplane[1],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[1],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N1Outer[1][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N0Outer[1][1] +
      (1.0 -
      ((pow(
        hyperplane[1],
        p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0}};

  std::vector<double> gammad =
  {(1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorTildeList[0][0] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorTildeList[1][0],
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorTildeList[0][1] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorTildeList[1][1]};

  double gamma = hyperplane[0] + hyperplane[1] - pow(
    pow(hyperplane[0], p) + pow(
      hyperplane[1],
      p), 1.0 / p);

  for (size_t i = 2; i < hyperplane.size(); i++) {
    std::vector<std::vector<double>> GammadGammadOuter = VectorOuterProduct(gammad, gammad);
    std::vector<std::vector<double>> GammadNiOuter = VectorOuterProduct(
      gammad,
      NormalVectorTildeList[i]);
    std::vector<std::vector<double>> NiGammadOuter = VectorOuterProduct(
      NormalVectorTildeList[i],
      gammad);
    std::vector<std::vector<double>> NiNiOuter = VectorOuterProduct(
      NormalVectorTildeList[i],
      NormalVectorTildeList[i]);
    gammadd =
    {{(-(p - 1.0) *
      ((pow(
        gamma,
        p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        gamma,
        2.0 * p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * GammadGammadOuter[0][0] +
      ((p - 1.0) *
      ((pow(
        gamma,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * GammadNiOuter[0][0] +
      (1.0 -
      ((pow(
        gamma,
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * gammadd[0][0] +
      (-(p - 1.0) *
      ((pow(
        hyperplane[i],
        p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[i],
        2.0 * p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiNiOuter[0][0] +
      ((p - 1.0) *
      ((pow(
        gamma,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiGammadOuter[0][0] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) / (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0,
      (-(p - 1.0) *
      ((pow(
        gamma,
        p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        gamma,
        2.0 * p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * GammadGammadOuter[0][1] +
      ((p - 1.0) *
      ((pow(
        gamma,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * GammadNiOuter[0][1] +
      (1.0 -
      ((pow(
        gamma,
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * gammadd[0][1] +
      (-(p - 1.0) *
      ((pow(
        hyperplane[i],
        p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[i],
        2.0 * p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiNiOuter[0][1] +
      ((p - 1.0) *
      ((pow(
        gamma,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiGammadOuter[0][1] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) / (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0},
      {(-(p - 1.0) *
        ((pow(
          gamma,
          p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          gamma,
          2.0 * p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * GammadGammadOuter[1][0] +
        ((p - 1.0) *
        ((pow(
          gamma,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * GammadNiOuter[1][0] +
        (1.0 -
        ((pow(
          gamma,
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p)))) * gammadd[1][0] +
        (-(p - 1.0) *
        ((pow(
          hyperplane[i],
          p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane[i],
          2.0 * p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiNiOuter[1][0] +
        ((p - 1.0) *
        ((pow(
          gamma,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiGammadOuter[1][0] +
        (1.0 -
        ((pow(
          hyperplane[i],
          p - 1.0)) / (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0,
        (-(p - 1.0) *
        ((pow(
          gamma,
          p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          gamma,
          2.0 * p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * GammadGammadOuter[1][1] +
        ((p - 1.0) *
        ((pow(
          gamma,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * GammadNiOuter[1][1] +
        (1.0 -
        ((pow(
          gamma,
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p)))) * gammadd[1][1] +
        (-(p - 1.0) *
        ((pow(
          hyperplane[i],
          p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane[i],
          2.0 * p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiNiOuter[1][1] +
        ((p - 1.0) *
        ((pow(
          gamma,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiGammadOuter[1][1] +
        (1.0 -
        ((pow(
          hyperplane[i],
          p - 1.0)) / (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0}};

    gammad =
    {(1.0 -
      ((pow(
        gamma,
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * gammad[0] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * NormalVectorTildeList[i][0],
      (1.0 -
      ((pow(
        gamma,
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * gammad[1] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * NormalVectorTildeList[i][1]};

    gamma = gamma + hyperplane[i] - pow(pow(gamma, p) + pow(hyperplane[i], p), 1.0 / p);
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = gamma;
  Output.Gradient = gammad;
  Output.Hessian = gammadd;

  return Output;
}


OutputStructScalar polygonInsideImplicit(
  std::vector<double> Position, PolygonClass PolygonUsed,
  DiffeoParamsClass DiffeoParams)
{
  /**
   * Function that computes gamma(x) (i.e., the R-function) for a point x inside an enclosing polygon, and its gradient and hessian
   *
   * Input:
   *  1) Position: Point to consider
   *  2) PolygonUsed: Description of the polygon
   *  3) DiffeoParams: Options for the diffeomorphism construction
   *
   * Output:
   *  1) Output: Output struct containing the value, gradient and hessian
   */

  // Unwrap parameters
  double p = DiffeoParams.get_p();

  // Make position a point
  point PositionPoint(Position[0], Position[1]);

  // Get collar vertices and normal vectors and convert them
  std::vector<point> PolygonVertexTildeListPoint = PolygonUsed.get_vertices_tilde();
  std::vector<point> NormalVectorTildeListPoint = PolygonUsed.get_r_tilde_n();
  std::vector<std::vector<double>> PolygonVertexTildeList;
  std::vector<std::vector<double>> NormalVectorTildeList;
  for (size_t i = 0; i < PolygonVertexTildeListPoint.size(); i++) {
    PolygonVertexTildeList.push_back(
      {PolygonVertexTildeListPoint[i].get<0>(), PolygonVertexTildeListPoint[i].get<1>()});
    NormalVectorTildeList.push_back(
      {NormalVectorTildeListPoint[i].get<0>(), NormalVectorTildeListPoint[i].get<1>()});
  }

  // Compute hyperplane functions
  std::vector<double> hyperplane(PolygonVertexTildeList.size(), 0.0);
  for (size_t i = 0; i < PolygonVertexTildeList.size(); i++) {
    hyperplane[i] = (Position[0] - PolygonVertexTildeList[i][0]) * NormalVectorTildeList[i][0] +
      (Position[1] - PolygonVertexTildeList[i][1]) * NormalVectorTildeList[i][1];
  }

  // Compute the R-function and its gradient and hessian
  std::vector<std::vector<double>> N0N0Outer = VectorOuterProduct(
    NormalVectorTildeList[0],
    NormalVectorTildeList[0]);
  std::vector<std::vector<double>> N0N1Outer = VectorOuterProduct(
    NormalVectorTildeList[0],
    NormalVectorTildeList[1]);
  std::vector<std::vector<double>> N1N0Outer = VectorOuterProduct(
    NormalVectorTildeList[1],
    NormalVectorTildeList[0]);
  std::vector<std::vector<double>> N1N1Outer = VectorOuterProduct(
    NormalVectorTildeList[1],
    NormalVectorTildeList[1]);
  std::vector<std::vector<double>> gammadd =
  {{(-(p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[0],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N0Outer[0][0] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N1Outer[0][0] +
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * 0.0 +
    (-(p - 1.0) *
    ((pow(
      hyperplane[1],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[1],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N1Outer[0][0] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N0Outer[0][0] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0,
    (-(p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[0],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N0Outer[0][1] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N0N1Outer[0][1] +
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * 0.0 +
    (-(p - 1.0) *
    ((pow(
      hyperplane[1],
      p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p))) + (p - 1.0) *
    ((pow(
      hyperplane[1],
      2.0 * p - 2.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N1Outer[0][1] +
    ((p - 1.0) *
    ((pow(
      hyperplane[0],
      p - 1.0) *
    pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      1.0 + (p - 1.0) / p)))) * N1N0Outer[0][1] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0},
    {(-(p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[0],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N0Outer[1][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N1Outer[1][0] +
      (1.0 -
      ((pow(
        hyperplane[0],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p)))) * 0.0 +
      (-(p - 1.0) *
      ((pow(
        hyperplane[1],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[1],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N1Outer[1][0] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N0Outer[1][0] +
      (1.0 -
      ((pow(
        hyperplane[1],
        p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0,
      (-(p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[0],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N0Outer[1][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N0N1Outer[1][1] +
      (1.0 -
      ((pow(
        hyperplane[0],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p)))) * 0.0 +
      (-(p - 1.0) *
      ((pow(
        hyperplane[1],
        p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[1],
        2.0 * p - 2.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N1Outer[1][1] +
      ((p - 1.0) *
      ((pow(
        hyperplane[0],
        p - 1.0) *
      pow(
        hyperplane[1],
        p - 1.0)) /
      (pow(
        pow(hyperplane[0], p) + pow(hyperplane[1], p),
        1.0 + (p - 1.0) / p)))) * N1N0Outer[1][1] +
      (1.0 -
      ((pow(
        hyperplane[1],
        p - 1.0)) / (pow(pow(hyperplane[0], p) + pow(hyperplane[1], p), (p - 1.0) / p)))) * 0.0}};

  std::vector<double> gammad =
  {(1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorTildeList[0][0] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorTildeList[1][0],
    (1.0 -
    ((pow(
      hyperplane[0],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorTildeList[0][1] +
    (1.0 -
    ((pow(
      hyperplane[1],
      p - 1.0)) /
    (pow(
      pow(hyperplane[0], p) + pow(hyperplane[1], p),
      (p - 1.0) / p)))) * NormalVectorTildeList[1][1]};

  double gamma = hyperplane[0] + hyperplane[1] - pow(
    pow(hyperplane[0], p) + pow(
      hyperplane[1],
      p), 1.0 / p);

  for (size_t i = 2; i < hyperplane.size(); i++) {
    std::vector<std::vector<double>> GammadGammadOuter = VectorOuterProduct(gammad, gammad);
    std::vector<std::vector<double>> GammadNiOuter = VectorOuterProduct(
      gammad,
      NormalVectorTildeList[i]);
    std::vector<std::vector<double>> NiGammadOuter = VectorOuterProduct(
      NormalVectorTildeList[i],
      gammad);
    std::vector<std::vector<double>> NiNiOuter = VectorOuterProduct(
      NormalVectorTildeList[i],
      NormalVectorTildeList[i]);
    gammadd =
    {{(-(p - 1.0) *
      ((pow(
        gamma,
        p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        gamma,
        2.0 * p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * GammadGammadOuter[0][0] +
      ((p - 1.0) *
      ((pow(
        gamma,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * GammadNiOuter[0][0] +
      (1.0 -
      ((pow(
        gamma,
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * gammadd[0][0] +
      (-(p - 1.0) *
      ((pow(
        hyperplane[i],
        p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[i],
        2.0 * p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiNiOuter[0][0] +
      ((p - 1.0) *
      ((pow(
        gamma,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiGammadOuter[0][0] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) / (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0,
      (-(p - 1.0) *
      ((pow(
        gamma,
        p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        gamma,
        2.0 * p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * GammadGammadOuter[0][1] +
      ((p - 1.0) *
      ((pow(
        gamma,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * GammadNiOuter[0][1] +
      (1.0 -
      ((pow(
        gamma,
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * gammadd[0][1] +
      (-(p - 1.0) *
      ((pow(
        hyperplane[i],
        p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p))) + (p - 1.0) *
      ((pow(
        hyperplane[i],
        2.0 * p - 2.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiNiOuter[0][1] +
      ((p - 1.0) *
      ((pow(
        gamma,
        p - 1.0) *
      pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        1.0 + (p - 1.0) / p)))) * NiGammadOuter[0][1] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) / (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0},
      {(-(p - 1.0) *
        ((pow(
          gamma,
          p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          gamma,
          2.0 * p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * GammadGammadOuter[1][0] +
        ((p - 1.0) *
        ((pow(
          gamma,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * GammadNiOuter[1][0] +
        (1.0 -
        ((pow(
          gamma,
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p)))) * gammadd[1][0] +
        (-(p - 1.0) *
        ((pow(
          hyperplane[i],
          p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane[i],
          2.0 * p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiNiOuter[1][0] +
        ((p - 1.0) *
        ((pow(
          gamma,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiGammadOuter[1][0] +
        (1.0 -
        ((pow(
          hyperplane[i],
          p - 1.0)) / (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0,
        (-(p - 1.0) *
        ((pow(
          gamma,
          p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          gamma,
          2.0 * p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * GammadGammadOuter[1][1] +
        ((p - 1.0) *
        ((pow(
          gamma,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * GammadNiOuter[1][1] +
        (1.0 -
        ((pow(
          gamma,
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p)))) * gammadd[1][1] +
        (-(p - 1.0) *
        ((pow(
          hyperplane[i],
          p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          (p - 1.0) / p))) + (p - 1.0) *
        ((pow(
          hyperplane[i],
          2.0 * p - 2.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiNiOuter[1][1] +
        ((p - 1.0) *
        ((pow(
          gamma,
          p - 1.0) *
        pow(
          hyperplane[i],
          p - 1.0)) /
        (pow(
          pow(gamma, p) + pow(hyperplane[i], p),
          1.0 + (p - 1.0) / p)))) * NiGammadOuter[1][1] +
        (1.0 -
        ((pow(
          hyperplane[i],
          p - 1.0)) / (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * 0.0}};

    gammad =
    {(1.0 -
      ((pow(
        gamma,
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * gammad[0] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(pow(gamma, p) + pow(hyperplane[i], p), (p - 1.0) / p)))) * NormalVectorTildeList[i][0],
      (1.0 -
      ((pow(
        gamma,
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * gammad[1] +
      (1.0 -
      ((pow(
        hyperplane[i],
        p - 1.0)) /
      (pow(
        pow(gamma, p) + pow(hyperplane[i], p),
        (p - 1.0) / p)))) * NormalVectorTildeList[i][1]};

    gamma = gamma + hyperplane[i] - pow(pow(gamma, p) + pow(hyperplane[i], p), 1.0 / p);
  }

  // Construct the output
  OutputStructScalar Output;
  Output.Value = gamma;
  Output.Gradient = gammad;
  Output.Hessian = gammadd;

  return Output;
}
