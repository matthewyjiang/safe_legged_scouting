#include <reactive_planner_lib.h>
#include <cmath>

DiffeoTransformResult computeDiffeoTransform(
  std::vector<double> & robot_position, double robot_orientation,
  std::vector<std::vector<PolygonClass>> & diffeo_tree_array,
  DiffeoParamsClass & diffeo_params, rclcpp::Logger logger)
{

  std::vector<double> RobotPositionTransformed = {robot_position[0],
    robot_position[1]};
  std::vector<std::vector<double>> RobotPositionTransformedD = {{1.0, 0.0},
    {0.0, 1.0}};
  std::vector<double> RobotPositionTransformedDD = {0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0};


  for (size_t i = 0; i < diffeo_tree_array.size(); i++) {

    OutputStructVector TempTransformation = polygonDiffeoConvex(
      RobotPositionTransformed, diffeo_tree_array[i], diffeo_params);

    std::vector<double> TempPositionTransformed = TempTransformation.Value;
    std::vector<std::vector<double>> TempPositionTransformedD =
      TempTransformation.Jacobian;
    std::vector<double> TempPositionTransformedDD =
      TempTransformation.JacobianD;


    double res1 =
      TempPositionTransformedD[0][0] * RobotPositionTransformedDD[0] +
      TempPositionTransformedD[0][1] * RobotPositionTransformedDD[4] +
      RobotPositionTransformedD[0][0] *
      (TempPositionTransformedDD[0] * RobotPositionTransformedD[0][0] +
      TempPositionTransformedDD[1] * RobotPositionTransformedD[1][0]) +
      RobotPositionTransformedD[1][0] *
      (TempPositionTransformedDD[2] * RobotPositionTransformedD[0][0] +
      TempPositionTransformedDD[3] * RobotPositionTransformedD[1][0]);
    double res2 =
      TempPositionTransformedD[0][0] * RobotPositionTransformedDD[1] +
      TempPositionTransformedD[0][1] * RobotPositionTransformedDD[5] +
      RobotPositionTransformedD[0][0] *
      (TempPositionTransformedDD[0] * RobotPositionTransformedD[0][1] +
      TempPositionTransformedDD[1] * RobotPositionTransformedD[1][1]) +
      RobotPositionTransformedD[1][0] *
      (TempPositionTransformedDD[2] * RobotPositionTransformedD[0][1] +
      TempPositionTransformedDD[3] * RobotPositionTransformedD[1][1]);
    double res3 =
      TempPositionTransformedD[0][0] * RobotPositionTransformedDD[2] +
      TempPositionTransformedD[0][1] * RobotPositionTransformedDD[6] +
      RobotPositionTransformedD[0][1] *
      (TempPositionTransformedDD[0] * RobotPositionTransformedD[0][0] +
      TempPositionTransformedDD[1] * RobotPositionTransformedD[1][0]) +
      RobotPositionTransformedD[1][1] *
      (TempPositionTransformedDD[2] * RobotPositionTransformedD[0][0] +
      TempPositionTransformedDD[3] * RobotPositionTransformedD[1][0]);
    double res4 =
      TempPositionTransformedD[0][0] * RobotPositionTransformedDD[3] +
      TempPositionTransformedD[0][1] * RobotPositionTransformedDD[7] +
      RobotPositionTransformedD[0][1] *
      (TempPositionTransformedDD[0] * RobotPositionTransformedD[0][1] +
      TempPositionTransformedDD[1] * RobotPositionTransformedD[1][1]) +
      RobotPositionTransformedD[1][1] *
      (TempPositionTransformedDD[2] * RobotPositionTransformedD[0][1] +
      TempPositionTransformedDD[3] * RobotPositionTransformedD[1][1]);
    double res5 =
      TempPositionTransformedD[1][0] * RobotPositionTransformedDD[0] +
      TempPositionTransformedD[1][1] * RobotPositionTransformedDD[4] +
      RobotPositionTransformedD[0][0] *
      (TempPositionTransformedDD[4] * RobotPositionTransformedD[0][0] +
      TempPositionTransformedDD[5] * RobotPositionTransformedD[1][0]) +
      RobotPositionTransformedD[1][0] *
      (TempPositionTransformedDD[6] * RobotPositionTransformedD[0][0] +
      TempPositionTransformedDD[7] * RobotPositionTransformedD[1][0]);
    double res6 =
      TempPositionTransformedD[1][0] * RobotPositionTransformedDD[1] +
      TempPositionTransformedD[1][1] * RobotPositionTransformedDD[5] +
      RobotPositionTransformedD[0][0] *
      (TempPositionTransformedDD[4] * RobotPositionTransformedD[0][1] +
      TempPositionTransformedDD[5] * RobotPositionTransformedD[1][1]) +
      RobotPositionTransformedD[1][0] *
      (TempPositionTransformedDD[6] * RobotPositionTransformedD[0][1] +
      TempPositionTransformedDD[7] * RobotPositionTransformedD[1][1]);
    double res7 =
      TempPositionTransformedD[1][0] * RobotPositionTransformedDD[2] +
      TempPositionTransformedD[1][1] * RobotPositionTransformedDD[6] +
      RobotPositionTransformedD[0][1] *
      (TempPositionTransformedDD[4] * RobotPositionTransformedD[0][0] +
      TempPositionTransformedDD[5] * RobotPositionTransformedD[1][0]) +
      RobotPositionTransformedD[1][1] *
      (TempPositionTransformedDD[6] * RobotPositionTransformedD[0][0] +
      TempPositionTransformedDD[7] * RobotPositionTransformedD[1][0]);
    double res8 =
      TempPositionTransformedD[1][0] * RobotPositionTransformedDD[3] +
      TempPositionTransformedD[1][1] * RobotPositionTransformedDD[7] +
      RobotPositionTransformedD[0][1] *
      (TempPositionTransformedDD[4] * RobotPositionTransformedD[0][1] +
      TempPositionTransformedDD[5] * RobotPositionTransformedD[1][1]) +
      RobotPositionTransformedD[1][1] *
      (TempPositionTransformedDD[6] * RobotPositionTransformedD[0][1] +
      TempPositionTransformedDD[7] * RobotPositionTransformedD[1][1]);


    RobotPositionTransformedDD[0] = res1;
    RobotPositionTransformedDD[1] = res2;
    RobotPositionTransformedDD[2] = res3;
    RobotPositionTransformedDD[3] = res4;
    RobotPositionTransformedDD[4] = res5;
    RobotPositionTransformedDD[5] = res6;
    RobotPositionTransformedDD[6] = res7;
    RobotPositionTransformedDD[7] = res8;

    RobotPositionTransformedD = MatrixMatrixMultiplication(
      TempPositionTransformedD, RobotPositionTransformedD);
    RobotPositionTransformed = TempPositionTransformed;
  }


  // Find alpha1, alpha2, beta1, beta2
  double alpha1 = -(RobotPositionTransformedD[1][0] * cos(robot_orientation) +
    RobotPositionTransformedD[1][1] * sin(robot_orientation));
  double beta1 =
    RobotPositionTransformedDD[0] * pow(cos(robot_orientation), 2) +
    (RobotPositionTransformedDD[1] + RobotPositionTransformedDD[2]) *
    sin(robot_orientation) * cos(robot_orientation) +
    RobotPositionTransformedDD[3] * pow(sin(robot_orientation), 2);
  double alpha2 = RobotPositionTransformedD[0][0] * cos(robot_orientation) +
    RobotPositionTransformedD[0][1] * sin(robot_orientation);
  double beta2 =
    RobotPositionTransformedDD[4] * pow(cos(robot_orientation), 2) +
    (RobotPositionTransformedDD[5] + RobotPositionTransformedDD[6]) *
    sin(robot_orientation) * cos(robot_orientation) +
    RobotPositionTransformedDD[7] * pow(sin(robot_orientation), 2);


  // Find transformed orientation
  double y_component = RobotPositionTransformedD[1][0] * cos(robot_orientation) +
    RobotPositionTransformedD[1][1] * sin(robot_orientation);
  double x_component = RobotPositionTransformedD[0][0] * cos(robot_orientation) +
    RobotPositionTransformedD[0][1] * sin(robot_orientation);

  double RobotOrientationTransformed = atan2(y_component, x_component);


  DiffeoTransformResult result;
  result.transformed_position = RobotPositionTransformed;
  result.transformed_jacobian = RobotPositionTransformedD;
  result.transformed_hessian = RobotPositionTransformedDD;
  result.alpha1 = alpha1;
  result.alpha2 = alpha2;
  result.beta1 = beta1;
  result.beta2 = beta2;
  result.transformed_orientation = RobotOrientationTransformed;

  return result;
}
