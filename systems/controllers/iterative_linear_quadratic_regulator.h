#pragma once

#include <memory>

#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

/// A structure for storing the result of iLQR
struct IterativeLinearQuadraticRegulatorResult {
  Eigen::MatrixXd X;
  Eigen::MatrixXd U;
  double J;
};

struct BackwardPassResult {
    Eigen::MatrixXd K;
    Eigen::MatrixXd d;
    Eigen::MatrixXd DeltaV;
};

struct ForwardPassResult {
  Eigen::MatrixXd X;
  Eigen::MatrixXd U;
  double J;
};

// Add some helper functions for iLQR. Might include BackwardPass and ForwardPass
// and other functions for intermediate calculations...


/// Computes an optimal state and control trajectory using discrete iterative LQR
IterativeLinearQuadraticRegulatorResult DiscreteTimeIterativeLinearQuadraticRegulator(
  Eigen::MatrixXd X,
  Eigen::MatrixXd U
);

}  // namespace controllers
}  // namespace systems
}  // namespace drake
