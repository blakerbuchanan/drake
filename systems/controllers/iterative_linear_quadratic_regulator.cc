#include "drake/systems/controllers/iterative_linear_quadratic_regulator.h"

#include <Eigen/QR>

namespace drake {
namespace systems {
namespace controllers {

BackwardPassResult BackwardPass(Eigen::MatrixXd X, Eigen::MatrixXd d, Eigen::MatrixXd DeltaV)
{
  // Nothing here yet
  BackwardPassResult bp_result;
  return bp_result;
}

ForwardPassResult ForwardPass(Eigen::MatrixXd X, Eigen::MatrixXd U, Eigen::MatrixXd K, 
  Eigen::MatrixXd d, Eigen::MatrixXd DeltaV, double Jprev)
{
  // Nothing here yet
  ForwardPassResult fp_result;
  return fp_result;
}

IterativeLinearQuadraticRegulatorResult DiscreteTimeIterativeLinearQuadraticRegulator(
  Eigen::MatrixXd X,
  Eigen::MatrixXd U)
 {

  BackwardPassResult bp_result;
  ForwardPassResult fp_result;

  // Initialize X0, U, and tolerance
  // Simulate from X0 using U (need a forward rollout)

  // compute J (quadratic cost function) using X and U
  double tol = 0.0001;
  double J = 0;
  double Jprev = 0;
  Eigen::MatrixXd DeltaV;
  
  while (std::abs(J - Jprev) > tol)
  {
    Jprev = J;
    bp_result = BackwardPass(X, U, DeltaV);
    fp_result = ForwardPass(X, U, bp_result.K, bp_result.d, bp_result.DeltaV, Jprev);

  }

  
  IterativeLinearQuadraticRegulatorResult result;
  result.X = fp_result.X;
  result.U = fp_result.U;
  result.J = fp_result.J;

  return result;
}


}  // namespace controllers
}  // namespace systems
}  // namespace drake
